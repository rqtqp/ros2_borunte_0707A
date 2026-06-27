"""Pure-socket client for the HC1 RemoteMonitor / HCRemoteCommand interface.

No rclpy dependency on purpose: this mirrors the proven payloads in
`scripts/hc1_remote_preflight.py` (query) and `scripts/hc1_home.py` (AddRCC)
so the ROS 2 layer is a thin wrapper, not a re-implementation.
"""

from __future__ import annotations

import json
import socket

REMOTE_MONITOR_DSID = "www.hc-system.com.RemoteMonitor"


class HC1Client:
    """One reused TCP connection per client.

    The controller's RemoteMonitor answers many requests on a single persistent
    socket (verified), and its embedded socket pool is small -- opening a fresh
    connection per query floods it and eventually wedges the service. So we keep
    one connection open and reconnect transparently if it breaks. Call close()
    (or use as a context manager) on shutdown.
    """

    def __init__(self, host: str, port: int = 9760, timeout: float = 3.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._conn: socket.socket | None = None

    def _connect(self) -> socket.socket:
        self._close()
        conn = socket.create_connection((self.host, self.port), timeout=self.timeout)
        conn.settimeout(self.timeout)
        self._conn = conn
        return conn

    def _close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None

    def _send(self, payload: dict, retry: bool = True, timeout: float | None = None) -> dict:
        encoded = json.dumps(payload, separators=(",", ":")).encode("ascii")
        call_timeout = self.timeout if timeout is None else timeout
        # Reuse the open connection. retry=True (idempotent reads) reconnects once
        # on a stale/broken socket. retry=False (motion writes) NEVER resends: a
        # timed-out AddRCC may already have been received, so resending it could
        # execute the move twice.
        last_error: Exception | None = None
        for attempt in range(2 if retry else 1):
            try:
                conn = self._conn or self._connect()
                conn.settimeout(call_timeout)
                conn.sendall(encoded)
                response = conn.recv(65536)
                if not response:
                    raise ConnectionError("Controller closed the connection without a reply.")
                return json.loads(response.decode("utf-8"))
            except OSError as error:
                last_error = error
                self._close()  # force a fresh connection on the next attempt
        suffix = " after reconnect" if retry else ""
        raise RuntimeError(f"HC1 request failed{suffix}: {last_error}") from last_error

    def close(self) -> None:
        self._close()

    def __enter__(self) -> "HC1Client":
        return self

    def __exit__(self, *exc) -> None:
        self._close()

    def __del__(self) -> None:
        self._close()

    def query(self, addresses: list[str]) -> dict[str, str]:
        """Send a read-only query; return {address: value} mapping."""
        reply = self._send({
            "dsID": REMOTE_MONITOR_DSID,
            "packID": "ros2-query",
            "reqType": "query",
            "queryAddr": addresses,
        })
        keys = reply.get("queryAddr", addresses)
        values = reply.get("queryData", [])
        return dict(zip(keys, values))

    def send_addrcc(self, service_id: str, instructions: list[dict],
                    empty_list: str = "1", pack_id: str = "ros2-addrcc",
                    timeout: float | None = None) -> dict:
        """Send a motion instruction list. Used in phase 3; no safety gate here
        -- the caller (action server) is responsible for preconditions.

        retry=False: a timed-out motion command is NEVER resent (it may already
        have been received). Pass a longer timeout than for queries -- the
        controller can be slow to acknowledge while finishing a prior move."""
        return self._send({
            "dsID": service_id,
            "reqType": "AddRCC",
            "emptyList": empty_list,
            "packID": pack_id,
            "instructions": instructions,
        }, retry=False, timeout=timeout)

    def send_command(self, cmd: str, *args, timeout: float | None = None) -> dict:
        """Send a RemoteMonitor control command (cmdData), e.g. 'actionStop' to
        immediately halt the current motion. Not retried (a control action)."""
        reply = self._send({
            "dsID": REMOTE_MONITOR_DSID,
            "reqType": "command",
            "packID": "ros2-cmd",
            "cmdData": [cmd, *[str(a) for a in args]],
        }, retry=False, timeout=timeout)
        ok = "ok" in [str(x) for x in reply.get("cmdReply", [])]
        return {"ok": ok, "reply": reply}
