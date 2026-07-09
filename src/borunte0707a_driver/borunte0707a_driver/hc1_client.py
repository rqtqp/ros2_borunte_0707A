"""Pure-socket client for the HC1 RemoteMonitor / HCRemoteCommand interface.

No rclpy dependency on purpose: the payloads were validated live against the
controller, and keeping this module socket-only makes it testable and reusable
outside ROS (see `hc1_ping` for a standalone connectivity check).
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
        # Small request/reply payloads: Nagle only adds latency here. Keepalive
        # detects a silently dropped path (e.g. a WiFi hop or NAT forward going
        # away) instead of leaving a half-open session wedging the controller.
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
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
                return self._recv_json(conn)
            except (OSError, ValueError) as error:
                # ValueError covers a reply that never parsed (desync/garbage):
                # the connection must be dropped too, or leftover bytes would
                # corrupt every subsequent reply on the reused socket.
                last_error = error
                self._close()  # force a fresh connection on the next attempt
        suffix = " after reconnect" if retry else ""
        raise RuntimeError(f"HC1 request failed{suffix}: {last_error}") from last_error

    @staticmethod
    def _recv_json(conn: socket.socket) -> dict:
        """Read one JSON reply. The controller frames replies as a single JSON
        object with no delimiter, and TCP may deliver it split across segments
        (likely once a WiFi hop / forwarder is in the path) -- so accumulate
        until the buffer parses as complete JSON."""
        buffer = b""
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                raise ConnectionError("Controller closed the connection without a complete reply.")
            buffer += chunk
            try:
                return json.loads(buffer.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                if len(buffer) > 1 << 20:  # desync guard: replies are tiny
                    raise ValueError(f"unparseable reply exceeds 1 MiB ({len(buffer)} bytes)")
                continue  # partial reply; keep reading (recv timeout still applies)

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
