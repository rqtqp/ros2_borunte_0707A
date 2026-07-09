"""hc1_client.py against a fake controller socket: framing, desync recovery,
reconnect semantics, and the never-resend rule for motion (no ROS needed)."""
import json
import socket
import threading
import time

import pytest

from borunte0707a_driver.hc1_client import HC1Client


class FakeController:
    """One-connection fake HC1: applies a scripted behavior per request.

    Behaviors: 'whole' (reply in one segment), 'split' (reply fragmented into
    3 delayed segments), 'silent' (never reply), 'garbage' (unparseable bytes).
    """

    def __init__(self, behaviors):
        self.behaviors = list(behaviors)
        self.requests = 0
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(2)
        self.port = self._srv.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        conn = None
        for behavior in self.behaviors:
            while True:
                if conn is None:
                    conn, _ = self._srv.accept()
                req = conn.recv(65536)
                if req:
                    break
                conn.close()
                conn = None   # client reconnected; accept the new socket
            self.requests += 1
            addrs = json.loads(req.decode()).get("queryAddr", [])
            reply = json.dumps({
                "queryAddr": addrs,
                "queryData": [str(i) for i in range(len(addrs))],
            }).encode()
            if behavior == "whole":
                conn.sendall(reply)
            elif behavior == "split":
                third = len(reply) // 3
                for part in (reply[:third], reply[third:2 * third], reply[2 * third:]):
                    conn.sendall(part)
                    time.sleep(0.03)
            elif behavior == "garbage":
                conn.sendall(b"\xff\xfe not json at all \xff" * 4)
            elif behavior == "silent":
                pass
        if conn is not None:
            conn.close()
        self._srv.close()


ADDRS = [f"axis-{i}" for i in range(6)] + [f"curSpeed-{i}" for i in range(6)]


def test_whole_and_fragmented_replies_no_desync():
    srv = FakeController(["whole", "split", "whole"])
    with HC1Client("127.0.0.1", srv.port, timeout=2.0) as c:
        r1 = c.query(ADDRS)
        r2 = c.query(ADDRS)   # fragmented: must accumulate until it parses
        r3 = c.query(ADDRS)   # next reply on the same socket must stay in sync
    assert r1 == r3 and len(r2) == len(ADDRS)


def test_socket_options_set():
    srv = FakeController(["whole"])
    with HC1Client("127.0.0.1", srv.port, timeout=2.0) as c:
        c.query(["version"])
        assert c._conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) != 0
        assert c._conn.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE) != 0


def test_garbage_reply_closes_and_retries_read():
    # A read (retry=True) that gets garbage must drop the connection and retry
    # once on a fresh socket -- the second, clean reply wins.
    srv = FakeController(["garbage", "whole"])
    with HC1Client("127.0.0.1", srv.port, timeout=1.0) as c:
        r = c.query(ADDRS)
    assert len(r) == len(ADDRS)
    assert srv.requests == 2


def test_motion_never_resent_on_timeout():
    # AddRCC uses retry=False: a silent controller means ONE request on the
    # wire and a RuntimeError -- never a resend that could execute twice.
    srv = FakeController(["silent"])
    c = HC1Client("127.0.0.1", srv.port, timeout=0.4)
    with pytest.raises(RuntimeError):
        c.send_addrcc("svc", [{"action": "4"}], timeout=0.4)
    c.close()
    time.sleep(0.1)
    assert srv.requests == 1


def test_read_retries_once_on_timeout():
    srv = FakeController(["silent", "whole"])
    with HC1Client("127.0.0.1", srv.port, timeout=0.4) as c:
        r = c.query(ADDRS)
    assert len(r) == len(ADDRS)
    assert srv.requests == 2


def test_os_environ_overrides_env_file(monkeypatch):
    from borunte0707a_driver.env_config import load_env
    monkeypatch.setenv("ROBOT_IP", "192.0.2.77")
    assert load_env()["ROBOT_IP"] == "192.0.2.77"
