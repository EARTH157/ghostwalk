import socket
import time

import pytest
from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

from sim.protocol import decode_frame
from sim.server import TeleopServer
from tests.test_protocol import VALID


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def server():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = TeleopServer(port=port)
    srv.start()
    srv.url = f"ws://127.0.0.1:{port}"
    yield srv
    srv.stop()


def test_receives_latest_input(server):
    with connect(server.url) as c:
        c.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None)
        assert server.latest_input().right.trigger == 1.0


def test_stale_input_is_none(server):
    with connect(server.url) as c:
        c.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None)
        time.sleep(0.6)
        assert server.latest_input(0.5) is None


def test_bad_message_keeps_connection(server):
    with connect(server.url) as c:
        c.send("{")
        c.send(b"binary")
        c.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None)
        assert server.rejected == 2


def test_publish_frame_reaches_client(server):
    with connect(server.url) as c:
        c.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None)
        server.publish_frame({"seq": 7}, b"LL", b"RRR")
        h, l, r = decode_frame(c.recv(timeout=2))
        assert h["seq"] == 7 and l == b"LL" and r == b"RRR"


def test_new_client_replaces_old(server):
    with connect(server.url) as a, connect(server.url) as b:
        b.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None)
        server.publish_frame({"seq": 1}, b"L", b"R")
        assert decode_frame(b.recv(timeout=2))[0]["seq"] == 1
        with pytest.raises(ConnectionClosed):
            a.recv(timeout=2)


def test_silent_old_client_does_not_stall_new_one(server):
    # handshake, then never read: the server's close handshake gets no answer
    old = socket.create_connection(("127.0.0.1", int(server.url.rsplit(":", 1)[1])))
    old.sendall(b"GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n")
    assert b"101" in old.recv(1024)
    with connect(server.url) as b:
        b.send(VALID)
        assert wait_for(lambda: server.latest_input() is not None, 1.0)
    old.close()
