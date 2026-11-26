"""WebSocket bridge: headset input in, stereo frames out. Runs in a daemon thread beside the sync sim loop."""
import asyncio
import threading
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from sim.protocol import encode_frame, parse_input


class TeleopServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8765):
        self.host, self.port = host, port
        self.rejected = 0
        self._latest = None  # (InputMsg, monotonic receive time)
        self._client = None
        self._pending = None
        self._busy = False
        self._closing = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=lambda: asyncio.run(self._main()), daemon=True)

    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(5):
            raise RuntimeError(f"teleop server failed to start on {self.host}:{self.port}")

    def stop(self) -> None:
        self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(5)

    def latest_input(self, max_age_s: float = 0.5):
        latest = self._latest
        if latest is None or time.monotonic() - latest[1] > max_age_s:
            return None
        return latest[0]

    def publish_frame(self, header: dict, left_jpeg: bytes, right_jpeg: bytes) -> None:
        data = encode_frame(header, left_jpeg, right_jpeg)
        self._loop.call_soon_threadsafe(self._offer, data)

    async def _main(self):
        self._loop, self._stop = asyncio.get_running_loop(), asyncio.Event()
        async with serve(self._handle, self.host, self.port, max_size=65536, origins=["http://localhost:8000", None]):
            self._ready.set()
            await self._stop.wait()

    def _offer(self, data):
        if self._client is None:
            return
        self._pending = data  # overwrites an unsent older frame
        if not self._busy:
            asyncio.ensure_future(self._flush())

    async def _flush(self):
        self._busy = True
        try:
            while self._pending is not None and self._client is not None:
                data, self._pending = self._pending, None
                await self._client.send(data)
        except ConnectionClosed:
            pass
        finally:
            self._busy = False

    async def _handle(self, ws):
        old, self._client, self._pending = self._client, ws, None
        if old is not None:
            self._closing = asyncio.ensure_future(old.close())  # a silent old peer must not stall the new client
        try:
            async for msg in ws:
                try:
                    if not isinstance(msg, str):
                        raise ValueError("binary message")
                    self._latest = (parse_input(msg), time.monotonic())
                except ValueError:
                    self.rejected += 1
        except ConnectionClosed:
            pass
        finally:
            if self._client is ws:
                self._client = None
