"""Headset test source without Isaac Sim: sends 640x480 test frames at 20 fps, prints received poses."""
import time

import numpy as np
from PIL import Image, ImageDraw

from sim.protocol import encode_jpeg
from sim.server import TeleopServer

IDENTITY = {"p": [0, 0, 0], "q": [0, 0, 0, 1]}


def eye_image(bg, letter, seq):
    img = Image.new("RGB", (640, 480), bg)
    d = ImageDraw.Draw(img)
    for x in range(0, 640, 80):
        d.line([(x, 0), (x, 480)], fill="white")
    for y in range(0, 480, 80):
        d.line([(0, y), (640, y)], fill="white")
    d.text((300, 220), letter, fill="white", font_size=80)
    d.text((10, 10), f"seq {seq}", fill="white", font_size=24)
    return np.asarray(img)


def main():
    server = TeleopServer()
    server.start()
    print("fake_sim serving ws://127.0.0.1:8765")
    head, seq, last_print = IDENTITY, 0, 0.0
    try:
        while True:
            t0 = time.monotonic()
            msg = server.latest_input()
            if msg:
                head = {"p": list(msg.head.p), "q": list(msg.head.q)}
            if t0 - last_print >= 1.0:
                print("pose", msg)
                last_print = t0
            server.publish_frame({"seq": seq, "head": head},
                                 encode_jpeg(eye_image("red", "L", seq)),
                                 encode_jpeg(eye_image("blue", "R", seq)))
            seq += 1
            time.sleep(max(0, 0.05 - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
