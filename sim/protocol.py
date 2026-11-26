"""Quest <-> PC wire formats. Raw WebXR values (xyzw quaternions); no frame conversion here."""
import io
import json
import math
import struct
from dataclasses import dataclass

import numpy as np
from PIL import Image

MAX_LEN = 65536


@dataclass
class Pose:
    p: tuple[float, float, float]
    q: tuple[float, float, float, float]


@dataclass
class Hand:
    pose: Pose
    trigger: float
    stick: tuple[float, float]
    a: bool


@dataclass
class InputMsg:
    t: float
    head: Pose
    left: Hand | None
    right: Hand | None


def _reject(c):
    raise ValueError(f"non-finite constant {c}")


def _vec(v, n):
    if not isinstance(v, list) or len(v) != n:
        raise ValueError(f"expected array of {n}")
    # bool is an int subclass; reject it explicitly
    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v):
        raise ValueError("non-numeric or non-finite value")
    return tuple(float(x) for x in v)


def _pose(d):
    q = _vec(d["q"], 4)
    n = math.hypot(*q)
    if not math.isfinite(n) or n < 1e-6:
        raise ValueError("zero quaternion")
    return Pose(_vec(d["p"], 3), tuple(c / n for c in q))


def _hand(d):
    if d is None:
        return None
    a = d["a"]
    if not isinstance(a, bool):
        raise ValueError("a must be bool")
    (trig,) = _vec([d["trigger"]], 1)
    return Hand(_pose(d), min(max(trig, 0.0), 1.0),
                tuple(min(max(c, -1.0), 1.0) for c in _vec(d["stick"], 2)), a)


def parse_input(text: str) -> InputMsg:
    if len(text) > MAX_LEN:
        raise ValueError("message too long")
    try:
        d = json.loads(text, parse_constant=_reject)
        (t,) = _vec([d["t"]], 1)
        return InputMsg(t, _pose(d["head"]), _hand(d["left"]), _hand(d["right"]))
    except (KeyError, TypeError, AttributeError, OverflowError, RecursionError) as e:
        raise ValueError(f"malformed input: {e!r}") from e


def encode_frame(header: dict, left_jpeg: bytes, right_jpeg: bytes) -> bytes:
    h = json.dumps({**header, "left_size": len(left_jpeg)}).encode()
    return struct.pack("<I", len(h)) + h + left_jpeg + right_jpeg


def decode_frame(data: bytes) -> tuple[dict, bytes, bytes]:
    (n,) = struct.unpack_from("<I", data)
    h = json.loads(data[4:4 + n])
    body = data[4 + n:]
    return h, body[:h["left_size"]], body[h["left_size"]:]


def encode_jpeg(rgb: np.ndarray, quality: int = 80) -> bytes:
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, "JPEG", quality=quality)
    return buf.getvalue()
