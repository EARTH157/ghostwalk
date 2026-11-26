import math

import numpy as np
import pytest

from sim.protocol import decode_frame, encode_frame, encode_jpeg, parse_input

VALID = '{"t":1,"head":{"p":[0,1.6,0],"q":[0,0,0,1]},"left":null,"right":{"p":[0.2,1.2,-0.3],"q":[0,0,0,1],"trigger":1.4,"stick":[0,-2],"a":false}}'


def test_parse_valid():
    m = parse_input(VALID)
    assert m.left is None and m.right.trigger == 1.0 and m.right.stick == (0.0, -1.0)


@pytest.mark.parametrize("bad", ["{", "{}", VALID.replace("[0,1.6,0]", "[0,1.6]"),
                                 VALID.replace("[0,1.6,0]", "[0,NaN,0]"), "x" * 70000])
def test_parse_rejects(bad):
    with pytest.raises(ValueError):
        parse_input(bad)


def test_parse_normalizes_quaternions():
    m = parse_input(VALID.replace('"q":[0,0,0,1]},"left"', '"q":[0,0,0,2]},"left"'))
    assert math.isclose(sum(c * c for c in m.head.q), 1.0)
    assert m.head.q == (0.0, 0.0, 0.0, 1.0)


def test_parse_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        parse_input(VALID.replace('"q":[0,0,0,1]},"left"', '"q":[0,0,0,0]},"left"'))


def test_frame_roundtrip():
    h, l, r = decode_frame(encode_frame({"seq": 3, "head": {"p": [0, 0, 0], "q": [0, 0, 0, 1]}}, b"LL", b"RRR"))
    assert h["seq"] == 3 and h["left_size"] == 2 and l == b"LL" and r == b"RRR"


def test_encode_jpeg_magic():
    assert encode_jpeg(np.zeros((480, 640, 3), np.uint8))[:2] == b"\xff\xd8"
