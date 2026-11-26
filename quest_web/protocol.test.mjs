import test from 'node:test';
import assert from 'node:assert/strict';
import { buildInputMessage, parseFrame } from './protocol.js';

test('buildInputMessage roundtrips nulls', () => {
  const m = JSON.parse(buildInputMessage(1, { p: [0, 1.6, 0], q: [0, 0, 0, 1] }, null, null));
  assert.deepEqual([m.left, m.right, m.head.p], [null, null, [0, 1.6, 0]]);
});

test('buildInputMessage keeps hand fields', () => {
  const hand = { p: [1, 2, 3], q: [0, 0, 0, 1], trigger: 0.5, stick: [0.1, -0.2], a: true };
  const m = JSON.parse(buildInputMessage(2, { p: [0, 0, 0], q: [0, 0, 0, 1] }, hand, null));
  assert.deepEqual(m.left, hand);
});

test('parseFrame splits header and both images', async () => {
  const h = new TextEncoder().encode(JSON.stringify({ left_size: 2, seq: 5, head: { q: [0, 0, 0, 1] } }));
  const buf = new Uint8Array(4 + h.length + 5);
  new DataView(buf.buffer).setUint32(0, h.length, true);
  buf.set(h, 4);
  buf.set(new TextEncoder().encode('LLRRR'), 4 + h.length);
  const f = parseFrame(buf.buffer);
  assert.equal(f.header.seq, 5);
  assert.equal(f.left.size, 2);
  assert.equal(f.right.size, 3);
  assert.equal(await f.right.text(), 'RRR');
});
