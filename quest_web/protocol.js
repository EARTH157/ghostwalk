// Wire formats shared with sim/protocol.py. Quaternions are [x,y,z,w].
export const buildInputMessage = (t, head, left, right) => JSON.stringify({ t, head, left, right });

// Frame: u32 LE header length | JSON header (has left_size) | left JPEG | right JPEG
export function parseFrame(buf) {
  const n = new DataView(buf).getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, n)));
  const body = buf.slice(4 + n);
  const type = 'image/jpeg';
  return {
    header,
    left: new Blob([body.slice(0, header.left_size)], { type }),
    right: new Blob([body.slice(header.left_size)], { type }),
  };
}
