import * as THREE from 'three';
import { buildInputMessage, parseFrame } from './protocol.js';

const WS_URL = 'ws://localhost:8765';
const W = 2 * Math.tan(Math.PI / 4); // 1 m away, 90 deg horizontal FOV
const H = W * 480 / 640;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera();

// WebXRManager makes the left eye camera see layer 1 and the right eye layer 2.
const quads = [1, 2].map((layer) => {
  const m = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshBasicMaterial({ toneMapped: false }));
  m.layers.set(layer);
  m.visible = false; // until the first frame arrives
  scene.add(m);
  return m;
});
let frameQ = new THREE.Quaternion(); // head orientation the displayed frame was rendered at
let lastSeq = -1;

const decode = (blob) => createImageBitmap(blob, { imageOrientation: 'flipY' });

function swap(quad, bmp) {
  const tex = new THREE.Texture(bmp);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  quad.material.map?.image.close(); // Texture.dispose() does not close ImageBitmaps
  quad.material.map?.dispose();
  quad.material.map = tex;
  quad.material.needsUpdate = true;
  quad.visible = true;
}

let ws;
function connect() {
  ws = new WebSocket(WS_URL);
  ws.binaryType = 'arraybuffer';
  ws.onmessage = async (e) => {
    const { header, left, right } = parseFrame(e.data);
    const bmps = await Promise.all([decode(left), decode(right)]);
    if (header.seq <= lastSeq) { bmps.forEach((b) => b.close()); return; } // stale or out of order
    lastSeq = header.seq;
    quads.forEach((q, i) => swap(q, bmps[i]));
    frameQ.set(...header.head.q); // only now, so image and orientation change together
  };
  ws.onclose = () => { lastSeq = -1; setTimeout(connect, 1000); }; // fake_sim restarts seq at 0
}
connect();

const pose = (t) => t && {
  p: [t.position.x, t.position.y, t.position.z],
  q: [t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w],
};

function hand(src, frame, ref) {
  const p = pose(frame.getPose(src.gripSpace, ref));
  if (!p) return null;
  const g = src.gamepad;
  return { ...p, trigger: g?.buttons[0]?.value ?? 0, stick: [g?.axes[2] ?? 0, g?.axes[3] ?? 0], a: g?.buttons[4]?.pressed === true };
}

const EYE_X = [-0.032, 0.032];
const headP = new THREE.Vector3();
renderer.setAnimationLoop((t, frame) => {
  if (frame) {
    const ref = renderer.xr.getReferenceSpace();
    const vp = frame.getViewerPose(ref);
    if (vp) {
      const head = pose(vp.transform);
      // Reproject: keep the frame fixed in the world at the orientation it was rendered for.
      // each quad sits at its sim camera's offset (IPD 0.064) so infinity converges correctly
      quads.forEach((q, i) => { q.position.set(EYE_X[i], 0, -1).applyQuaternion(frameQ).add(headP.set(...head.p)); q.quaternion.copy(frameQ); });
      if (ws.readyState === WebSocket.OPEN) {
        const hands = { left: null, right: null };
        for (const s of frame.session.inputSources) if (!s.hand && s.gripSpace && s.handedness in hands) hands[s.handedness] = hand(s, frame, ref);
        ws.send(buildInputMessage(t, head, hands.left, hands.right));
      }
    }
  }
  renderer.render(scene, camera);
});

const btn = document.getElementById('enter');
navigator.xr?.isSessionSupported('immersive-vr').then((ok) => { btn.disabled = !ok; });
btn.onclick = async () => {
  const s = await navigator.xr.requestSession('immersive-vr', { requiredFeatures: ['local-floor'] });
  await renderer.xr.setSession(s);
};
