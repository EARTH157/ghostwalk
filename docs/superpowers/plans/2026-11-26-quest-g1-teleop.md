# Quest 3S → G1 Teleop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ใส่ Quest 3S แล้วบังคับ Unitree G1 ใน Isaac Sim ได้ทั้งเดินและแขน โดยมองผ่านตาหุ่น

**Architecture:** หน้าเว็บ WebXR (รันใน Quest Browser) ส่ง pose/ปุ่มเป็น JSON ผ่าน WebSocket ไป PC; สคริปต์ Isaac Lab แบบ headless แปลงเป็นคำสั่งเดิน (policy ช่วงล่างที่ NVIDIA เทรนไว้) + IK แขน แล้วเรนเดอร์กล้องคู่เสมือนส่ง JPEG กลับ แว่นทำ reprojection เอง โค้ดคณิตศาสตร์/โปรโตคอลแยกเป็นโมดูลบริสุทธิ์ที่เทสได้โดยไม่ต้องเปิด Isaac Sim

**Tech Stack:** Python 3 (Isaac Lab env), numpy, Pillow, websockets, pytest · JavaScript ES modules, three.js `0.160.0` จาก CDN, `node --test` · Isaac Sim / Isaac Lab (เวอร์ชันกำหนดใน Task 0) · adb

**Spec:** `docs/superpowers/specs/2026-11-26-quest-g1-teleop-design.md`

## Global Constraints

- รันบนโน้ตบุ๊กนี้อย่างเดียว (RTX 5060 Laptop 8GB VRAM, RAM 14GB, Ubuntu 24.04) ไม่มีทางคลาวด์
- ขั้น 0 ไม่ผ่าน (PC ส่งภาพ < 15 fps ต่อตา หรือ Isaac Sim เปิดไม่ได้) → **หยุดทั้งแผน กลับไปถามเจ้าของ**
- แว่นเชื่อมผ่าน USB: `adb reverse tcp:8000 tcp:8000` (หน้าเว็บ) และ `adb reverse tcp:8765 tcp:8765` (WebSocket); server bind `127.0.0.1` เท่านั้น
- ภาพ: JPEG ผ่าน WebSocket, 640×480 ต่อตา, horizontal FOV 90°, IPD 0.064 m
- คอนโทรลเลอร์ ไม่ใช้ hand tracking; มือ G1 แค่กำ/แบตาม trigger
- โมดูลใน `sim/` ที่ไม่ใช่ `teleop_app.py` ห้าม import `isaacsim`, `isaaclab`, `omni` (ต้องเทสใน venv ธรรมดาได้)
- quaternion ในโค้ดฝั่ง sim ใช้ลำดับ Isaac `(w, x, y, z)`; ใน JSON/WebXR ใช้ `(x, y, z, w)` แปลงที่ `sim/coords.py` ที่เดียว
- ไม่มี build step ฝั่งเว็บ

## Review Focus

1. **ลำดับ quaternion สลับ** (WebXR xyzw ↔ Isaac wxyz) → ภาพหมุนผิดทาง; เทสใน Task 1
2. **คอนโทรลเลอร์หายจาก tracking / ยังไม่หยิบขึ้นมา** (`left`/`right` เป็น `null`) → แขนค้างที่เป้าล่าสุด ไม่ crash; เทสใน Task 2 และ Task 6
3. **ข้อความเสียจาก socket** (JSON พัง, NaN, ความยาว array ผิด, ใหญ่เกิน 64 KB) → ทิ้งข้อความนั้น server ยังรันต่อ; เทสใน Task 2 และ Task 3
4. **แว่นหลับ/หลุดการเชื่อมต่อกลางทาง** → input เก่าเกิน 0.5 s ถือว่าไม่มี หุ่นหยุดเดิน; ต่อใหม่ได้; เทสใน Task 3 และ Task 7
5. **thumbstick drift ตอนปล่อยมือ** → deadzone 0.15 ให้ความเร็วเป็นศูนย์; เทสใน Task 7

---

## File Structure

```
README.md                       # Task 1 โครง, Task 8 เต็ม
.gitignore
requirements-dev.txt            # numpy pillow websockets pytest
docs/milestone0.md              # Task 0: ผลพิสูจน์เครื่อง + ค่าที่ task ถัดไปต้องใช้
sim/coords.py                   # WebXR ↔ Isaac frames
sim/protocol.py                 # parse input JSON, encode frame bytes, JPEG
sim/server.py                   # TeleopServer (asyncio ใน thread)
sim/fake_sim.py                 # ส่งภาพทดสอบ ไม่ต้องใช้ Isaac Sim
sim/camera_math.py              # recenter, smoothing, eye poses
sim/control_math.py             # stick → velocity, hand target, reach clamp, grip
sim/teleop_app.py               # Isaac Lab glue (เทสด้วยมือ)
tests/test_*.py
quest_web/index.html
quest_web/protocol.js           # build input message, parse frame bytes
quest_web/app.js                # WebXR loop, stereo quads, reprojection
quest_web/protocol.test.mjs
```

---

### Task 0: พิสูจน์เครื่อง (go/no-go)

**Files:**
- Create: `docs/milestone0.md`

**Interfaces:**
- Produces: `docs/milestone0.md` มีหัวข้อตายตัวเหล่านี้ (task ถัดไปอ่านจากที่นี่):
  - `Versions`: Isaac Sim, Isaac Lab (หรือ Isaac Lab-Arena) ที่ใช้, คำสั่งติดตั้ง
  - `G1 asset`: config/USD ที่ใช้, รุ่นมือ, ชื่อ joint แขนซ้าย/ขวา (7 ต่อข้าง), ชื่อ joint มือ + ค่าแบ/กำ, ชื่อ link หัว (ที่จะวางกล้อง), ชื่อ link ไหล่ซ้าย/ขวา, ชื่อ link มือ (end-effector)
  - `Walking policy`: ไฟล์ policy, input ที่ต้องการ (รูปแบบ velocity command), joint ที่ policy คุม, license/redistribution
  - `Performance`: fps ของ sim + กล้อง 2 ตัว 640×480, RAM peak, VRAM peak
  - `Decision`: GO หรือ NO-GO

- [ ] **Step 1:** เลือกเวอร์ชัน Isaac Sim/Isaac Lab ล่าสุดที่รองรับ RTX 50 + Ubuntu 24.04 และมี G1 + lower-body walking policy (ดูตัวอย่าง G1 loco-manipulation ของ Isaac Lab-Arena เป็นแหล่งอ้างอิง) ติดตั้งตามเอกสารทางการ
- [ ] **Step 2:** รัน Isaac Sim compatibility checker จดผล
- [ ] **Step 3:** รันฉาก G1 + walking policy แบบ headless ให้ยืน/เดินด้วย velocity command คงที่ 10 วินาที โดยไม่ล้ม
- [ ] **Step 4:** เพิ่มกล้อง RGB 2 ตัว 640×480 FOV 90° ที่หัว อ่านภาพออกมาเป็น numpy ทุก step วัด fps เฉลี่ย 30 วินาที และ RAM/VRAM peak (`nvidia-smi`, `free -h`)
- [ ] **Step 5:** เขียน `docs/milestone0.md` ตามหัวข้อข้างบน
- [ ] **Step 6: Decision** — GO เมื่อ fps ภาพ ≥ 15 และ RAM ไม่ swap จนค้าง; ถ้า NO-GO **หยุด แจ้งเจ้าของพร้อมตัวเลข**

### Task 1: Repo + การแปลงพิกัด

**Files:**
- Create: `README.md` (ชื่อโปรเจกต์ + 1 ย่อหน้า), `.gitignore` (Python, `__pycache__`, `.venv`), `requirements-dev.txt`, `sim/__init__.py`, `sim/coords.py`
- Test: `tests/test_coords.py`

**Interfaces:**
- Produces:
  - `xr_pos_to_isaac(p: Sequence[float]) -> np.ndarray` — `(x,y,z)_isaac = (-z, -x, y)_xr`
  - `xr_quat_to_isaac(q_xyzw: Sequence[float]) -> np.ndarray` — คืน `(w,x,y,z)` ใน frame Isaac
  - `quat_mul(a_wxyz, b_wxyz) -> np.ndarray`, `quat_from_yaw(yaw: float) -> np.ndarray`, `yaw_of(q_wxyz) -> float`, `quat_rotate(q_wxyz, v) -> np.ndarray`

- [ ] **Step 1:** `git init`, สร้างไฟล์ skeleton, `python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt`
- [ ] **Step 2: เขียนเทสที่ fail**

```python
def test_xr_forward_is_isaac_x():   assert np.allclose(xr_pos_to_isaac([0,0,-1]), [1,0,0])
def test_xr_up_is_isaac_z():        assert np.allclose(xr_pos_to_isaac([0,1,0]), [0,0,1])
def test_xr_right_is_isaac_minus_y(): assert np.allclose(xr_pos_to_isaac([1,0,0]), [0,-1,0])
def test_identity_quat():           assert np.allclose(xr_quat_to_isaac([0,0,0,1]), [1,0,0,0])
def test_xr_yaw_left_is_isaac_yaw_left():
    s = np.sin(np.pi/4); q_xr = [0, s, 0, s]          # +90° about XR +y (turn left)
    assert np.isclose(yaw_of(xr_quat_to_isaac(q_xr)), np.pi/2)
def test_rotate_forward_by_yaw():
    assert np.allclose(quat_rotate(quat_from_yaw(np.pi/2), [1,0,0]), [0,1,0])
```

- [ ] **Step 3:** รัน `.venv/bin/pytest tests/test_coords.py -v` → FAIL (import error)
- [ ] **Step 4:** implement `sim/coords.py` ด้วย numpy (quaternion: แปลงเวกเตอร์ส่วน xyz ด้วย mapping เดียวกับตำแหน่ง แล้วย้าย w ไปหน้า)
- [ ] **Step 5:** รันเทส → PASS
- [ ] **Step 6:** `git add -A && git commit -m "feat: repo skeleton and WebXR to Isaac coordinate conversion"` (รวม `docs/` ของ spec/plan/milestone0)

### Task 2: โปรโตคอล

**Files:**
- Create: `sim/protocol.py`
- Test: `tests/test_protocol.py`

**Interfaces:**
- Consumes: ไม่มี (เก็บค่า raw XR ไว้ แปลงพิกัดทีหลัง)
- Produces:
  - ข้อความ Quest → PC (text JSON):
    `{"t": <ms float>, "head": {"p":[3], "q":[4 xyzw]}, "left": Hand|null, "right": Hand|null}`
    โดย `Hand = {"p":[3], "q":[4], "trigger": 0..1, "stick": [x,y], "a": bool}`
  - `@dataclass Pose: p: tuple[float,float,float]; q: tuple[float,float,float,float]`
  - `@dataclass Hand: pose: Pose; trigger: float; stick: tuple[float,float]; a: bool`
  - `@dataclass InputMsg: t: float; head: Pose; left: Hand | None; right: Hand | None`
  - `parse_input(text: str) -> InputMsg` — raise `ValueError` เมื่อไม่ถูกต้อง (ความยาว > 65536, JSON พัง, ขาด field, ความยาว array ผิด, ค่าไม่ finite); clamp `trigger` ให้อยู่ [0,1], `stick` ให้อยู่ [-1,1]
  - `encode_frame(header: dict, left_jpeg: bytes, right_jpeg: bytes) -> bytes` — รูปแบบ: `u32 LE header_len | header JSON utf-8 | left_jpeg | right_jpeg`; header ต้องมี `"left_size"` (เติมให้อัตโนมัติ), `"seq"`, `"head"` (pose XR ดิบที่ใช้เรนเดอร์)
  - `decode_frame(data: bytes) -> tuple[dict, bytes, bytes]` (ใช้ในเทส)
  - `encode_jpeg(rgb: np.ndarray, quality: int = 80) -> bytes` — Pillow, รับ `uint8 HxWx3`

- [ ] **Step 1: เขียนเทสที่ fail**

```python
VALID = '{"t":1,"head":{"p":[0,1.6,0],"q":[0,0,0,1]},"left":null,"right":{"p":[0.2,1.2,-0.3],"q":[0,0,0,1],"trigger":1.4,"stick":[0,-2],"a":false}}'
def test_parse_valid():
    m = parse_input(VALID)
    assert m.left is None and m.right.trigger == 1.0 and m.right.stick == (0.0, -1.0)
@pytest.mark.parametrize("bad", ["{", "{}", VALID.replace("[0,1.6,0]","[0,1.6]"),
    VALID.replace("[0,1.6,0]","[0,NaN,0]"), "x"*70000])
def test_parse_rejects(bad):
    with pytest.raises(ValueError): parse_input(bad)
def test_frame_roundtrip():
    h, l, r = decode_frame(encode_frame({"seq":3,"head":{"p":[0,0,0],"q":[0,0,0,1]}}, b"LL", b"RRR"))
    assert h["seq"] == 3 and l == b"LL" and r == b"RRR"
def test_encode_jpeg_magic():
    assert encode_jpeg(np.zeros((480,640,3), np.uint8))[:2] == b"\xff\xd8"
```

- [ ] **Step 2:** รัน `.venv/bin/pytest tests/test_protocol.py -v` → FAIL
- [ ] **Step 3:** implement `sim/protocol.py` (`json.loads` ปฏิเสธ `NaN` ผ่าน `parse_constant` ที่ raise)
- [ ] **Step 4:** รันเทส → PASS
- [ ] **Step 5:** commit `feat: input/frame protocol`

### Task 3: WebSocket server + fake sim

**Files:**
- Create: `sim/server.py`, `sim/fake_sim.py`
- Test: `tests/test_server.py`

**Interfaces:**
- Consumes: `parse_input`, `encode_frame`, `encode_jpeg` (Task 2)
- Produces:
  - `class TeleopServer(host: str = "127.0.0.1", port: int = 8765)`
    - `start() -> None` — รัน asyncio loop ใน daemon thread
    - `stop() -> None`
    - `latest_input(max_age_s: float = 0.5) -> InputMsg | None` — คืนข้อความล่าสุด, `None` ถ้าไม่มีหรือเก่ากว่า `max_age_s` (วัดด้วย `time.monotonic()` ตอนรับ)
    - `publish_frame(header: dict, left_jpeg: bytes, right_jpeg: bytes) -> None` — thread-safe, ส่งให้ client ปัจจุบัน; ไม่มี client = ไม่ทำอะไร; ถ้าเฟรมก่อนยังส่งไม่เสร็จให้ทิ้งเฟรมเก่า (เก็บแค่ล่าสุด)
    - client ใหม่เชื่อมเข้ามา → แทน client เก่า
    - ข้อความที่ `parse_input` ปฏิเสธ → ทิ้ง, นับใน `self.rejected: int`, connection ไม่ปิด
  - `python -m sim.fake_sim` — ส่งภาพทดสอบ 20 fps: ตาซ้ายพื้นแดงตัว "L", ตาขวาพื้นน้ำเงินตัว "R", มีตาราง + เลข seq; header `head` = head pose ล่าสุดที่ได้รับ (ไม่มีให้ใช้ identity); print pose ที่ได้รับทุก 1 วินาที

- [ ] **Step 1: เขียนเทสที่ fail** (ใช้ `websockets.sync.client.connect`, port สุ่มว่าง)

```python
def test_receives_latest_input(server):     # ส่ง VALID → latest_input().right.trigger == 1.0
def test_stale_input_is_none(server):       # ส่ง VALID, รอ 0.6 s → latest_input(0.5) is None
def test_bad_message_keeps_connection(server):  # ส่ง "{" แล้ว VALID → rejected == 1 และ latest_input() ไม่ None
def test_publish_frame_reaches_client(server):  # publish_frame(...) → client recv bytes, decode_frame ได้ seq ตรง
def test_new_client_replaces_old(server):   # client B เชื่อม → publish ไปที่ B
```

- [ ] **Step 2:** รัน `.venv/bin/pytest tests/test_server.py -v` → FAIL
- [ ] **Step 3:** implement `sim/server.py` (`websockets.asyncio.server.serve`, `max_size=65536`), แล้ว `sim/fake_sim.py`
- [ ] **Step 4:** รันเทส → PASS
- [ ] **Step 5:** commit `feat: teleop websocket server and fake sim`

### Task 4: หน้าเว็บ WebXR (ขั้น 1: แว่น ↔ PC)

**Files:**
- Create: `quest_web/index.html`, `quest_web/protocol.js`, `quest_web/app.js`
- Test: `quest_web/protocol.test.mjs`

**Interfaces:**
- Consumes: รูปแบบข้อความ/เฟรมจาก Task 2
- Produces:
  - `protocol.js`: `buildInputMessage(t, headPose, leftHand, rightHand) -> string` (pose = `{p:[3], q:[4]}`, hand = `{...pose, trigger, stick, a}` หรือ `null`); `parseFrame(arrayBuffer) -> {header, left: Blob, right: Blob}`
  - `app.js`:
    - ปุ่ม "Enter VR" → `immersive-vr` + `local-floor`
    - ทุก XR frame: อ่าน viewer pose, grip pose ของแต่ละมือ, gamepad xr-standard (`buttons[0]` trigger, `axes[2],axes[3]` thumbstick, `buttons[4]` A) → ส่ง `buildInputMessage` ถ้า socket เปิดและส่งไม่เกิน 1 ข้อความต่อ XR frame
    - WebSocket `ws://localhost:8765`, ต่อใหม่ทุก 1 s เมื่อหลุด
    - เฟรมที่ได้รับ → decode JPEG (`createImageBitmap`) → texture ของ quad ซ้าย (three.js layer 1) / ขวา (layer 2)
    - **Reprojection:** quad หันตาม `header.head.q`, จุดกลางอยู่ที่ตำแหน่งหัวปัจจุบัน + `rotate(header.head.q, [0,0,-1])` ระยะ 1 m, กว้าง `2·tan(45°)` m สูงตามอัตราส่วน 480/640; ไม่มีเฟรมใหม่ = แสดงเฟรมเดิมต่อ
    - three.js import จาก `https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js`

- [ ] **Step 1: เขียนเทสที่ fail**

```js
test('buildInputMessage roundtrips nulls', () => {
  const m = JSON.parse(buildInputMessage(1, {p:[0,1.6,0], q:[0,0,0,1]}, null, null));
  assert.deepEqual([m.left, m.right, m.head.p], [null, null, [0,1.6,0]]);
});
test('parseFrame splits header and both images', async () => {
  // สร้าง buffer: u32 LE len | {"left_size":2,"seq":5,...} | "LL" | "RRR"
  // คาดหวัง header.seq === 5, left.size === 2, right.size === 3
});
```

- [ ] **Step 2:** รัน `node --test quest_web/` → FAIL
- [ ] **Step 3:** implement `protocol.js` → รันเทส → PASS
- [ ] **Step 4:** implement `index.html` + `app.js`
- [ ] **Step 5: ทดสอบในแว่น (ด้วยมือ)**
  1. `sudo apt install adb` (เจ้าของรันเอง), เปิด developer mode ของ Quest, ต่อสาย USB-C
  2. `adb reverse tcp:8000 tcp:8000 && adb reverse tcp:8765 tcp:8765`
  3. `python3 -m http.server 8000 -d quest_web` และ `.venv/bin/python -m sim.fake_sim`
  4. ใน Quest Browser เปิด `http://localhost:8000` → Enter VR
  5. ผ่านเมื่อ: ตาซ้ายเห็น "L" ตาขวาเห็น "R", หันหัวแล้วภาพนิ่งในโลก (reprojection) ไม่ตามหัวแบบติดหน้า, terminal ของ fake_sim พิมพ์ pose ที่เปลี่ยนตามมือ/หัว, ถอดสายแล้วต่อใหม่ ภาพกลับมาเอง
- [ ] **Step 6:** commit `feat: WebXR client with stereo display and reprojection`

### Task 5: กล้องผ่านตาหุ่น (ขั้น 2)

**Files:**
- Create: `sim/camera_math.py`, `sim/teleop_app.py`
- Test: `tests/test_camera_math.py`

**Interfaces:**
- Consumes: `coords.*` (Task 1), `TeleopServer`, `encode_jpeg`, `InputMsg` (Task 2–3), ค่าจาก `docs/milestone0.md` (link หัว, เวอร์ชัน)
- Produces:
  - `class Recenter` — `update(head_q_isaac, pressed: bool) -> np.ndarray`: ครั้งแรก หรือเมื่อ `pressed` เปลี่ยนจาก False→True ให้จำ `yaw0 = yaw_of(head_q)`; คืน `quat_mul(quat_from_yaw(-yaw0), head_q)`; เปิด `self.yaw0: float` ให้ Task 6 ใช้
  - `smooth(prev: np.ndarray, new: np.ndarray, dt: float, tau: float = 0.3) -> np.ndarray` — exponential low-pass `prev + (1 - exp(-dt/tau))·(new - prev)`
  - `eye_poses(cam_pos, robot_yaw, head_rel_q, ipd=0.064) -> tuple[(pos, q_wxyz), (pos, q_wxyz)]` — `q = quat_mul(quat_from_yaw(robot_yaw), head_rel_q)`; ตาซ้าย = `cam_pos + quat_rotate(q, [0, +ipd/2, 0])`, ขวา `-ipd/2`
  - `teleop_app.py` (CLI: `--headless` ค่าเริ่มต้น): สร้างฉาก G1 ยืนนิ่ง (ยังไม่เดิน) + กล้อง 2 ตัว 640×480 FOV 90°; ทุก step: อ่าน `latest_input()` → `Recenter` (ปุ่ม A ขวา) → `smooth` ตำแหน่ง link หัว → `eye_poses` → ตั้ง pose กล้องด้วย `convention="world"` → `publish_frame` พร้อม `head` = head pose XR ดิบของข้อความที่ใช้; ไม่มี input ให้ใช้ head identity

- [ ] **Step 1: เขียนเทสที่ fail**

```python
def test_recenter_zeroes_initial_yaw():
    r = Recenter(); q = quat_from_yaw(1.0)
    assert np.isclose(yaw_of(r.update(q, False)), 0.0)
def test_recenter_on_button_edge_only():
    r = Recenter(); r.update(quat_from_yaw(0.0), False)
    r.update(quat_from_yaw(0.5), True)               # edge → yaw0 = 0.5
    assert np.isclose(yaw_of(r.update(quat_from_yaw(0.5), True)), 0.0)
def test_smooth_reaches_63_percent_at_tau():
    assert np.allclose(smooth(np.zeros(3), np.ones(3), dt=0.3, tau=0.3), 1 - np.exp(-1))
def test_eyes_separated_sideways_after_robot_yaw():
    (lp, _), (rp, _) = eye_poses(np.zeros(3), np.pi/2, np.array([1,0,0,0]))
    assert np.allclose(lp - rp, [-0.064, 0, 0])      # หุ่นหันซ้าย 90° → ตาซ้ายอยู่ทาง -x
```

- [ ] **Step 2:** รัน `.venv/bin/pytest tests/test_camera_math.py -v` → FAIL
- [ ] **Step 3:** implement `camera_math.py` → PASS
- [ ] **Step 4:** implement `teleop_app.py` ตาม Interfaces
- [ ] **Step 5: ทดสอบในแว่น** — รัน `teleop_app.py` แทน fake_sim; ผ่านเมื่อ: เห็นฉากจากตำแหน่งหัว G1 เป็นสเตอริโอ (มีความลึก), หันหัวซ้าย/ขวา/ก้ม/เงย ภาพในฉากหมุนตามถูกทาง, กด A แล้วมุมมองกลับมาหันไปทางหน้าหุ่น, PC ส่งภาพ ≥ 15 fps (log fps ทุก 5 วินาที)
- [ ] **Step 6:** commit `feat: first-person stereo view from G1 head`

### Task 6: แขนและมือ + ฉากโต๊ะ (ขั้น 3)

**Files:**
- Create: `sim/control_math.py`
- Modify: `sim/teleop_app.py`
- Test: `tests/test_control_math.py`

**Interfaces:**
- Consumes: Task 1, 5; ชื่อ joint แขน/มือ, link ไหล่/มือ, ค่าแบ/กำ จาก `docs/milestone0.md`
- Produces:
  - `hand_target_in_base(ctrl_p_isaac, head_p_isaac, head_yaw0, robot_head_in_base) -> np.ndarray` — `quat_rotate(quat_from_yaw(-head_yaw0), ctrl_p - head_p) + robot_head_in_base` (ตำแหน่งมือเทียบหัวของผู้ใช้ ย้ายไปเทียบหัวหุ่น ใน frame ฐานหุ่น)
  - `clamp_reach(target, shoulder, max_reach: float = 0.45) -> np.ndarray` — ถ้าไกลเกินให้ดึงกลับมาบนผิวทรงกลม
  - `grip_targets(trigger: float, open_q: np.ndarray, closed_q: np.ndarray) -> np.ndarray` — lerp
  - `class HandTracker` — `update(hand: Hand | None, ...) -> np.ndarray`: คืนเป้าใหม่ หรือเป้าล่าสุดเมื่อ `hand is None` (ก่อนมีเป้าแรก = ท่าเริ่มต้นของ asset)
  - `teleop_app.py`: IK (Pink หรือ Differential IK ของ Isaac Lab — builder เลือก) คุม **เฉพาะ joint แขน 7 ต่อข้าง** ตามเป้าที่ผ่าน `clamp_reach`; joint มือตาม `grip_targets`; เพิ่มโต๊ะสูง ~0.75 m ห่างหน้าหุ่น ~0.6 m และกล่อง ~5 cm มี rigid body
- [ ] **Step 1: เขียนเทสที่ fail**

```python
def test_hand_target_maps_forward_reach():
    t = hand_target_in_base(np.array([0.4,0,1.2]), np.array([0,0,1.6]), 0.0, np.array([0,0,1.2]))
    assert np.allclose(t, [0.4, 0, 0.8])
def test_clamp_reach_limits_distance():
    t = clamp_reach(np.array([2.0,0,0]), np.zeros(3), 0.45); assert np.isclose(np.linalg.norm(t), 0.45)
def test_clamp_reach_keeps_near_target():
    assert np.allclose(clamp_reach(np.array([0.1,0,0]), np.zeros(3)), [0.1,0,0])
def test_grip_half():
    assert np.allclose(grip_targets(0.5, np.zeros(2), np.ones(2)), [0.5,0.5])
def test_lost_tracking_holds_last_target():
    # update(valid) แล้ว update(None) → คืนค่าเดิม
```

- [ ] **Step 2:** รันเทส → FAIL; **Step 3:** implement → PASS
- [ ] **Step 4:** ต่อเข้า `teleop_app.py`
- [ ] **Step 5: ทดสอบในแว่น** — ผ่านเมื่อ: ยื่นมือจริงไปข้างหน้าแล้วเห็นมือหุ่นยื่นตามในภาพ, ขยับซ้าย/ขวา/ขึ้น/ลงตรงทิศ, บีบ trigger มือหุ่นกำ, หยิบกล่องขึ้นจากโต๊ะได้ขณะยืน, วางคอนโทรลเลอร์ลง (tracking หาย) แขนค้างไม่สะบัด, หุ่นไม่ล้มเมื่อเอื้อมสุด
- [ ] **Step 6:** commit `feat: arm IK and grip from controllers`

### Task 7: เดิน (ขั้น 4)

**Files:**
- Modify: `sim/control_math.py`, `sim/teleop_app.py`
- Test: `tests/test_control_math.py`

**Interfaces:**
- Consumes: walking policy + รูปแบบ velocity command จาก `docs/milestone0.md`; `latest_input(max_age_s=0.5)`
- Produces:
  - `stick_to_velocity(left_stick, right_stick, deadzone=0.15, max_vx=0.6, max_vy=0.3, max_wz=1.0) -> tuple[float,float,float]` — WebXR: ดันไปข้างหน้าได้ `y < 0`, ไปขวาได้ `x > 0`; คืน `vx = -ly·max_vx`, `vy = -lx·max_vy`, `wz = -rx·max_wz`; แต่ละแกนที่ `|v| < deadzone` = 0 แล้ว rescale ส่วนที่เหลือให้ต่อเนื่อง `(|v|-dz)/(1-dz)`
  - `teleop_app.py`: ส่ง velocity ให้ policy ทุก step; `latest_input()` เป็น `None` หรือมือไหนเป็น `None` → แกนของมือนั้นเป็น 0; กล้องใช้ `smooth` ตามเดิม (กรองการส่ายตอนเดิน)

- [ ] **Step 1: เขียนเทสที่ fail**

```python
def test_push_forward_full():       assert stick_to_velocity((0,-1),(0,0)) == pytest.approx((0.6,0,0))
def test_strafe_right_is_minus_y(): assert stick_to_velocity((1,0),(0,0)) == pytest.approx((0,-0.3,0))
def test_turn_right_is_minus_wz():  assert stick_to_velocity((0,0),(1,0)) == pytest.approx((0,0,-1.0))
def test_drift_inside_deadzone_is_zero(): assert stick_to_velocity((0.1,-0.14),(0.12,0)) == (0,0,0)
def test_just_outside_deadzone_is_small():
    assert 0 < stick_to_velocity((0,-0.2),(0,0))[0] < 0.05
```

- [ ] **Step 2:** รันเทส → FAIL; **Step 3:** implement → PASS
- [ ] **Step 4:** ต่อ walking policy เข้า `teleop_app.py`
- [ ] **Step 5: ทดสอบในแว่น** — ผ่านเมื่อ: ดัน stick ซ้ายแล้วเดินหน้า/ถอย/ไปข้างถูกทาง, stick ขวาหมุนตัวถูกทาง, ปล่อยมือหุ่นหยุดนิ่ง, ถอดสาย USB ระหว่างเดิน → หุ่นหยุดภายใน 0.5 s ไม่ล้ม, เดิน 30 วินาทีแล้วไม่รู้สึกเวียนหัวมาก (ถ้าเวียน ลองเพิ่ม `tau` ก่อน), เดินพร้อมยื่นแขนแล้วไม่ล้ม
- [ ] **Step 6:** commit `feat: thumbstick walking with pretrained policy`

### Task 8: เดโม + README (ขั้น 5)

**Files:**
- Modify: `README.md`
- Create: `docs/demo.gif` (หรือลิงก์วิดีโอ)

- [ ] **Step 1:** อัดเดโม: เริ่มห่างโต๊ะ ≥ 2 m เดินไป หยิบกล่องขึ้น (อัดจากแว่น และ/หรือจอ PC)
- [ ] **Step 2:** README: คลิป/GIF, แผนภาพสถาปัตยกรรม (จาก spec), ส่วนไหนเขียนเอง/ยืมมา, สเปกเครื่องที่ทดสอบ + fps จริง, ขั้นตอนติดตั้ง (เวอร์ชันจาก `docs/milestone0.md`), วิธีรัน (adb reverse, http.server, teleop_app), การบังคับ, ข้อจำกัดที่รู้, license ของ policy การเดิน
- [ ] **Step 3: ทดสอบ README** — clone repo ไปโฟลเดอร์ใหม่ ทำตาม README ทีละบรรทัดจนเข้า VR ได้; แก้ทุกจุดที่ไม่ตรง
- [ ] **Step 4:** รัน `.venv/bin/pytest -q && node --test quest_web/` → ผ่านทั้งหมด
- [ ] **Step 5:** commit `docs: demo and README`
