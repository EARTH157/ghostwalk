# Quest 3S → Unitree G1 Teleop ใน Isaac Sim — Design

วันที่: 2026-11-26

## เจตนาและเหตุผล

ผลงาน portfolio ลง GitHub: ใส่ Meta Quest 3S แล้วบังคับหุ่น humanoid Unitree G1 ใน Isaac Sim
ได้ทั้งการเดินและแขน โดยมองเห็นโลกในซิม **ผ่านตาหุ่น** (first-person)
คนดู repo ต้องเห็นชัดว่าส่วนไหนเจ้าของเขียนเอง → สะพาน แว่น↔ซิม เขียนเอง, ฟิสิกส์/เรนเดอร์/policy การเดินยืมมา

## เป้าหมาย

- เปิดหน้าเว็บในเบราว์เซอร์ Quest → เข้าโหมด VR → เห็นภาพสเตอริโอจากหัว G1 ในซิม
- หันหัวจริง → มุมกล้องหันตาม
- คอนโทรลเลอร์ซ้าย/ขวา → ตำแหน่งมือซ้าย/ขวาของหุ่น (IK แขน), trigger → กำมือ/แบมือ
- thumbstick ซ้าย → เดินหน้า/ถอย/ไปข้าง, thumbstick ขวา (แกน x) → หมุนตัว
- ฉากเดโม: เดินไปที่โต๊ะ แล้วหยิบกล่องขึ้นมา
- README มีคลิป/GIF เดโม + แผนภาพสถาปัตยกรรม + วิธีรัน

## Non-goals

- หุ่นจริง, เก็บข้อมูลเทรน (HDF5/imitation learning), hand tracking ระดับนิ้ว
- หลายผู้ใช้, Windows, การเทรน policy การเดินเอง
- WebRTC ในเวอร์ชันแรก (ดู "ทางลัดที่ตั้งใจ")

## Anti-goals (ถ้าเกิดถือว่าล้มเหลวแม้จะ "ทำงาน")

- ใส่แว่นแล้วเวียนหัวจนใช้ไม่ได้
- เป็นแค่ wrapper ครอบเดโมของ NVIDIA แทบไม่มีโค้ดของตัวเอง
- หุ่นล้มบ่อยจนเดโมไม่จบ

## ข้อจำกัด

- เครื่อง: โน้ตบุ๊ก RTX 5060 Laptop **VRAM 8GB, RAM 14GB**, Ubuntu 24.04
  ต่ำกว่าสเปกทางการ (Isaac Sim 5.0: VRAM 16GB; Isaac Lab: RAM ≥32GB) → **ขั้น 0 ต้องพิสูจน์ว่ารันได้ก่อน**
  **เจ้าของตัดสิน: รันบนเครื่องนี้อย่างเดียว** แว่นต่อ USB ไม่เตรียมทางคลาวด์ล่วงหน้า
  ถ้าขั้น 0 ไม่ผ่าน → หยุด แล้วกลับไปตัดสินใจกับเจ้าของ (เพิ่ม RAM ไม่แก้ปัญหา VRAM 8GB)
- แว่น: Meta Quest 3S, ใช้ Meta Quest Browser (WebXR) ไม่ทำแอป native
- WebXR ต้องการ secure context → ช่วงพัฒนาใช้ `adb reverse` ผ่านสาย USB-C ให้แว่นเรียก `localhost` ได้
- G1 ไม่มีข้อต่อคอ → "หันหัวแล้วหัวหุ่นหันตาม" ทำทางกายภาพไม่ได้ (ดูการออกแบบกล้อง)

## วิธีที่พิจารณา

| | วิธี | ข้อดี | ข้อเสีย |
|---|---|---|---|
| A | ต่อยอด Isaac Lab-Arena G1 loco-manipulation + CloudXR | ใช้งานได้เร็วสุด | ฝีมือตัวเองไม่ชัด, หนักเครื่อง, API เปลี่ยนบ่อย, ไม่มีมุมตาหุ่น |
| **B (เลือก)** | WebXR page เขียนเอง ↔ WebSocket ↔ สคริปต์ Isaac Lab เขียนเอง, ยืม policy การเดิน | โค้ดหลักเป็นของตัวเอง, คุมความละเอียดภาพให้เข้ากับเครื่องได้ | งานเยอะกว่า, ต้องจัดการความหน่วงเอง |
| C | ALVR + SteamVR + OpenXR | — | มีรายงานว่าคุมได้แค่ช่วงบน, SteamVR บน Linux ไม่เสถียร |

เจ้าของเลือก B เพราะเป็น portfolio และต้องการมุมตาหุ่น ใช้ A เป็นตัวอ้างอิงได้ (เช่นดูว่าเขาต่อ policy การเดินกับ IK แขนยังไง)

## สถาปัตยกรรม

```
Quest Browser (WebXR, JS)                    PC (Python, Isaac Lab, headless)
┌──────────────────────────┐  pose+input   ┌───────────────────────────────────┐
│ อ่าน head pose,          │ ───────────▶ │ teleop loop:                        │
│ controller poses,        │  JSON/WS      │  thumbstick → velocity cmd → policy │
│ buttons, thumbsticks     │               │  controller pose → IK แขน+เอว      │
│                          │ ◀─────────── │  trigger → มือ G1 กำ/แบ            │
│ แสดงภาพซ้าย/ขวา แยกตา    │  JPEG L/R/WS  │  กล้องคู่เสมือนที่หัว → เรนเดอร์   │
└──────────────────────────┘               └───────────────────────────────────┘
```

### ฝั่งแว่น (`quest_web/`)
- หน้าเว็บเดียว: ปุ่ม "Enter VR" → `immersive-vr` session
- ทุกเฟรม: ส่ง head pose, controller poses (grip space), trigger, thumbstick เป็น JSON ผ่าน WebSocket
- รับภาพ JPEG ซ้าย/ขวา → วาดบน quad แยกตา
- **Reprojection (การตัดสินใจของผม)**: แต่ละภาพแนบ head pose ที่ใช้เรนเดอร์มาด้วย แว่นวาง quad ไว้ในโลกตาม pose นั้น
  แล้วให้แว่นเรนเดอร์เองที่ 72 Hz ตามหัวล่าสุด → หันหัวไม่หน่วงแม้ PC ส่งภาพช้ากว่า (หน่วงแค่ภาพในฉาก ไม่ใช่การหมุนหัว)
- ไม่มี build step; WebXR API ตรงๆ หรือ three.js จาก CDN (การตัดสินใจของ builder)

### ฝั่ง PC (`sim/`)
- Isaac Lab script เดียว รัน G1 + ฉาก (พื้น, โต๊ะ, กล้อง) แบบ headless
- **การเดิน**: ใช้ policy ช่วงล่างของ G1 ที่ NVIDIA เทรนไว้ (เช่นตัวที่ใช้ใน G1 loco-manipulation ของ Isaac Lab-Arena) รับ velocity command จาก thumbstick
- **แขน**: IK ของ Isaac Lab (Pink IK หรือ Differential IK) ให้มือหุ่นตามตำแหน่งคอนโทรลเลอร์ เทียบกับลำตัวหุ่น
  จำกัดระยะเอื้อมในเวอร์ชันแรก กันจุดถ่วงเปลี่ยนจนล้ม
- **กล้อง (การตัดสินใจของผม)**: กล้องคู่เสมือน ระยะห่าง ~64mm
  - ตำแหน่ง = ตำแหน่งหัว G1 ที่ผ่าน low-pass filter (ตัดการส่ายตามจังหวะก้าว → ลดเวียนหัว)
  - การหมุน = yaw ลำตัวหุ่น + การหมุนหัวของผู้ใช้
  - เหตุผล: G1 ไม่มีคอ และการผูกกล้องกับหัวตรงๆ จะทำให้เวียนหัวตอนเดิน
- ส่ง JPEG ซ้าย/ขวา กลับทาง WebSocket เดียวกัน

## ทางลัดที่ตั้งใจ (การตัดสินใจของผม)

- ภาพเป็น JPEG ผ่าน WebSocket ไม่ใช่ WebRTC — ง่ายสุด; เปลี่ยนเมื่อวัดแล้วความหน่วง/แบนด์วิดท์ไม่พอ
- คอนโทรลเลอร์แทน hand tracking — hand tracking ทำให้ไม่มี thumbstick ไว้สั่งเดิน
- มือ G1 แค่กำ/แบ ตาม trigger ไม่ map นิ้วทีละนิ้ว
- ความละเอียดภาพต่อตาเริ่มต่ำ (เช่น 640×480) แล้วค่อยเพิ่มตามที่เครื่องไหว

## ลำดับงาน (milestones)

0. **พิสูจน์เครื่อง**: ติดตั้ง Isaac Sim/Isaac Lab, รัน G1 + policy การเดิน + กล้อง 2 ตัวแบบ headless บนโน้ตบุ๊กนี้ จดเฟรมเรต/RAM/VRAM
1. **แว่น ↔ PC**: หน้า WebXR ส่ง pose ไป PC (log ได้), PC ส่งภาพทดสอบกลับมาโชว์แยกตา
2. **มองผ่านตาหุ่น**: G1 ยืนนิ่ง, กล้องคู่ตามหัวผู้ใช้
3. **แขน**: IK ให้มือตามคอนโทรลเลอร์ + กำ/แบ หยิบกล่องบนโต๊ะได้ขณะยืน
4. **เดิน**: thumbstick → policy การเดิน + กล้องกรองการส่าย
5. **เดโม + README**: เดินไปหยิบกล่อง, อัดคลิป, เขียน README

แต่ละขั้นจบแล้วต้องใช้งานได้จริงและ commit ลง GitHub ได้ ถ้าขั้น 4 ติด ขั้น 0–3 ยังเป็นผลงานที่ใช้ได้

## เกณฑ์ความสำเร็จ (ตั้งโดยผม แก้ได้)

- เดโมเดินไปหยิบกล่องได้ครบโดยหุ่นไม่ล้ม
- แว่นแสดงผลเต็มเฟรมเรตของแว่นด้วย reprojection, หันหัวแล้วไม่รู้สึกเวียน
- PC ส่งภาพใหม่ ≥ 15 fps ต่อตา (เส้นผ่าน/ไม่ผ่านของขั้น 0), เป้า 30 fps
- คนอื่น clone repo แล้วรันตาม README ได้ (ระบุเวอร์ชัน Isaac Sim/Isaac Lab ชัดเจน)

## ปล่อยให้ builder ตัดสิน

- เวอร์ชัน Isaac Sim / Isaac Lab ที่ใช้ (เลือกตัวที่มี G1 + policy การเดิน และรันบนเครื่องนี้ได้)
- จะหยิบ policy การเดินและ asset G1 มาจาก Isaac Lab หรือ Isaac Lab-Arena ยังไง
- Pink IK หรือ Differential IK
- WebXR ตรงๆ หรือ three.js
- รูปแบบข้อความ JSON, การจับคู่แกนพิกัด WebXR ↔ Isaac Sim
- License ของ repo

## อ้างอิง

- Isaac Lab-Arena G1 loco-manipulation teleop: https://isaac-sim.github.io/IsaacLab-Arena/main/pages/example_workflows/locomanipulation/step_2_teleoperation.html
- Isaac Capture (XR teleop ของ Isaac Lab): https://docs.nvidia.com/isaaclab/features/isaac-capture
- Isaac Sim 5.0 requirements: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html
