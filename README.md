# 🤖 ESP32 Robot Arm Controller

ระบบควบคุมแขนกล 4 แกน (4-DOF) + Gripper ผ่าน Web Interface โดยใช้ ESP32 และ Servo 5 ตัว ตามโครงสร้าง URDF

![Robot Arm Control](https://img.shields.io/badge/ESP32-Robot%20Arm-blue?style=for-the-badge)
![WebSocket](https://img.shields.io/badge/Protocol-WebSocket-green?style=for-the-badge)
![YOLOv11](https://img.shields.io/badge/Detection-YOLOv11-purple?style=for-the-badge)

## ⚡ Features

- 🎮 **Web-based Control** — ควบคุมผ่าน Browser (React Frontend)
- 📍 **XYZ Coordinates** — ใส่พิกัด XYZ แล้ว Inverse Kinematics คำนวณมุม Servo อัตโนมัติ (4 joints)
- 📡 **Real-time WebSocket** — การสื่อสารแบบ Real-time ความหน่วงต่ำ
- 🎨 **Modern UI** — สร้างด้วย React + Vite พร้อม Premium dark theme และ Micro-animations
- 📊 **Servo Visualization** — แสดงมุม Servo 4 ตัวพร้อม 3D visualization (Side/Top/Front/3D view)
- 🛑 **Emergency Stop** — ปุ่มหยุดฉุกเฉิน
- 📷 **Object Detection** — ตรวจจับวัตถุด้วย YOLOv11 ผ่าน USB Webcam
- 📹 **Camera Selection** — เลือกระบุและสลับกล้องที่ใช้งานผ่านหน้า UI ได้สดๆ
- 🎯 **ArUco Calibration** — ปรับเทียบพิกัดด้วย ArUco Markers สำหรับความแม่นยำสูง
- 🤏 **Pick & Place** — ระบบหยิบ-วางอัตโนมัติโดยอ้างอิงจาก Object Detection
- 👁️ **Hide WiFi Password** — สลับโหมดเปิด/ปิดหน้าตา/รหัสผ่าน Wi-Fi ของหุ่นยนต์ได้

## 🦾 URDF Robot Arm Specifications

แขนกลใช้โครงสร้างตาม URDF (`URDF_Robot_arm/urdf/ufdr.urdf`) มี 4 Joints:

| Joint | ชื่อ URDF | Axis | ความหมาย | GPIO | Servo |
|-------|-----------|------|----------|------|-------|
| Joint0 | base_link → Link_1 | Y | **Waist** — หมุนฐาน | 13 | SG90 |
| Joint1 | Link_1 → Link_2 | Z | **Shoulder** — ไหล่ | 12 | SG90 |
| Joint2 | Link_2 → Link_3 | Z | **Elbow** — ข้อศอก | 14 | SG90 |
| Joint3 | Link_3 → End | X | **Wrist** — ข้อมือ | 27 | **MG90s** |
| — | — | — | **Gripper** — จับ/ปล่อย | 26 | **MG90s** |

### Arm Dimensions (จาก URDF)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `BASE_HEIGHT` | 56 mm | Joint0 z-offset |
| `SHOULDER_OFFSET` | 42.6 mm | Joint1 offset |
| `ARM_LENGTH_1` | 120 mm | Link_2 — ความยาวแขนส่วนบน |
| `ARM_LENGTH_2` | 116.25 mm | Link_3 to End — ความยาวแขนส่วนล่าง |

## 🔧 Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Microcontroller | ESP32 DevKit |
| Servo Motors | SG90 × 3 (Waist, Shoulder, Elbow) |
| Servo Motors | **MG90s** × 2 (Wrist, Gripper) |
| USB Webcam | สำหรับ Object Detection (optional) |
| Power Supply | 5V 2A (สำหรับ Servo) |
| Jumper Wires | ตามความเหมาะสม |

### Wiring Diagram

```
ESP32 Pin   ->   Servo            URDF Joint       Type
----------------------------------------------------------
GPIO 13     ->   Servo 1          Joint0 (Waist)   SG90
GPIO 12     ->   Servo 2          Joint1 (Shoulder) SG90
GPIO 14     ->   Servo 3          Joint2 (Elbow)   SG90
GPIO 27     ->   Servo 4          Joint3 (Wrist)   MG90s
GPIO 26     ->   Servo 5          Gripper          MG90s
5V          ->   Servo VCC        (ใช้ External Power)
GND         ->   Servo GND
```

> ⚠️ **สำคัญ**: ควรใช้ Power Supply แยกสำหรับ Servo เพื่อป้องกัน ESP32 Reset

## 📦 Installation

### 1. Upload ESP32 Firmware

**Libraries Required (ติดตั้งผ่าน Arduino Library Manager):**
- `ESP32Servo` by Kevin Harrington
- `WebSocketsServer` by Markus Sattler
- `ArduinoJson` by Benoit Blanchon

**Steps:**
1. เปิด Arduino IDE
2. ติดตั้ง ESP32 Board ใน Board Manager
3. เปิดไฟล์ `esp32/esp32_robot_arm.ino`
4. เลือก Board: ESP32 Dev Module
5. Upload โค้ดไปยัง ESP32

### 2. ติดตั้ง Web Frontend (React + Vite)

โค้ดส่วนของ Frontend ดั้งเดิม (Vanilla JS) ถูกย้ายและเปลี่ยนเป็น React ในโฟลเดอร์ `client/`

```bash
cd client
npm install
npm run dev
```

### 3. ติดตั้ง Object Detection Server (Optional)

ใช้ FastAPI สำหรับการทำงานแทน Flask ในระบบเดิม

```bash
cd server
pip install -r requirements.txt
python app.py
```

Server จะเริ่มที่ `http://localhost:5000` (FastAPI + Uvicorn) พร้อม API สำหรับ:
- Video stream (MJPEG)
- Object detection (YOLOv11)
- ArUco calibration
- Pick & Place sequences
- Camera list detection

### 4. Connect to Robot Arm

1. **เปิด ESP32** — รอสักครู่ให้เริ่มทำงาน
2. **เชื่อมต่อ WiFi** — ค้นหาและเชื่อมต่อ WiFi `RobotArm_AP`
   - Password: `12345678`
3. **เปิด Web Interface** — เปิดที่ `http://localhost:5173` (หากรันจาก Vite)
4. **Connect** — กดปุ่ม CONNECT (IP: `192.168.4.1`, Port: `81`)

## 🎮 Usage

### Control Panel

| Control | Description |
|---------|-------------|
| **X Slider** | เลื่อนแขนกลในแนวซ้าย-ขวา (-100 to 100 mm) |
| **Y Slider** | เลื่อนแขนกลในแนวหน้า-หลัง (-100 to 100 mm) |
| **Z Slider** | เลื่อนแขนกลในแนวสูง-ต่ำ (0 to 150 mm) |

### Servo Monitors

| Gauge | Joint | Function |
|-------|-------|----------|
| J0 (Cyan) | Waist | หมุนฐาน 0°–180° |
| J1 (Magenta) | Shoulder | ยกไหล่ 0°–180° |
| J2 (Green) | Elbow | งอข้อศอก 0°–180° |
| J3 (Orange) | Wrist | หมุนข้อมือ 0°–180° [MG90s] |
| GRP (Pink) | Gripper | จับ/ปล่อย 0°–180° [MG90s] |

### Quick Actions

| Button | Function |
|--------|----------|
| 🏠 HOME | กลับตำแหน่งเริ่มต้น (ทุก Joint = 90°) |
| ✊ GRAB | หยิบจับ (Wrist ปิด) |
| 🖐️ RELEASE | ปล่อย (Wrist เปิด) |
| 🛑 STOP | หยุดฉุกเฉิน |

## 📁 File Structure

```
WebRobot/
├── client/                 # React + Vite Frontend (Active)
│   ├── src/                # ซอร์สโค้ด React Components
│   └── package.json        # Node.js dependencies
├── index.html              # หน้าเว็บหลัก (Vanilla JS เดิม/Deprecated)
├── style.css               # สไตล์หน้าเว็บเดิม
├── app.js                  # Logic เดิมการควบคุม
├── README.md               # คู่มือการใช้งานฉบับนี้
├── esp32/
│   └── esp32_robot_arm.ino # Firmware ESP32 (4 joints + gripper, MG90s)
├── server/
│   ├── app.py              # FastAPI server - Object Detection & ArUco
│   ├── requirements.txt    # Python dependencies สำหรับ Backend
│   └── yolo11n.pt          # YOLOv11 model weights
└── URDF_Robot_arm/
    ├── urdf/
    │   ├── ufdr.urdf       # URDF model definition
    │   ├── ufdr.csv        # Joint parameters summary
    │   └── ...
    ├── meshes/             # 3D mesh files (STL/DAE)
    ├── config/             # Joint configuration (YAML)
    └── launch/             # ROS launch files
```

## 🔧 Configuration

### ปรับค่า Arm Dimensions (app.js)

```javascript
const CONFIG = {
    // จาก URDF_Robot_arm specs (ufdr.urdf)
    BASE_HEIGHT: 56,        // Joint0 z-offset (mm)
    SHOULDER_OFFSET: 42.6,  // Joint1 offset (mm)
    ARM_LENGTH_1: 120,      // Upper arm length (mm)
    ARM_LENGTH_2: 116.25,   // Forearm length (mm)
};
```

### เปลี่ยน WiFi Mode (esp32_robot_arm.ino)

```cpp
// ใช้ Station Mode แทน Access Point
#define USE_STATION_MODE
const char* STA_SSID = "YourWiFiName";
const char* STA_PASSWORD = "YourWiFiPassword";
```

### WebSocket Command Format

```json
// Move command (ส่งจาก Web → ESP32)
{"type": "move", "x": 50, "y": 30, "z": 80, "s1": 120, "s2": 85, "s3": 95, "s4": 90}

// Status response (ส่งจาก ESP32 → Web)
{"type": "status", "s1": 120, "s2": 85, "s3": 95, "s4": 90, "gripper": 90}
```

## 📝 License

MIT License — สามารถนำไปใช้และดัดแปลงได้ตามต้องการ

---

Made with ❤️ for Robotics Education
