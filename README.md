# 🤖 ESP32 Robot Arm Controller

ระบบควบคุมแขนกล 3 แกน (XYZ) ผ่าน Web Interface โดยใช้ ESP32 และ Servo 3 ตัว

![Robot Arm Control](https://img.shields.io/badge/ESP32-Robot%20Arm-blue?style=for-the-badge)
![WebSocket](https://img.shields.io/badge/Protocol-WebSocket-green?style=for-the-badge)

## ⚡ Features

- 🎮 **Web-based Control** - ควบคุมผ่าน Browser (Chrome, Firefox, Edge)
- 📍 **XYZ Coordinates** - ใส่พิกัด XYZ แล้วระบบจะคำนวณมุม Servo อัตโนมัติ
- 📡 **Real-time WebSocket** - การสื่อสารแบบ Real-time ความหน่วงต่ำ
- 🎨 **Modern UI** - Dark theme พร้อม animations
- 📊 **Servo Visualization** - แสดงมุม Servo และ 3D visualization
- 🛑 **Emergency Stop** - ปุ่มหยุดฉุกเฉิน

## 🔧 Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Microcontroller | ESP32 DevKit |
| Servo Motors | SG90 or MG996R x 3 |
| Power Supply | 5V 2A (สำหรับ Servo) |
| Jumper Wires | ตามความเหมาะสม |

### Wiring Diagram

```
ESP32 Pin   ->   Servo
--------------------------
GPIO 13     ->   Servo 1 (Base)
GPIO 12     ->   Servo 2 (Shoulder)
GPIO 14     ->   Servo 3 (Elbow)
GPIO 27     ->   Gripper (Optional)
5V          ->   Servo VCC (ใช้ External Power)
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

### 2. Connect to Robot Arm

1. **เปิด ESP32** - รอสักครู่ให้เริ่มทำงาน
2. **เชื่อมต่อ WiFi** - ค้นหาและเชื่อมต่อ WiFi `RobotArm_AP`
   - Password: `12345678`
3. **เปิด Web Interface** - เปิดไฟล์ `index.html` ใน Browser
4. **Connect** - กดปุ่ม CONNECT (IP: 192.168.4.1, Port: 81)

## 🎮 Usage

### Control Panel

| Control | Description |
|---------|-------------|
| **X Slider** | เลื่อนแขนกลในแนวซ้าย-ขวา (-100 to 100 mm) |
| **Y Slider** | เลื่อนแขนกลในแนวหน้า-หลัง (-100 to 100 mm) |
| **Z Slider** | เลื่อนแขนกลในแนวสูง-ต่ำ (0 to 150 mm) |

### Quick Actions

| Button | Function |
|--------|----------|
| 🏠 HOME | กลับตำแหน่งเริ่มต้น |
| ✊ GRAB | หยิบจับ (ปิด Gripper) |
| 🖐️ RELEASE | ปล่อย (เปิด Gripper) |
| 🛑 STOP | หยุดฉุกเฉิน |

## 📁 File Structure

```
WebRobot/
├── index.html          # หน้าเว็บหลัก
├── style.css           # สไตล์ Modern Dark Theme
├── app.js              # Logic และ WebSocket
├── README.md           # คู่มือการใช้งาน
└── esp32/
    └── esp32_robot_arm.ino  # Firmware สำหรับ ESP32
```

## 🔧 Configuration

### ปรับค่า Arm Length (app.js)

```javascript
const CONFIG = {
    ARM_LENGTH_1: 80,   // ความยาวแขนส่วนบน (mm)
    ARM_LENGTH_2: 100,  // ความยาวแขนส่วนล่าง (mm)
};
```

### เปลี่ยน WiFi Mode (esp32_robot_arm.ino)

```cpp
// ใช้ Station Mode แทน Access Point
#define USE_STATION_MODE
const char* STA_SSID = "YourWiFiName";
const char* STA_PASSWORD = "YourWiFiPassword";
```

## 📝 License

MIT License - สามารถนำไปใช้และดัดแปลงได้ตามต้องการ

---

Made with ❤️ for Robotics Education
