# 🤖 WebRobot Arm Controller

ระบบควบคุมแขนกล 4 แกน (4-DOF) + Gripper ผ่าน Web Interface ด้วย React Frontend, Python FastAPI Backend สำหรับ Object Detection (YOLOv11/YOLO26) และ ESP32 ควบคุม Servo ผ่านการสื่อสาร Serial (USB) 

![React](https://img.shields.io/badge/Frontend-React%20%2B%20Vite-blue?style=for-the-badge&logo=react)
![FastAPI](https://img.shields.io/badge/Backend-FastAPI-green?style=for-the-badge&logo=fastapi)
![ESP32](https://img.shields.io/badge/Microcontroller-ESP32-yellow?style=for-the-badge)
![YOLO](https://img.shields.io/badge/Detection-YOLOv11-purple?style=for-the-badge)

## ⚡ Features

- 🎮 **Web-based Control** — ควบคุมแขนกลผ่านเบราว์เซอร์ด้วย React Dashboard 
- 📍 **XYZ Inverse Kinematics** — สั่งงานเป็นพิกัดอ้างอิง XYZ แล้วให้ ESP32 คำนวณมุม Servo อัตโนมัติในพื้นที่ทำงาน (Workspace Bounds)
- 🤖 **Serial Communication** — เชื่อมต่อกับ ESP32 ง่ายๆ ผ่านสาย USB มีระบบตรวจจับและเลือก COM Port ผ่านหน้า UI (FastAPI ทำหน้าที่เป็นตัวกลาง)
- 🎯 **ArUco Calibration** — ปรับเทียบพิกัดโลกจริง (Real-World Coordinates) ง่ายๆ ผ่านกล้องและการทำ Homography วางกระดาษ A4 ลงกริต
- 📷 **YOLO Object Detection** — สลับใช้โมเดล YOLOv11 และ YOLO26 สำหรับการตรวจจับชิ้นงาน และทำการหยิบวาง (Pick & Place)
- 📹 **Camera & Model Selection** — เลือกระบุกล้อง USB/Webcam ที่ใช้งานจากระบบ และเลือกโมเดล (เช่น yolo11n.pt, yolo26.pt, best.pt) ได้บนหน้าเว็บ
- 🧠 **PCA9685 PWM Controller** — ควบคุม Servo 4 แกนอย่างแม่นยำราบรื่นผ่านโมดูล PCA9685 ที่ต่อแบบ I2C เข้ากับ ESP32 ลดปัญหาไฟตกจาก ESP32
- 🛑 **Emergency Stop / Home** — ปุ่มสั่งกลับตำแหน่ง 90 องศา หรือเปิด-ปิด Gripper ผ่าน Dashboard ทันที
- 🖥️ **Real-time 3D Simulation** — มีโปรแกรมจำลองแขนกลอัจฉริยะ (`robot_arm_sim.py`) ใช้งานบน PC รองรับการคำนวณ IK และสั่งงานตรงผ่าน Serial ได้
- 🧪 **Jupyter Model Testing** — ทดสอบแบบออฟไลน์สำหรับความแม่นยำของ YOLO Object Detection ไว้อย่างรัดกุม (`test_yolo26.ipynb`)

---

## 🦾 System Architecture

การทำงานของระบบถูกปรับปรุงใหม่ให้มีความเสถียรขึ้นและรับประกันการทำงานของโมเดล AI ที่มีขนาดใหญ่ โดยแบ่งเป็น 3 ชิ้นส่วน:

1. **Client (React + Vite)**: ทำหน้าที่แสดงภาพวิดีโอ (MJPEG) วาดจุด ArUco, กล่อง Bounding Box และคำสั่งสั่งงานต่างๆ
2. **Server (FastAPI)**: ใช้กล้องเพื่อรัน YOLO และ ArUco สร้าง REST API และต่อ Serial/COM Port ไปหา ESP32 โดยตรง 
3. **ESP32 Firmware**: รอรับคำสั่ง (X, Y, Z / HOME / GRIP_OPEN / GRIP_CLOSE) ที่ส่งผ่าน Serial (Baudrate `115200`) นำไปแปลงเป็น Inverse Kinematics แล้วสั่งให้ PCA9685 กับ ESP32Servo ขับมอเตอร์

---

## 🔧 Hardware Requirements & Wiring

1. **ESP32 DevKit**
2. **PCA9685 (I2C 16-Channel PWM)** — โมดูลควบคุมเซอร์โวเพื่อลดภาระกระแสให้กับโมดูล
3. **Servo Motors** — 4 ตัวสำหรับแกน และ 1 ตัวสำหรับ Gripper
4. **USB Power 5V** กระแสสูง (แยกส่วนสำหรับ Servo และ PCA9685)
5. **USB Webcam** สำหรับการจับภาพจากมุม Top-Down 

### 📐 Wiring Diagram
| Hardware Component | ESP32 Pin / Port | Type & Description |
|-------------------|------------------|-------------------|
| PCA9685 (SDA) | `GPIO 21` (SDA) | I2C Communication |
| PCA9685 (SCL) | `GPIO 22` (SCL) | I2C Communication |
| Servo 1 (Base/Waist)| PCA9685 `Ch 0` | ควบคุมการหมุนฐาน |
| Servo 2 (Shoulder)| PCA9685 `Ch 1` | ควบคุมไหล่ |
| Servo 3 (Elbow) | PCA9685 `Ch 2` | ควบคุมศอก |
| Servo 4 (Wrist) | PCA9685 `Ch 3` | ควบคุมข้อมือ (Tilt Compensated) |
| Servo 5 (Gripper)| `GPIO 15` | ESP32 Servo ควบคุมการจับวัตถุ |

> ⚠️ **คำเตือนเรื่องพลังงาน**: จ่ายไฟ 5V นอกเข้าที่จุด *V+* ของบอร์ด PCA9685 และ Gripper (ไม่ต้องจ่ายไฟจากขา 5V/VIN ของ ESP32 เด็ดขาด เพื่อป้องกันบอร์ดพังและการ Reset อัตโนมัติระหว่างที่แขนทำงาน)

---

## 📦 Installation & Setup

ระบบทั้งหมดถูกแยกย่อยออกเป็น 3 ไดเรกทอรี ได้แก่ `client`, `server` และ `esp32`

### 1. ⚙️ ESP32 Firmware
1. เปิดไฟล์ `esp32/robot_arm_4dof_final (1).ino` ด้วย Arduino IDE
2. ติดตั้ง Library ดังนี้:
   - `Adafruit PWMServoDriver`
   - `ESP32Servo`
3. เชื่อมต่อ ESP32 ผ่านสาย USB เข้ากับคอมพิวเตอร์ และจำ COM Port เอาไว้
4. Upload Firmware ลงบอร์ด 

### 2. 🧠 FastAPI Backend (Server)
สำหรับประมวลผลวิดีโอ (OpenCV) โมเดล YOLOv11 และเชื่อมต่อ Serial ให้แขนกล:
```bash
cd server
pip install -r requirements.txt
python app.py
```
> Server จะเริ่มการทำงานที่ `http://localhost:5000` (FastAPI) พร้อมรับคำสั่งไปขับแขนกล และเปิดพอร์ต MJPEG 

### 3. 🌐 React Frontend (Client)
Dashboard สำหรับแสดงผลกล้อง ควบคุมจุดพิกัด และ Pick & Place
```bash
cd client
npm install
npm run dev
```
> สามารถเปิดเข้าใช้งาน UI แบบ Modern ได้ที่ `http://localhost:5173` หากใช้ Vite ทั่วไป

---

## 📚 User Manual & Usage Guide

1. **เริ่มการทำงาน**
   - รัน FastAPI (`app.py`) ทิ้งไว้ในเบื้องหลัง จากนั้นเปิด React Frontend และเข้าไปที่เบราเซอร์ (`localhost:5173`)
   
2. **การเชื่อมต่อกับ แขนกล (ESP32)**
   - ไปยังส่วนของการควบคุมทางขวามือ หรือปุ่ม Device List
   - รายชื่อพอร์ต (เช่น `COM3` หรือ `/dev/ttyUSB0`) จะปรากฎ (ผ่านการตรวจสอบด้วย `/robot/ports` ของ Backend)
   - เลือกพอร์ตที่ถูกต้อง แล้วกด **Connect** แขนกลจะแสดงสถานะ Ready 

3. **การตั้งค่ากล้อง (Camera Settings)**
   - ในหน้า Dashboard จะพบกับ Live Feed ทันที และมีตัวเลือกให้กำหนด `Camera Index` หากกล้องไม่ติด กรุณากดปุ่มเพื่อเลือกสลับกล้องแล้วกด Start
   - รอภาพโผล่ ระบบจะโชว์ว่า YOLO Model ถูกโหลดและพร้อมใช้งาน 

4. **การทำ ArUco Calibration (การตั้งค่ามุมมองภาพ)**
   - ใช้ ArUco Markers (`DICT_4X4_50`) จำนวน 4 ชิ้น (ID: 0, 1, 2, 3) 
   - วางแผ่น A4 ของฐาน ArUco ชิดไปทางด้านหุ่นยนต์ โดยให้ ID 0, 1 อยู่ใกล้เคียงหุ่นที่สุด (ระยะการติดตั้งในโค้ดอยู่ที่ระยะแกน X เริ่ม 50mm สิ้นสุด 260mm)
   - กดปุ่ม **Calibrate** ในแอป หากครบ 4 มุม ระบบจะโชว์กล่องตารางสีเขียว (หรือสีอื่นๆ) บอกอาณาเขตพื้นที่ทำงานให้โดยอัตโนมัติ ทำให้การประมวลผลจับวัตถุแม่นยำในโลกจริง

5. **โหมด Manual Control**
   - **X, Y, Z Sliders**: ใช้ลากเพื่อขยับแขนกลหาพิกัด XYZ ทันที 
   - **🏠 HOME**: แขนกลจะยกขึ้นเข้าสู่ท่าพื้นฐานอย่างนุ่มนวลที่ 90 องศาทุกแกน 
   - **✊ GRAB / 🖐️ RELEASE**: ใช้สั่งให้ Gripper กำหดเปิด-ปิด ด้วยคำสั่งตรงผ่าน Serial (`GRIP_OPEN` / `GRIP_CLOSE`)
   
6. **Pick and Place Mode**
   - กล้องจะมองเห็นวัตถุตามรุ่นโมเดล (เช่น ไขควง น็อต กล่อง) พร้อมขึ้น Bounding Box
   - คลิกเลือกที่วัตถุในรายการระบบ จะส่ง `Sequence โค้ด` นำพาปลายแขนให้ไปอยู่เหนือวัตถุ
   - แขนกลจะค่อยๆ ลดระดับลง (ค่า `Z`) คีบเป้าหมาย ยกขึ้น และวางเป้าหมายให้เองโดยอิงจากพิกัดอัจฉริยะ (IK)

7. **ใช้งานโปรแกรมจำลอง 3D (3D Simulator)**
   - สามารถพิมพ์คำสั่ง: `python robot_arm_sim.py` ในคอมพิวเตอร์เพื่อเปิด GUI ขึ้นมา
   - ใช้จำลองแขนกลเพื่อทดสอบพิกัด Inverse Kinematics ก่อนส่งไปแขนจริง พร้อมเชื่อมต่อ Serial ไปยัง ESP32 หากเสียบสายไว้

8. **การทดสอบ YOLO ออฟไลน์ผ่าน Jupyter**
   - รันสภาพแวดล้อมเพื่อทดสอบ: `jupyter notebook`
   - รันไฟล์ `test_yolo26.ipynb` ในโฟลเดอร์รันงานเพื่อดูพรีวิวความแม่นยำและเวลาที่ใช้ Inference ด้วยภาพตัวอย่าง

## 📁 File Structure

```text
WebRobot/
├── client/                     # วางระบบ React Frontend + Vite
│   ├── src/                    # โค้ด Components, APIs, และ CSS ยุคใหม่ 
│   └── package.json            # Node Dependencies
├── server/                     # ระบบ Backend ตัวกลางเชื่อม ESP32 และรัน AI
│   ├── app.py                  # FastAPI แหล่งรัน YOLOv11 และส่งคำสั่ง Serial COM
│   ├── requirements.txt
│   └── *.pt (e.g. yolo11.pt)   # Weights ของ YOLOv11 Model (ถ้าวางโฟลเดอร์นี้) 
├── esp32/                      
│   └── robot_arm_4dof_final (1).ino  # เฟิร์มแวร์ C++ อิง PCA9685/ESP32Servo 
├── URDF_Robot_arm/             # ข้อมูลสเปกขนาดโครงสร้าง (อิงโมเดล 3D) 
├── robot_arm_sim.py            # โปรแกรม Simulation แขนกล 3D บน PC
├── test_yolo26.ipynb           # Jupyter Notebook สำหรับทดสอบ YOLO โดยตรง
├── best.pt                     # โมเดลสำเร็จรูป (ควรนำใส่ไปใน server/ เพื่อให้ Web เรียกใช้)
└── README.md                   # คู่มือและโครงสร้างโครงการ
```
