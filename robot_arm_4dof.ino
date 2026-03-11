#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ============================================================
//  ความยาวแขน (mm) – แก้ให้ตรงกับหุ่นจริง
// ============================================================
const float L1 = 95.0;
const float L2 = 120.0;
const float L3 = 120.0;
const float L4 = 130.0;

// ============================================================
//  PWM Range สำหรับ MG995 / MG90S
//  102 = 500µs = 0°  |  512 = 2500µs = 180°
// ============================================================
#define SERVOMIN 102
#define SERVOMAX 512

// ============================================================
//  Pin 13 = Potentiometer ควบคุมก้มเงย gripper (Wrist Tilt)
//  อ่านค่า 0–4095 (ESP32 ADC 12-bit) แล้วแปลงเป็น offset °
//  ตรงกลาง = 0°  (gripper ตั้งฉากพื้น)
//  หมุนขึ้น   = เงย (บวก)
//  หมุนลง    = ก้ม (ลบ)
//  ช่วงที่ปรับได้: WRIST_TILT_MIN ถึง WRIST_TILT_MAX องศา
// ============================================================
#define WRIST_TILT_PIN 13
#define WRIST_TILT_MIN -60  // ก้มสุด (องศา)
#define WRIST_TILT_MAX 60   // เงยสุด (องศา)
#define ADC_RESOLUTION 4095 // ESP32 = 12-bit

// Dead-zone: ถ้า offset น้อยกว่านี้ → ถือว่า 0 (ลดการสั่นของ pot)
#define TILT_DEADZONE 2.0f

// ============================================================
//  ความเร็วการเคลื่อนที่ (Interpolation ใน loop)
// ============================================================
float stepSize = 0.5; // mm ต่อ loop
int speedDelay = 25;  // ms ต่อ loop

// ============================================================
//  พิกัดปัจจุบัน และเป้าหมาย
// ============================================================
float currentX = 0.0, currentY = 120.0, currentZ = 85.0;
float targetX = 0.0, targetY = 120.0, targetZ = 85.0;

// ============================================================
//  มุมปัจจุบันของเซอร์โว
// ============================================================
float currentAngleBase = 90.0;
float currentAngleShoulder = 90.0;
float currentAngleElbow = 90.0;
float currentAngleWrist = 90.0;

// ============================================================
//  ค่า Wrist Tilt Offset ล่าสุด (อ่านจาก pin 13)
// ============================================================
float wristTiltOffset = 0.0;

// ============================================================
//  ฟังก์ชัน Forward Declaration
// ============================================================
int angleToPWM(float angle);
float readWristTilt();
void calculateAndMoveIK(float x, float y, float z, float wristOffset);
void moveToHome90();

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  // ตั้ง pin 13 เป็น input (ADC)
  // ESP32 ไม่ต้องการ pinMode สำหรับ analogRead
  // แต่ถ้าใช้ Arduino Uno ให้ uncomment บรรทัดด้านล่าง:
  // pinMode(WRIST_TILT_PIN, INPUT);

  Serial.println("Initializing...");

  // ── Step 1: ส่ง PWM 90° ไปทุกตัวทันที ──────────────────
  int pwm90 = angleToPWM(90.0);
  pwm.setPWM(0, 0, pwm90);
  pwm.setPWM(1, 0, pwm90);
  pwm.setPWM(2, 0, pwm90);
  pwm.setPWM(3, 0, pwm90);

  // ── Step 2: รอให้เซอร์โวถึง 90° ─────────────────────────
  delay(2000);

  // ── Step 3: อัปเดตตัวแปร ────────────────────────────────
  currentAngleBase = 90.0;
  currentAngleShoulder = 90.0;
  currentAngleElbow = 90.0;
  currentAngleWrist = 90.0;

  currentX = 0.0;
  targetX = 0.0;
  currentY = 120.0;
  targetY = 120.0;
  currentZ = 85.0;
  targetZ = 85.0;

  Serial.println("Ready! Send X,Y,Z or HOME");
  Serial.println(
      "Pin 13 = Wrist Tilt Pot  (center=straight, CW=tilt up, CCW=tilt down)");
}

// ============================================================
//  Loop
// ============================================================
void loop() {

  // ── Wrist Tilt: ล็อกให้ตั้งฉากพื้นตลอดเวลา (offset = 0) ─
  // หากต้องการเปิดใช้ potentiometer ก้มเงย ให้ uncomment บรรทัดด้านล่าง:
  // wristTiltOffset = readWristTilt();
  wristTiltOffset = 0.0f; // ← ตั้งฉากพื้นเสมอ

  // ── รับคำสั่งจาก Python ──────────────────────────────────
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data == "HOME") {
      moveToHome90();

    } else {
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);

      if (c1 > 0 && c2 > 0) {
        targetX = data.substring(0, c1).toFloat();
        targetY = data.substring(c1 + 1, c2).toFloat();
        targetZ = data.substring(c2 + 1).toFloat();
      }
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // ── Interpolation ─────────────────────────────────────────
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  float dz = targetZ - currentZ;
  float distance = sqrt(dx * dx + dy * dy + dz * dz);

  if (distance > stepSize) {
    currentX += (dx / distance) * stepSize;
    currentY += (dy / distance) * stepSize;
    currentZ += (dz / distance) * stepSize;
    calculateAndMoveIK(currentX, currentY, currentZ, wristTiltOffset);
    delay(speedDelay);

  } else if (distance > 0.001) {
    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;
    calculateAndMoveIK(currentX, currentY, currentZ, wristTiltOffset);

  } else {
    // ถึงเป้าแล้ว แต่ยัง re-apply ทุก loop เพื่อ track ค่า pot
    calculateAndMoveIK(currentX, currentY, currentZ, wristTiltOffset);
    delay(speedDelay);
  }
}

// ============================================================
//  อ่านค่า Potentiometer จาก pin 13
//  คืนค่า offset เป็นองศา: ลบ = ก้ม, บวก = เงย
//  ตรงกลาง ADC (≈2048) → 0°
// ============================================================
float readWristTilt() {
  int raw = analogRead(WRIST_TILT_PIN); // 0–4095 (ESP32)

  // แปลงเป็น offset องศา: ตรงกลาง ADC = 0°
  float offset = ((float)raw / (float)ADC_RESOLUTION * 2.0f - 1.0f) *
                 ((WRIST_TILT_MAX - WRIST_TILT_MIN) / 2.0f);

  // Dead-zone รอบ 0°
  if (fabs(offset) < TILT_DEADZONE)
    offset = 0.0f;

  return offset;
}

// ============================================================
//  แปลงองศา → PWM tick
// ============================================================
int angleToPWM(float angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  return map((long)angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ============================================================
//  คำนวณ IK → สั่งเซอร์โว
//
//  wristOffset (°):
//    = 0   → gripper ตั้งฉากพื้นเป๊ะ (ชี้ลงแนวดิ่ง)
//    > 0   → เงยขึ้น (tilt forward/up)
//    < 0   → ก้มลง  (tilt forward/down)
//
//  สูตร:
//    wristCompDeg = -(theta2 - theta3)   ← ชดเชยให้ตั้งฉาก
//    wristServoDeg = 90 + wristCompDeg + wristOffset
// ============================================================
void calculateAndMoveIK(float x, float y, float z, float wristOffset) {
  // Wrist joint = เลื่อน tip ขึ้นไป L4 ในแกน Z
  float wx = x;
  float wy = y;
  float wz = z + L4;

  float r_w = sqrt(wx * wx + wy * wy);
  if (r_w < 0.001f)
    return;

  float theta1 = atan2(wy, wx);
  float r = r_w;
  float z_adj = wz - L1;
  float d = sqrt(r * r + z_adj * z_adj);
  if (d > (L2 + L3))
    return;

  float cosTheta3 = (d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
  cosTheta3 = constrain(cosTheta3, -1.0, 1.0);

  float theta3 = acos(cosTheta3);
  float alpha = atan2(z_adj, r);
  float beta = atan2(L3 * sin(theta3), L2 + L3 * cos(theta3));
  float theta2 = alpha + beta;

  // ── Wrist: ตั้งฉาก + offset จาก pin 13 ─────────────────
  float wristCompDeg =
      (theta2 - theta3) * (180.0 / PI); // ชดเชยให้ตั้งฉากพื้น (แก้กลับด้าน)
  float wristServoDeg = 90.0 + wristCompDeg + wristOffset; // + offset ก้มเงย
  wristServoDeg = constrain(wristServoDeg, 0.0, 180.0);

  // อัปเดตมุมปัจจุบัน
  currentAngleBase = theta1 * (180.0 / PI);
  currentAngleShoulder = theta2 * (180.0 / PI);
  currentAngleElbow = theta3 * (180.0 / PI);
  currentAngleWrist = wristServoDeg;

  pwm.setPWM(0, 0, angleToPWM(currentAngleBase));
  pwm.setPWM(1, 0, angleToPWM(currentAngleShoulder));
  pwm.setPWM(2, 0, angleToPWM(currentAngleElbow));
  pwm.setPWM(3, 0, angleToPWM(currentAngleWrist));
}

// ============================================================
//  moveToHome90
// ============================================================
void moveToHome90() {
  Serial.println("Homing...");

  float startB = currentAngleBase;
  float startS = currentAngleShoulder;
  float startE = currentAngleElbow;
  float startW = currentAngleWrist;

  for (int i = 0; i <= 100; i++) {
    float t = (float)i / 100.0;
    float ease = t * t * (3.0 - 2.0 * t); // smoothstep

    pwm.setPWM(0, 0, angleToPWM(startB + (90.0 - startB) * ease));
    pwm.setPWM(1, 0, angleToPWM(startS + (90.0 - startS) * ease));
    pwm.setPWM(2, 0, angleToPWM(startE + (90.0 - startE) * ease));
    pwm.setPWM(3, 0, angleToPWM(startW + (90.0 - startW) * ease));
    delay(20);
  }

  currentAngleBase = 90.0;
  currentAngleShoulder = 90.0;
  currentAngleElbow = 90.0;
  currentAngleWrist = 90.0;

  currentX = 0.0;
  targetX = 0.0;
  currentY = 120.0;
  targetY = 120.0;
  currentZ = 85.0;
  targetZ = 85.0;

  while (Serial.available() > 0) {
    Serial.read();
  }
  Serial.println("Home complete!");
}
