/*
 * ESP32 Robot Arm Controller for meArm
 * WebSocket-based XYZ control with 4 Servo motors
 * 
 * Compatible with meArm Inverse Kinematics:
 * - Receives servo angles calculated from XYZ coordinates by web frontend
 * - Home position corresponds to meArm (0, 100, 50) mm
 * - Ref: https://github.com/phenoptix/meArm-1
 * 
 * Hardware Setup:
 * - ESP32 Development Board
 * - Servo 1 (Base rotation): GPIO 13 - rotates left/right
 * - Servo 2 (Shoulder): GPIO 12 - raises/lowers first arm segment
 * - Servo 3 (Elbow): GPIO 14 - raises/lowers second arm segment  
 * - Servo 4 (Gripper): GPIO 27 - opens/closes claw
 * 
 * Libraries Required:
 * - ESP32Servo (by Kevin Harrington)
 * - WebSocketsServer (by Markus Sattler)
 * - ArduinoJson (by Benoit Blanchon)
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// ============================================
// Configuration
// ============================================

// WiFi Access Point Settings
const char* AP_SSID = "RobotArm_AP";
const char* AP_PASSWORD = "12345678";

// Or use Station Mode (connect to existing WiFi)
// Uncomment these lines and comment out AP mode to use
// const char* STA_SSID = "YourWiFiName";
// const char* STA_PASSWORD = "YourWiFiPassword";
// #define USE_STATION_MODE

// WebSocket Port
const int WS_PORT = 81;

// Servo Pin Assignments
const int SERVO1_PIN = 13;  // Base rotation
const int SERVO2_PIN = 12;  // Shoulder
const int SERVO3_PIN = 14;  // Elbow
const int GRIPPER_PIN = 27; // Gripper (optional)

// Servo Limits
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

// Movement Settings
const int SERVO_STEP_DELAY = 15;  // ms between each degree movement (for smooth motion)
const bool SMOOTH_MOVEMENT = true;

// ============================================
// Global Objects
// ============================================

WebSocketsServer webSocket = WebSocketsServer(WS_PORT);
Servo servo1;  // Base
Servo servo2;  // Shoulder
Servo servo3;  // Elbow
Servo gripper; // Gripper

// Current positions
int currentS1 = 90;
int currentS2 = 90;
int currentS3 = 90;
int currentGripper = 90;

// Target positions
int targetS1 = 90;
int targetS2 = 90;
int targetS3 = 90;

// Status flags
bool stopFlag = false;
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 1000; // Send status every 1 second

// ============================================
// Function Prototypes
// ============================================

void setupWiFi();
void setupServos();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void handleCommand(uint8_t num, const char* payload);
void moveServoSmooth(Servo& servo, int& current, int target, int stepDelay);
void moveToPosition(int s1, int s2, int s3);
void homePosition();
void sendStatus(uint8_t num);
int constrainAngle(int angle);

// ============================================
// Setup
// ============================================

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=================================");
    Serial.println("  ESP32 Robot Arm Controller");
    Serial.println("=================================");
    
    // Initialize Servos
    setupServos();
    
    // Initialize WiFi
    setupWiFi();
    
    // Initialize WebSocket
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    Serial.println("WebSocket server started on port " + String(WS_PORT));
    Serial.println("System ready!");
    Serial.println("=================================");
    
    // Move to home position
    homePosition();
}

// ============================================
// Main Loop
// ============================================

void loop() {
    webSocket.loop();
    
    // Smooth movement update
    if (SMOOTH_MOVEMENT && !stopFlag) {
        updateServoPositions();
    }
    
    // Periodic status broadcast
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        broadcastStatus();
        lastStatusTime = millis();
    }
}

// ============================================
// WiFi Setup
// ============================================

void setupWiFi() {
#ifdef USE_STATION_MODE
    // Station Mode - Connect to existing WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(STA_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect! Starting AP mode...");
        startAccessPoint();
    }
#else
    // Access Point Mode
    startAccessPoint();
#endif
}

void startAccessPoint() {
    Serial.println("Starting Access Point...");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.println("=================================");
    Serial.println("  ACCESS POINT STARTED");
    Serial.println("=================================");
    Serial.print("  SSID: ");
    Serial.println(AP_SSID);
    Serial.print("  Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("  IP Address: ");
    Serial.println(IP);
    Serial.println("=================================");
}

// ============================================
// Servo Setup
// ============================================

void setupServos() {
    // Allow allocation of all timers for servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Attach servos to pins
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo3.setPeriodHertz(50);
    gripper.setPeriodHertz(50);
    
    servo1.attach(SERVO1_PIN, 500, 2400);
    servo2.attach(SERVO2_PIN, 500, 2400);
    servo3.attach(SERVO3_PIN, 500, 2400);
    gripper.attach(GRIPPER_PIN, 500, 2400);
    
    Serial.println("Servos initialized:");
    Serial.println("  Servo1 (Base): GPIO " + String(SERVO1_PIN));
    Serial.println("  Servo2 (Shoulder): GPIO " + String(SERVO2_PIN));
    Serial.println("  Servo3 (Elbow): GPIO " + String(SERVO3_PIN));
    Serial.println("  Gripper: GPIO " + String(GRIPPER_PIN));
}

// ============================================
// WebSocket Event Handler
// ============================================

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
                
                // Send welcome message
                sendStatus(num);
            }
            break;
            
        case WStype_TEXT:
            Serial.printf("[%u] Received: %s\n", num, payload);
            handleCommand(num, (const char*)payload);
            break;
            
        case WStype_PING:
            // Pong is automatically sent
            break;
            
        case WStype_PONG:
            break;
    }
}

// ============================================
// Command Handler
// ============================================

void handleCommand(uint8_t num, const char* payload) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
    }
    
    const char* type = doc["type"];
    
    if (strcmp(type, "move") == 0) {
        // Move command with XYZ and servo angles
        stopFlag = false;
        
        if (doc.containsKey("s1")) {
            targetS1 = constrainAngle(doc["s1"]);
        }
        if (doc.containsKey("s2")) {
            targetS2 = constrainAngle(doc["s2"]);
        }
        if (doc.containsKey("s3")) {
            targetS3 = constrainAngle(doc["s3"]);
        }
        
        Serial.printf("Target: S1=%d, S2=%d, S3=%d\n", targetS1, targetS2, targetS3);
        
        if (!SMOOTH_MOVEMENT) {
            moveToPosition(targetS1, targetS2, targetS3);
        }
    }
    else if (strcmp(type, "home") == 0) {
        Serial.println("Moving to HOME position");
        stopFlag = false;
        homePosition();
    }
    else if (strcmp(type, "grab") == 0) {
        Serial.println("Gripper: CLOSE");
        gripper.write(30);
        currentGripper = 30;
    }
    else if (strcmp(type, "release") == 0) {
        Serial.println("Gripper: OPEN");
        gripper.write(120);
        currentGripper = 120;
    }
    else if (strcmp(type, "stop") == 0) {
        Serial.println("EMERGENCY STOP!");
        stopFlag = true;
        targetS1 = currentS1;
        targetS2 = currentS2;
        targetS3 = currentS3;
    }
    else if (strcmp(type, "setServo") == 0) {
        // Direct servo control
        int servoNum = doc["servo"];
        int angle = constrainAngle(doc["angle"]);
        
        switch(servoNum) {
            case 1:
                targetS1 = angle;
                if (!SMOOTH_MOVEMENT) servo1.write(angle);
                break;
            case 2:
                targetS2 = angle;
                if (!SMOOTH_MOVEMENT) servo2.write(angle);
                break;
            case 3:
                targetS3 = angle;
                if (!SMOOTH_MOVEMENT) servo3.write(angle);
                break;
        }
    }
    
    // Send acknowledgment
    sendStatus(num);
}

// ============================================
// Servo Movement Functions
// ============================================

void updateServoPositions() {
    // Smooth movement towards target positions
    if (currentS1 != targetS1) {
        if (currentS1 < targetS1) currentS1++;
        else currentS1--;
        servo1.write(currentS1);
    }
    
    if (currentS2 != targetS2) {
        if (currentS2 < targetS2) currentS2++;
        else currentS2--;
        servo2.write(currentS2);
    }
    
    if (currentS3 != targetS3) {
        if (currentS3 < targetS3) currentS3++;
        else currentS3--;
        servo3.write(currentS3);
    }
    
    // Small delay for smooth movement
    if (currentS1 != targetS1 || currentS2 != targetS2 || currentS3 != targetS3) {
        delay(SERVO_STEP_DELAY);
    }
}

void moveToPosition(int s1, int s2, int s3) {
    Serial.printf("Moving to: S1=%d, S2=%d, S3=%d\n", s1, s2, s3);
    
    servo1.write(s1);
    servo2.write(s2);
    servo3.write(s3);
    
    currentS1 = s1;
    currentS2 = s2;
    currentS3 = s3;
}

void homePosition() {
    targetS1 = 90;
    targetS2 = 90;
    targetS3 = 90;
    
    if (!SMOOTH_MOVEMENT) {
        moveToPosition(90, 90, 90);
    }
    
    Serial.println("HOME position set");
}

int constrainAngle(int angle) {
    if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    return angle;
}

// ============================================
// Status Functions
// ============================================

void sendStatus(uint8_t num) {
    StaticJsonDocument<256> doc;
    
    doc["type"] = "status";
    doc["s1"] = currentS1;
    doc["s2"] = currentS2;
    doc["s3"] = currentS3;
    doc["gripper"] = currentGripper;
    doc["stopFlag"] = stopFlag;
    
    String output;
    serializeJson(doc, output);
    
    webSocket.sendTXT(num, output);
}

void broadcastStatus() {
    StaticJsonDocument<256> doc;
    
    doc["type"] = "status";
    doc["s1"] = currentS1;
    doc["s2"] = currentS2;
    doc["s3"] = currentS3;
    doc["gripper"] = currentGripper;
    
    String output;
    serializeJson(doc, output);
    
    webSocket.broadcastTXT(output);
}
