/**
 * Robot Arm Control System
 * WebSocket-based XYZ coordinate controller for ESP32 with 3 servos
 */

// ============================================
// Configuration
// ============================================
const CONFIG = {
    // Arm dimensions (in mm) - adjust based on your robot
    ARM_LENGTH_1: 80,   // Shoulder to Elbow
    ARM_LENGTH_2: 100,  // Elbow to End Effector

    // Servo limits
    SERVO_MIN: 0,
    SERVO_MAX: 180,

    // Default positions
    DEFAULT_X: 0,
    DEFAULT_Y: 0,
    DEFAULT_Z: 50,

    // WebSocket
    WS_RECONNECT_INTERVAL: 3000,
    UPDATE_THROTTLE: 50,  // ms between updates

    // Object Detection Server
    DETECTION_SERVER: 'http://localhost:5000'
};

// ============================================
// State Management
// ============================================
const state = {
    x: 0,
    y: 0,
    z: 50,
    servo1: 90,  // Base rotation
    servo2: 90,  // Shoulder
    servo3: 90,  // Elbow
    isConnected: false,
    ws: null,
    lastUpdate: 0
};

// Object Detection State
const detectionState = {
    isCameraActive: false,
    detectedObjects: [],
    selectedObjectId: null,
    isExecutingSequence: false
};

// ============================================
// DOM Elements
// ============================================
const elements = {
    // Sliders
    xSlider: document.getElementById('xSlider'),
    ySlider: document.getElementById('ySlider'),
    zSlider: document.getElementById('zSlider'),

    // Inputs
    xInput: document.getElementById('xInput'),
    yInput: document.getElementById('yInput'),
    zInput: document.getElementById('zInput'),

    // Value displays
    xValue: document.getElementById('xValue'),
    yValue: document.getElementById('yValue'),
    zValue: document.getElementById('zValue'),

    // Current position
    currentX: document.getElementById('currentX'),
    currentY: document.getElementById('currentY'),
    currentZ: document.getElementById('currentZ'),

    // Servo displays
    servo1Value: document.getElementById('servo1Value'),
    servo2Value: document.getElementById('servo2Value'),
    servo3Value: document.getElementById('servo3Value'),
    servo1Gauge: document.getElementById('servo1Gauge'),
    servo2Gauge: document.getElementById('servo2Gauge'),
    servo3Gauge: document.getElementById('servo3Gauge'),

    // Connection
    connectionStatus: document.getElementById('connectionStatus'),
    espIP: document.getElementById('espIP'),
    wsPort: document.getElementById('wsPort'),
    btnConnect: document.getElementById('btnConnect'),

    // Actions
    btnHome: document.getElementById('btnHome'),
    btnGrab: document.getElementById('btnGrab'),
    btnRelease: document.getElementById('btnRelease'),
    btnStop: document.getElementById('btnStop'),

    // Canvas
    armCanvas: document.getElementById('armCanvas')
};

// ============================================
// Canvas Visualization
// ============================================
class ArmVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.currentView = 'side'; // side, top, front, 3d
        this.resize();
        window.addEventListener('resize', () => this.resize());
    }

    setView(view) {
        this.currentView = view;
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = 300;
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
        this.scale = 1.5;
    }

    draw(x, y, z, servo1, servo2, servo3) {
        const ctx = this.ctx;
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw grid
        this.drawGrid();

        // Calculate 3D arm positions
        const armPos = this.calculateArmPositions(x, y, z, servo1, servo2, servo3);

        // Draw based on current view
        switch (this.currentView) {
            case 'side':
                this.drawSideView(armPos, x, y, z);
                break;
            case 'top':
                this.drawTopView(armPos, x, y, z);
                break;
            case 'front':
                this.drawFrontView(armPos, x, y, z);
                break;
            case '3d':
                this.draw3DView(armPos, x, y, z);
                break;
        }

        // Draw position indicator
        this.drawPositionIndicator(x, y, z);
    }

    calculateArmPositions(x, y, z, servo1, servo2, servo3) {
        // Calculate 3D positions of arm joints
        const baseAngle = (servo1 - 90) * Math.PI / 180;
        const shoulderAngle = (90 - servo2) * Math.PI / 180;
        const elbowAngle = (90 - servo3) * Math.PI / 180;

        const L1 = CONFIG.ARM_LENGTH_1;
        const L2 = CONFIG.ARM_LENGTH_2;

        // Base is at origin
        const base = { x: 0, y: 0, z: 0 };

        // Shoulder joint position (after first arm segment)
        const shoulder = {
            x: Math.cos(baseAngle) * Math.cos(shoulderAngle) * L1,
            y: Math.sin(baseAngle) * Math.cos(shoulderAngle) * L1,
            z: Math.sin(shoulderAngle) * L1
        };

        // End effector position
        const totalAngle = shoulderAngle + elbowAngle - Math.PI / 2;
        const end = {
            x: shoulder.x + Math.cos(baseAngle) * Math.cos(totalAngle) * L2,
            y: shoulder.y + Math.sin(baseAngle) * Math.cos(totalAngle) * L2,
            z: shoulder.z + Math.sin(totalAngle) * L2
        };

        return { base, shoulder, end };
    }

    // Side View (XZ Plane) - looking from Y axis
    drawSideView(armPos, x, y, z) {
        const ctx = this.ctx;
        const baseY = this.canvas.height - 60;

        this.drawViewLabel('SIDE VIEW (XZ Plane)');
        this.drawAxes('X', 'Z');

        // Transform to screen coordinates
        const base = { x: this.centerX, y: baseY };
        const shoulder = {
            x: this.centerX + armPos.shoulder.x * this.scale,
            y: baseY - armPos.shoulder.z * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.x * this.scale,
            y: baseY - armPos.end.z * this.scale
        };

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, shoulder.x, shoulder.y, '#00f0ff', 12);
        this.drawArmSegment(shoulder.x, shoulder.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(shoulder.x, shoulder.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    // Top View (XY Plane) - looking from Z axis (bird's eye)
    drawTopView(armPos, x, y, z) {
        const ctx = this.ctx;

        this.drawViewLabel('TOP VIEW (XY Plane)');
        this.drawAxes('X', 'Y');

        // Transform to screen coordinates (X right, Y up on screen)
        const base = { x: this.centerX, y: this.centerY };
        const shoulder = {
            x: this.centerX + armPos.shoulder.x * this.scale,
            y: this.centerY - armPos.shoulder.y * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.x * this.scale,
            y: this.centerY - armPos.end.y * this.scale
        };

        // Draw range circle
        this.drawRangeCircle(base.x, base.y, (CONFIG.ARM_LENGTH_1 + CONFIG.ARM_LENGTH_2) * this.scale);

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, shoulder.x, shoulder.y, '#00f0ff', 12);
        this.drawArmSegment(shoulder.x, shoulder.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(shoulder.x, shoulder.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    // Front View (YZ Plane) - looking from X axis
    drawFrontView(armPos, x, y, z) {
        const ctx = this.ctx;
        const baseY = this.canvas.height - 60;

        this.drawViewLabel('FRONT VIEW (YZ Plane)');
        this.drawAxes('Y', 'Z');

        // Transform to screen coordinates
        const base = { x: this.centerX, y: baseY };
        const shoulder = {
            x: this.centerX + armPos.shoulder.y * this.scale,
            y: baseY - armPos.shoulder.z * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.y * this.scale,
            y: baseY - armPos.end.z * this.scale
        };

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, shoulder.x, shoulder.y, '#00f0ff', 12);
        this.drawArmSegment(shoulder.x, shoulder.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(shoulder.x, shoulder.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    // 3D Isometric View
    draw3DView(armPos, x, y, z) {
        const ctx = this.ctx;

        this.drawViewLabel('3D ISOMETRIC VIEW');

        // Isometric projection angles
        const isoAngle = Math.PI / 6; // 30 degrees
        const scale3D = this.scale * 0.8;

        // Transform 3D to 2D isometric
        const toIso = (p) => ({
            x: this.centerX + (p.x - p.y) * Math.cos(isoAngle) * scale3D,
            y: this.centerY + 40 + (p.x + p.y) * Math.sin(isoAngle) * scale3D - p.z * scale3D
        });

        const base = toIso(armPos.base);
        const shoulder = toIso(armPos.shoulder);
        const end = toIso(armPos.end);

        // Draw 3D axes
        this.draw3DAxes(toIso);

        // Draw ground shadow
        this.drawGroundShadow(armPos, toIso);

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, shoulder.x, shoulder.y, '#00f0ff', 12);
        this.drawArmSegment(shoulder.x, shoulder.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(shoulder.x, shoulder.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    drawViewLabel(label) {
        const ctx = this.ctx;
        ctx.font = '12px Orbitron';
        ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.textAlign = 'left';
        ctx.fillText(label, 10, 20);
    }

    drawAxes(hLabel, vLabel) {
        const ctx = this.ctx;
        const baseY = this.currentView === 'top' ? this.centerY : this.canvas.height - 60;

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);

        // Horizontal axis
        ctx.beginPath();
        ctx.moveTo(50, baseY);
        ctx.lineTo(this.canvas.width - 50, baseY);
        ctx.stroke();

        // Vertical axis
        ctx.beginPath();
        ctx.moveTo(this.centerX, 30);
        ctx.lineTo(this.centerX, this.canvas.height - 30);
        ctx.stroke();

        ctx.setLineDash([]);

        // Labels
        ctx.font = '10px Orbitron';
        ctx.fillStyle = 'rgba(255, 68, 102, 0.7)';
        ctx.fillText(hLabel, this.canvas.width - 40, baseY - 5);
        ctx.fillStyle = 'rgba(68, 136, 255, 0.7)';
        ctx.fillText(vLabel, this.centerX + 5, 40);
    }

    draw3DAxes(toIso) {
        const ctx = this.ctx;
        const origin = toIso({ x: 0, y: 0, z: 0 });
        const axisLen = 50;

        ctx.lineWidth = 2;
        ctx.setLineDash([]);

        // X axis (red)
        const xEnd = toIso({ x: axisLen, y: 0, z: 0 });
        ctx.strokeStyle = '#ff4466';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(xEnd.x, xEnd.y);
        ctx.stroke();
        ctx.fillStyle = '#ff4466';
        ctx.fillText('X', xEnd.x + 5, xEnd.y);

        // Y axis (green)
        const yEnd = toIso({ x: 0, y: axisLen, z: 0 });
        ctx.strokeStyle = '#44ff88';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(yEnd.x, yEnd.y);
        ctx.stroke();
        ctx.fillStyle = '#44ff88';
        ctx.fillText('Y', yEnd.x - 15, yEnd.y);

        // Z axis (blue)
        const zEnd = toIso({ x: 0, y: 0, z: axisLen });
        ctx.strokeStyle = '#4488ff';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(zEnd.x, zEnd.y);
        ctx.stroke();
        ctx.fillStyle = '#4488ff';
        ctx.fillText('Z', zEnd.x + 5, zEnd.y);
    }

    drawGroundShadow(armPos, toIso) {
        const ctx = this.ctx;

        // Project arm onto ground (z=0)
        const base = toIso({ x: 0, y: 0, z: 0 });
        const shoulder = toIso({ x: armPos.shoulder.x, y: armPos.shoulder.y, z: 0 });
        const end = toIso({ x: armPos.end.x, y: armPos.end.y, z: 0 });

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 4;
        ctx.setLineDash([5, 5]);

        ctx.beginPath();
        ctx.moveTo(base.x, base.y);
        ctx.lineTo(shoulder.x, shoulder.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();

        ctx.setLineDash([]);
    }

    drawRangeCircle(x, y, radius) {
        const ctx = this.ctx;
        ctx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
    }

    drawBaseCircle(x, y) {
        const ctx = this.ctx;
        ctx.fillStyle = 'rgba(0, 240, 255, 0.2)';
        ctx.strokeStyle = '#00f0ff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 25, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
    }

    drawGrid() {
        const ctx = this.ctx;
        ctx.strokeStyle = 'rgba(0, 240, 255, 0.08)';
        ctx.lineWidth = 1;

        for (let x = 0; x < this.canvas.width; x += 30) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.canvas.height);
            ctx.stroke();
        }

        for (let y = 0; y < this.canvas.height; y += 30) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.canvas.width, y);
            ctx.stroke();
        }
    }

    drawArmSegment(x1, y1, x2, y2, color, width) {
        const ctx = this.ctx;
        ctx.shadowColor = color;
        ctx.shadowBlur = 15;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        ctx.shadowBlur = 0;
    }

    drawJoint(x, y, radius, color) {
        const ctx = this.ctx;
        const gradient = ctx.createRadialGradient(x, y, 0, x, y, radius * 1.5);
        gradient.addColorStop(0, color);
        gradient.addColorStop(0.5, color);
        gradient.addColorStop(1, 'transparent');
        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(x, y, radius * 1.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#0a0a0f';
        ctx.beginPath();
        ctx.arc(x, y, radius * 0.6, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    drawEndEffector(x, y) {
        const ctx = this.ctx;
        ctx.strokeStyle = '#ffcc00';
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x - 8, y + 15);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + 8, y + 15);
        ctx.stroke();
        ctx.shadowColor = '#ffcc00';
        ctx.shadowBlur = 20;
        ctx.fillStyle = '#ffcc00';
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
    }

    drawPositionIndicator(x, y, z) {
        const ctx = this.ctx;
        ctx.font = 'bold 14px Orbitron';
        ctx.textAlign = 'right';
        const startX = this.canvas.width - 20;
        const startY = 30;
        ctx.fillStyle = '#ff4466';
        ctx.fillText(`X: ${x}`, startX, startY);
        ctx.fillStyle = '#44ff88';
        ctx.fillText(`Y: ${y}`, startX, startY + 20);
        ctx.fillStyle = '#4488ff';
        ctx.fillText(`Z: ${z}`, startX, startY + 40);
    }
}

// ============================================
// Inverse Kinematics
// ============================================
function calculateServoAngles(x, y, z) {
    // Simple 2D IK for a 2-link arm in the XZ plane
    // Base rotation is determined by X and Y

    // Servo 1: Base rotation (controls XY direction)
    let baseAngle = Math.atan2(y, x) * 180 / Math.PI + 90;
    baseAngle = clamp(baseAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);

    // Distance from base in XY plane
    const r = Math.sqrt(x * x + y * y);

    // Height adjustment
    const h = z;

    // Distance to target point
    const d = Math.sqrt(r * r + h * h);

    const L1 = CONFIG.ARM_LENGTH_1;
    const L2 = CONFIG.ARM_LENGTH_2;

    // Check if target is reachable
    if (d > L1 + L2) {
        // Target too far, extend arm fully
        const angle = Math.atan2(h, r) * 180 / Math.PI;
        return {
            servo1: Math.round(baseAngle),
            servo2: Math.round(90 - angle),
            servo3: 90
        };
    }

    if (d < Math.abs(L1 - L2)) {
        // Target too close
        return {
            servo1: Math.round(baseAngle),
            servo2: 90,
            servo3: 180
        };
    }

    // Calculate angles using law of cosines
    const cosAngle2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const angle2 = Math.acos(clamp(cosAngle2, -1, 1));

    const cosAngle1 = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
    const angle1 = Math.acos(clamp(cosAngle1, -1, 1));

    const baseElevation = Math.atan2(h, r);

    // Convert to servo angles
    let shoulderAngle = 90 - (baseElevation + angle1) * 180 / Math.PI;
    let elbowAngle = 180 - angle2 * 180 / Math.PI;

    shoulderAngle = clamp(shoulderAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);
    elbowAngle = clamp(elbowAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);

    return {
        servo1: Math.round(baseAngle),
        servo2: Math.round(shoulderAngle),
        servo3: Math.round(elbowAngle)
    };
}

function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
}

// ============================================
// WebSocket Connection
// ============================================
function connectWebSocket() {
    const ip = elements.espIP.value;
    const port = elements.wsPort.value;
    const wsUrl = `ws://${ip}:${port}`;

    console.log(`Connecting to ${wsUrl}...`);

    try {
        state.ws = new WebSocket(wsUrl);

        state.ws.onopen = () => {
            console.log('WebSocket connected!');
            state.isConnected = true;
            updateConnectionStatus(true);
            showNotification('Connected to ESP32!', 'success');
        };

        state.ws.onclose = () => {
            console.log('WebSocket disconnected');
            state.isConnected = false;
            updateConnectionStatus(false);

            // Auto-reconnect
            setTimeout(() => {
                if (!state.isConnected) {
                    console.log('Attempting to reconnect...');
                }
            }, CONFIG.WS_RECONNECT_INTERVAL);
        };

        state.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            showNotification('Connection failed!', 'error');
        };

        state.ws.onmessage = (event) => {
            console.log('Received:', event.data);
            handleMessage(event.data);
        };

    } catch (error) {
        console.error('Failed to create WebSocket:', error);
        showNotification('Failed to connect!', 'error');
    }
}

function disconnectWebSocket() {
    if (state.ws) {
        state.ws.close();
        state.ws = null;
    }
    state.isConnected = false;
    updateConnectionStatus(false);
}

function sendCommand(command) {
    if (state.isConnected && state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify(command));
        console.log('Sent:', command);
    } else {
        console.log('Not connected. Command:', command);
    }
}

function handleMessage(data) {
    try {
        const msg = JSON.parse(data);
        if (msg.type === 'status') {
            // Handle status updates from ESP32
            console.log('ESP32 Status:', msg);
        }
    } catch (e) {
        console.log('Raw message:', data);
    }
}

// ============================================
// UI Updates
// ============================================
function updateConnectionStatus(connected) {
    const statusEl = elements.connectionStatus;
    const btnConnect = elements.btnConnect;

    if (connected) {
        statusEl.classList.add('connected');
        statusEl.querySelector('.status-text').textContent = 'Connected';
        btnConnect.innerHTML = '<span class="btn-icon">✓</span><span>DISCONNECT</span>';
        btnConnect.classList.add('connected');
    } else {
        statusEl.classList.remove('connected');
        statusEl.querySelector('.status-text').textContent = 'Disconnected';
        btnConnect.innerHTML = '<span class="btn-icon">🔌</span><span>CONNECT</span>';
        btnConnect.classList.remove('connected');
    }
}

function updateServoDisplay(servo1, servo2, servo3) {
    // Update text values (without degree symbol, it's in separate span now)
    elements.servo1Value.textContent = servo1;
    elements.servo2Value.textContent = servo2;
    elements.servo3Value.textContent = servo3;

    // Update circular gauge fills
    // Circle circumference = 2 * PI * r = 2 * 3.14159 * 50 ≈ 314
    const circumference = 314;

    // Calculate stroke-dashoffset: full circle at 0°, empty at 180°
    // dashoffset = circumference - (angle/180 * circumference)
    const offset1 = circumference - (servo1 / 180) * circumference;
    const offset2 = circumference - (servo2 / 180) * circumference;
    const offset3 = circumference - (servo3 / 180) * circumference;

    elements.servo1Gauge.style.strokeDashoffset = offset1;
    elements.servo2Gauge.style.strokeDashoffset = offset2;
    elements.servo3Gauge.style.strokeDashoffset = offset3;
}

function updatePositionDisplay(x, y, z) {
    elements.xValue.textContent = x;
    elements.yValue.textContent = y;
    elements.zValue.textContent = z;

    elements.currentX.textContent = x;
    elements.currentY.textContent = y;
    elements.currentZ.textContent = z;
}

function showNotification(message, type = 'info') {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification ${type}`;
    notification.textContent = message;
    notification.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        padding: 15px 25px;
        border-radius: 10px;
        font-family: 'Orbitron', sans-serif;
        font-size: 14px;
        z-index: 1000;
        animation: slideInRight 0.3s ease;
        ${type === 'success' ? 'background: linear-gradient(135deg, #059669, #047857); color: white;' : ''}
        ${type === 'error' ? 'background: linear-gradient(135deg, #dc2626, #b91c1c); color: white;' : ''}
        ${type === 'info' ? 'background: linear-gradient(135deg, #00f0ff, #ff00aa); color: #0a0a0f;' : ''}
    `;

    document.body.appendChild(notification);

    setTimeout(() => {
        notification.style.animation = 'slideOutRight 0.3s ease';
        setTimeout(() => notification.remove(), 300);
    }, 3000);
}

// ============================================
// Control Updates
// ============================================
function updatePosition() {
    const now = Date.now();
    if (now - state.lastUpdate < CONFIG.UPDATE_THROTTLE) return;
    state.lastUpdate = now;

    // Get current values
    state.x = parseInt(elements.xSlider.value);
    state.y = parseInt(elements.ySlider.value);
    state.z = parseInt(elements.zSlider.value);

    // Calculate servo angles
    const angles = calculateServoAngles(state.x, state.y, state.z);
    state.servo1 = angles.servo1;
    state.servo2 = angles.servo2;
    state.servo3 = angles.servo3;

    // Update displays
    updatePositionDisplay(state.x, state.y, state.z);
    updateServoDisplay(state.servo1, state.servo2, state.servo3);

    // Update visualization
    visualizer.draw(state.x, state.y, state.z, state.servo1, state.servo2, state.servo3);

    // Send to ESP32
    sendCommand({
        type: 'move',
        x: state.x,
        y: state.y,
        z: state.z,
        s1: state.servo1,
        s2: state.servo2,
        s3: state.servo3
    });
}

function setAxisValue(axis, value) {
    const slider = elements[`${axis}Slider`];
    const input = elements[`${axis}Input`];

    value = clamp(value, parseInt(slider.min), parseInt(slider.max));

    slider.value = value;
    input.value = value;

    updatePosition();
}

// ============================================
// Event Listeners
// ============================================
function initEventListeners() {
    // View selector buttons
    document.querySelectorAll('.view-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            // Remove active from all
            document.querySelectorAll('.view-btn').forEach(b => b.classList.remove('active'));
            // Add active to clicked
            btn.classList.add('active');
            // Change view
            const view = btn.dataset.view;
            visualizer.setView(view);
            updatePosition();
        });
    });

    // Slider events
    ['x', 'y', 'z'].forEach(axis => {
        elements[`${axis}Slider`].addEventListener('input', (e) => {
            elements[`${axis}Input`].value = e.target.value;
            updatePosition();
        });

        elements[`${axis}Input`].addEventListener('change', (e) => {
            setAxisValue(axis, parseInt(e.target.value) || 0);
        });
    });

    // Adjust buttons
    document.querySelectorAll('.btn-adjust').forEach(btn => {
        btn.addEventListener('click', () => {
            const axis = btn.dataset.axis;
            const dir = parseInt(btn.dataset.dir);
            const currentValue = parseInt(elements[`${axis}Slider`].value);
            setAxisValue(axis, currentValue + dir * 5);
        });

        // Long press support
        let intervalId;
        btn.addEventListener('mousedown', () => {
            intervalId = setInterval(() => {
                const axis = btn.dataset.axis;
                const dir = parseInt(btn.dataset.dir);
                const currentValue = parseInt(elements[`${axis}Slider`].value);
                setAxisValue(axis, currentValue + dir * 2);
            }, 100);
        });

        btn.addEventListener('mouseup', () => clearInterval(intervalId));
        btn.addEventListener('mouseleave', () => clearInterval(intervalId));
    });

    // Connect button
    elements.btnConnect.addEventListener('click', () => {
        if (state.isConnected) {
            disconnectWebSocket();
        } else {
            connectWebSocket();
        }
    });

    // Action buttons
    elements.btnHome.addEventListener('click', () => {
        setAxisValue('x', 0);
        setAxisValue('y', 0);
        setAxisValue('z', 50);
        sendCommand({ type: 'home' });
        showNotification('Moving to HOME position', 'info');
    });

    elements.btnGrab.addEventListener('click', () => {
        sendCommand({ type: 'grab' });
        showNotification('Gripper CLOSED', 'info');
    });

    elements.btnRelease.addEventListener('click', () => {
        sendCommand({ type: 'release' });
        showNotification('Gripper OPENED', 'info');
    });

    elements.btnStop.addEventListener('click', () => {
        sendCommand({ type: 'stop' });
        showNotification('EMERGENCY STOP!', 'error');
    });
}

// ============================================
// Particle Effect
// ============================================
function createParticles() {
    const container = document.getElementById('particles');
    const particleCount = 20;

    for (let i = 0; i < particleCount; i++) {
        const particle = document.createElement('div');
        particle.className = 'particle';
        particle.style.left = `${Math.random() * 100}%`;
        particle.style.animationDelay = `${Math.random() * 15}s`;
        particle.style.animationDuration = `${15 + Math.random() * 10}s`;
        container.appendChild(particle);
    }
}

// ============================================
// Initialization
// ============================================
let visualizer;

function init() {
    console.log('🤖 Robot Arm Control System initialized');

    // Create visualizer
    visualizer = new ArmVisualizer(elements.armCanvas);

    // Initialize event listeners
    initEventListeners();

    // Create particle effects
    createParticles();

    // Initial position update
    setAxisValue('z', 50);
    updatePosition();

    // Add CSS for notifications
    const style = document.createElement('style');
    style.textContent = `
        @keyframes slideInRight {
            from { transform: translateX(100px); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }
        @keyframes slideOutRight {
            from { transform: translateX(0); opacity: 1; }
            to { transform: translateX(100px); opacity: 0; }
        }
    `;
    document.head.appendChild(style);
}

// ============================================
// Object Detection Functions
// ============================================
function initObjectDetection() {
    const btnStartCamera = document.getElementById('btnStartCamera');
    const btnStopCamera = document.getElementById('btnStopCamera');
    const btnDetect = document.getElementById('btnDetect');
    const btnPickPlace = document.getElementById('btnPickPlace');

    if (btnStartCamera) {
        btnStartCamera.addEventListener('click', startCamera);
    }
    if (btnStopCamera) {
        btnStopCamera.addEventListener('click', stopCamera);
    }
    if (btnDetect) {
        btnDetect.addEventListener('click', detectObjects);
    }
    if (btnPickPlace) {
        btnPickPlace.addEventListener('click', executePickPlace);
    }

    // Check if detection server is available
    checkDetectionServer();
}

async function checkDetectionServer() {
    try {
        const response = await fetch(`${CONFIG.DETECTION_SERVER}/`);
        const data = await response.json();
        console.log('🎯 Detection server:', data);
        if (data.yolo_available) {
            showNotification('Detection server online!', 'success');
        }
    } catch (error) {
        console.log('Detection server not available');
    }
}

function startCamera() {
    const cameraFeed = document.getElementById('cameraFeed');
    const cameraOverlay = document.getElementById('cameraOverlay');
    const cameraStatus = document.getElementById('cameraStatus');
    const btnStartCamera = document.getElementById('btnStartCamera');
    const btnStopCamera = document.getElementById('btnStopCamera');
    const btnDetect = document.getElementById('btnDetect');

    // Set video source to MJPEG stream
    cameraFeed.src = `${CONFIG.DETECTION_SERVER}/video_feed`;
    cameraFeed.classList.add('active');
    cameraOverlay.classList.add('hidden');

    // Update status
    const statusDot = cameraStatus.querySelector('.status-dot');
    const statusText = cameraStatus.querySelector('.status-text');
    statusDot.classList.remove('offline');
    statusDot.classList.add('online');
    statusText.textContent = 'Live';

    // Update buttons
    btnStartCamera.disabled = true;
    btnStopCamera.disabled = false;
    btnDetect.disabled = false;

    detectionState.isCameraActive = true;
    showNotification('Camera started', 'info');
}

function stopCamera() {
    const cameraFeed = document.getElementById('cameraFeed');
    const cameraOverlay = document.getElementById('cameraOverlay');
    const cameraStatus = document.getElementById('cameraStatus');
    const btnStartCamera = document.getElementById('btnStartCamera');
    const btnStopCamera = document.getElementById('btnStopCamera');
    const btnDetect = document.getElementById('btnDetect');

    // Stop video
    cameraFeed.src = '';
    cameraFeed.classList.remove('active');
    cameraOverlay.classList.remove('hidden');
    cameraOverlay.innerHTML = '<span>Camera stopped</span>';

    // Update status
    const statusDot = cameraStatus.querySelector('.status-dot');
    const statusText = cameraStatus.querySelector('.status-text');
    statusDot.classList.remove('online');
    statusDot.classList.add('offline');
    statusText.textContent = 'Offline';

    // Update buttons
    btnStartCamera.disabled = false;
    btnStopCamera.disabled = true;
    btnDetect.disabled = true;

    detectionState.isCameraActive = false;
    showNotification('Camera stopped', 'info');
}

async function detectObjects() {
    try {
        const response = await fetch(`${CONFIG.DETECTION_SERVER}/detect`);
        const data = await response.json();

        if (data.success) {
            detectionState.detectedObjects = data.objects;
            renderDetectedObjects(data.objects);

            if (data.count > 0) {
                showNotification(`Detected ${data.count} object(s)`, 'success');
                document.getElementById('pickPlaceControls').style.display = 'block';
            } else {
                showNotification('No objects detected', 'info');
            }
        }
    } catch (error) {
        console.error('Detection error:', error);
        showNotification('Detection failed - is server running?', 'error');
    }
}

function renderDetectedObjects(objects) {
    const objectsList = document.getElementById('objectsList');

    if (objects.length === 0) {
        objectsList.innerHTML = '<p class="no-objects">No objects detected</p>';
        return;
    }

    objectsList.innerHTML = objects.map((obj, index) => `
        <div class="object-item" data-id="${obj.id}" onclick="selectObject(${obj.id})">
            <div class="object-info">
                <span class="object-name">${obj.class}</span>
                <span class="object-coords">X: ${obj.robot_coords.x}, Y: ${obj.robot_coords.y}</span>
            </div>
            <span class="object-confidence">${(obj.confidence * 100).toFixed(0)}%</span>
        </div>
    `).join('');
}

function selectObject(objectId) {
    // Deselect all
    document.querySelectorAll('.object-item').forEach(item => {
        item.classList.remove('selected');
    });

    // Select clicked
    const selectedItem = document.querySelector(`.object-item[data-id="${objectId}"]`);
    if (selectedItem) {
        selectedItem.classList.add('selected');
    }

    detectionState.selectedObjectId = objectId;
    document.getElementById('btnPickPlace').disabled = false;
}

async function executePickPlace() {
    if (detectionState.selectedObjectId === null) {
        showNotification('Please select an object first', 'error');
        return;
    }

    if (detectionState.isExecutingSequence) {
        showNotification('Sequence already running', 'error');
        return;
    }

    detectionState.isExecutingSequence = true;
    showNotification('Starting pick & place sequence...', 'info');

    try {
        // Get pick sequence
        const pickResponse = await fetch(
            `${CONFIG.DETECTION_SERVER}/pick_sequence/${detectionState.selectedObjectId}`
        );
        const pickData = await pickResponse.json();

        // Get drop target
        const dropX = parseInt(document.getElementById('dropX').value) || 50;
        const dropY = parseInt(document.getElementById('dropY').value) || 0;
        const dropZ = parseInt(document.getElementById('dropZ').value) || 10;

        // Get place sequence
        const placeResponse = await fetch(`${CONFIG.DETECTION_SERVER}/place_sequence`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x: dropX, y: dropY, z: dropZ })
        });
        const placeData = await placeResponse.json();

        // Combine sequences
        const fullSequence = [...pickData.sequence, ...placeData.sequence];

        // Execute sequence
        await executeSequence(fullSequence);

        showNotification('Pick & Place complete!', 'success');

    } catch (error) {
        console.error('Pick & Place error:', error);
        showNotification('Pick & Place failed', 'error');
    }

    detectionState.isExecutingSequence = false;
}

async function executeSequence(sequence) {
    for (const step of sequence) {
        showNotification(step.desc, 'info');

        switch (step.action) {
            case 'move':
                setAxisValue('x', step.x);
                setAxisValue('y', step.y);
                setAxisValue('z', step.z);
                await delay(1000);
                break;
            case 'grab':
                sendCommand({ type: 'grab' });
                await delay(500);
                break;
            case 'release':
                sendCommand({ type: 'release' });
                await delay(500);
                break;
            case 'home':
                setAxisValue('x', 0);
                setAxisValue('y', 0);
                setAxisValue('z', 50);
                sendCommand({ type: 'home' });
                await delay(1000);
                break;
        }
    }
}

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    init();
    initObjectDetection();
});
