import { CONFIG as KINEMATICS_CONFIG } from '../../utils/kinematics';

// Merge Arm Dimensions into CONFIG for visualizer
export const CONFIG = {
    ...KINEMATICS_CONFIG,
    DETECTION_SERVER: 'http://localhost:5000'
};

export class ArmVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.currentView = 'side'; // side, top, front, 3d
        this.scale = 1.0;
        this.resize();

        // Zoom with mouse wheel
        this.handleWheel = (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
            this.scale = Math.max(0.3, Math.min(5, this.scale * zoomFactor));
            if (this.lastState) {
                this.draw(...this.lastState);
            }
        };
        this.canvas.addEventListener('wheel', this.handleWheel, { passive: false });
    }

    destroy() {
        this.canvas.removeEventListener('wheel', this.handleWheel);
    }

    setView(view) {
        this.currentView = view;
        if (this.lastState) {
            this.draw(...this.lastState);
        }
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = 300;
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
        if (this.lastState) {
            this.draw(...this.lastState);
        }
    }

    draw(x, y, z, servo1, servo2, servo3, servo4) {
        this.lastState = [x, y, z, servo1, servo2, servo3, servo4];
        const ctx = this.ctx;
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        this.drawGrid();

        const armPos = this.calculateArmPositions(x, y, z, servo1, servo2, servo3, servo4);

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

        this.drawPositionIndicator(x, y, z);
    }

    calculateArmPositions(x, y, z, servo1, servo2, servo3, servo4) {
        const baseAngle = (servo1 - 90) * Math.PI / 180;
        const shoulderAngle = (90 - servo2) * Math.PI / 180;
        const elbowAngle = (90 - servo3) * Math.PI / 180;

        const L1 = CONFIG.ARM_LENGTH_1;
        const L2 = CONFIG.ARM_LENGTH_2;

        const base = { x: 0, y: 0, z: CONFIG.BASE_HEIGHT };

        const elbow = {
            x: Math.cos(baseAngle) * Math.cos(shoulderAngle) * L1,
            y: Math.sin(baseAngle) * Math.cos(shoulderAngle) * L1,
            z: base.z + Math.sin(shoulderAngle) * L1
        };

        const totalAngle = shoulderAngle + elbowAngle - Math.PI / 2;
        const end = {
            x: elbow.x + Math.cos(baseAngle) * Math.cos(totalAngle) * L2,
            y: elbow.y + Math.sin(baseAngle) * Math.cos(totalAngle) * L2,
            z: elbow.z + Math.sin(totalAngle) * L2
        };

        return { base, elbow, end };
    }

    drawSideView(armPos, x, y, z) {
        const baseY = this.canvas.height - 60;
        this.drawViewLabel('SIDE VIEW (XZ Plane)');
        this.drawAxes('X', 'Z');

        const base = { x: this.centerX, y: baseY };
        const elbow = {
            x: this.centerX + armPos.elbow.x * this.scale,
            y: baseY - armPos.elbow.z * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.x * this.scale,
            y: baseY - armPos.end.z * this.scale
        };

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, elbow.x, elbow.y, '#00f0ff', 12);
        this.drawArmSegment(elbow.x, elbow.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(elbow.x, elbow.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    drawTopView(armPos, x, y, z) {
        this.drawViewLabel('TOP VIEW (XY Plane)');
        this.drawAxes('X', 'Y');

        const base = { x: this.centerX, y: this.centerY };
        const elbow = {
            x: this.centerX + armPos.elbow.x * this.scale,
            y: this.centerY - armPos.elbow.y * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.x * this.scale,
            y: this.centerY - armPos.end.y * this.scale
        };

        this.drawRangeCircle(base.x, base.y, (CONFIG.ARM_LENGTH_1 + CONFIG.ARM_LENGTH_2) * this.scale);

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, elbow.x, elbow.y, '#00f0ff', 12);
        this.drawArmSegment(elbow.x, elbow.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(elbow.x, elbow.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    drawFrontView(armPos, x, y, z) {
        const baseY = this.canvas.height - 60;
        this.drawViewLabel('FRONT VIEW (YZ Plane)');
        this.drawAxes('Y', 'Z');

        const base = { x: this.centerX, y: baseY };
        const elbow = {
            x: this.centerX + armPos.elbow.y * this.scale,
            y: baseY - armPos.elbow.z * this.scale
        };
        const end = {
            x: this.centerX + armPos.end.y * this.scale,
            y: baseY - armPos.end.z * this.scale
        };

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, elbow.x, elbow.y, '#00f0ff', 12);
        this.drawArmSegment(elbow.x, elbow.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(elbow.x, elbow.y, 12, '#ff00aa');
        this.drawEndEffector(end.x, end.y);
    }

    draw3DView(armPos, x, y, z) {
        this.drawViewLabel('3D ISOMETRIC VIEW');

        const isoAngle = Math.PI / 6;
        const scale3D = this.scale * 0.8;

        const toIso = (p) => ({
            x: this.centerX + (p.x - p.y) * Math.cos(isoAngle) * scale3D,
            y: this.centerY + 40 + (p.x + p.y) * Math.sin(isoAngle) * scale3D - p.z * scale3D
        });

        const base = toIso(armPos.base);
        const elbow = toIso(armPos.elbow);
        const end = toIso(armPos.end);

        this.draw3DAxes(toIso);
        this.drawGroundShadow(armPos, toIso);

        this.drawBaseCircle(base.x, base.y);
        this.drawArmSegment(base.x, base.y, elbow.x, elbow.y, '#00f0ff', 12);
        this.drawArmSegment(elbow.x, elbow.y, end.x, end.y, '#ff00aa', 10);
        this.drawJoint(base.x, base.y, 15, '#00f0ff');
        this.drawJoint(elbow.x, elbow.y, 12, '#ff00aa');
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

        ctx.beginPath();
        ctx.moveTo(50, baseY);
        ctx.lineTo(this.canvas.width - 50, baseY);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(this.centerX, 30);
        ctx.lineTo(this.centerX, this.canvas.height - 30);
        ctx.stroke();

        ctx.setLineDash([]);

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

        const xEnd = toIso({ x: axisLen, y: 0, z: 0 });
        ctx.strokeStyle = '#ff4466';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(xEnd.x, xEnd.y);
        ctx.stroke();
        ctx.fillStyle = '#ff4466';
        ctx.fillText('X', xEnd.x + 5, xEnd.y);

        const yEnd = toIso({ x: 0, y: axisLen, z: 0 });
        ctx.strokeStyle = '#44ff88';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(yEnd.x, yEnd.y);
        ctx.stroke();
        ctx.fillStyle = '#44ff88';
        ctx.fillText('Y', yEnd.x - 15, yEnd.y);

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
        const base = toIso({ x: 0, y: 0, z: 0 });
        const elbow = toIso({ x: armPos.elbow.x, y: armPos.elbow.y, z: 0 });
        const end = toIso({ x: armPos.end.x, y: armPos.end.y, z: 0 });

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 4;
        ctx.setLineDash([5, 5]);

        ctx.beginPath();
        ctx.moveTo(base.x, base.y);
        ctx.lineTo(elbow.x, elbow.y);
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
