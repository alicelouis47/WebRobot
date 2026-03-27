export const CONFIG = {
    BASE_HEIGHT: 56,
    SHOULDER_OFFSET: 42.6,
    ARM_LENGTH_1: 120,
    ARM_LENGTH_2: 116.25,
    GRIPPER_OFFSET_L: 35.0, // Perpendicular offset
    GRIPPER_OFFSET_A: 5.0,  // Axis offset
    SERVO_MIN: 0,
    SERVO_MAX: 180,
    DEFAULT_X: 0,
    DEFAULT_Y: 0,
    DEFAULT_Z: 50,
};

function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
}

export function calculateServoAngles(x, y, z) {
    let baseAngle = Math.atan2(y, x) * 180 / Math.PI + 90;
    baseAngle = clamp(baseAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);

    const r = Math.sqrt(x * x + y * y);
    const h = z - CONFIG.BASE_HEIGHT - CONFIG.SHOULDER_OFFSET;
    const d = Math.sqrt(r * r + h * h);

    const L1 = CONFIG.ARM_LENGTH_1;
    const L2 = CONFIG.ARM_LENGTH_2;

    if (d > L1 + L2) {
        const angle = Math.atan2(h, r) * 180 / Math.PI;
        return [
            Math.round(baseAngle),
            Math.round(90 - angle),
            90,
            90,
            90 // Gripper
        ];
    }

    if (d < Math.abs(L1 - L2)) {
        return [
            Math.round(baseAngle),
            90,
            180,
            90,
            90 // Gripper
        ];
    }

    const cosAngle2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const angle2 = Math.acos(clamp(cosAngle2, -1, 1));

    const cosAngle1 = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
    const angle1 = Math.acos(clamp(cosAngle1, -1, 1));

    const baseElevation = Math.atan2(h, r);

    let shoulderAngle = 90 - (baseElevation + angle1) * 180 / Math.PI;
    let elbowAngle = 180 - angle2 * 180 / Math.PI;

    let wristAngle = 90 - (shoulderAngle - 90) - (elbowAngle - 90);
    wristAngle = clamp(wristAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);

    shoulderAngle = clamp(shoulderAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);
    elbowAngle = clamp(elbowAngle, CONFIG.SERVO_MIN, CONFIG.SERVO_MAX);

    return [
        Math.round(baseAngle),
        Math.round(shoulderAngle),
        Math.round(elbowAngle),
        Math.round(wristAngle),
        90 // Gripper (mock)
    ];
}
