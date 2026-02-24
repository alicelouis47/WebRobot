import React from 'react';

function ServoMonitor({ angles }) {
    const servos = [
        { id: '1', name: 'WAIST', j: 'J0' },
        { id: '2', name: 'SHOULDER', j: 'J1' },
        { id: '3', name: 'ELBOW', j: 'J2' },
        { id: '4', name: 'WRIST', j: 'J3' },
        { id: '5', name: 'GRIPPER', j: 'MG90s' }
    ];

    return (
        <section className="servo-panel glass-panel">
            <h2><span className="icon">⚙️</span> Servo Angles</h2>
            <div className="servo-monitors">
                {servos.map((servo, i) => {
                    const val = angles[i] || 90;
                    // Calculate stroke dasharray for the circle based on value
                    // circumference = 2 * Math.PI * 50 = 314.159
                    const maxVal = servo.id === '5' ? 180 : 180; // Gripper uses 0-180 too usually
                    const progress = (val / maxVal) * 314.159;

                    return (
                        <div key={servo.id} className="servo-item" data-servo={servo.id}>
                            <div className="servo-gauge-wrapper">
                                <svg viewBox="0 0 120 120" className="gauge-svg-circle">
                                    <circle cx="60" cy="60" r="50" className="gauge-track" />
                                    <circle
                                        cx="60" cy="60" r="50"
                                        className={`gauge-progress gauge-progress-${servo.id}`}
                                        style={{ strokeDasharray: `${progress}, 315` }}
                                    />
                                    <circle cx="60" cy="60" r="35" className={`gauge-inner gauge-inner-${servo.id}`} />
                                    <circle cx="60" cy="60" r="30" className="gauge-center" />
                                </svg>
                                <div className="servo-value-display">
                                    <span className="servo-angle">{val}</span>
                                    <span className="servo-deg">°</span>
                                </div>
                            </div>
                            <div className="servo-info">
                                <span className="servo-name">{servo.name}</span>
                                <span className="servo-id">{servo.j}</span>
                            </div>
                        </div>
                    );
                })}
            </div>
        </section>
    );
}

export default ServoMonitor;
