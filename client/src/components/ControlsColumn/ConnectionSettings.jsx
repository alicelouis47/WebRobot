import React, { useState } from 'react';

function ConnectionSettings({ ip, setIp, port, setPort, connected, onConnect }) {
    const [showPassword, setShowPassword] = useState(false);

    return (
        <section className="settings-panel glass-panel">
            <h2><span className="icon">📡</span> Connection</h2>
            <div className="connection-form">
                <div className="input-group">
                    <label htmlFor="espIP">ESP32 IP Address</label>
                    <input
                        type="text"
                        id="espIP"
                        value={ip}
                        onChange={(e) => setIp(e.target.value)}
                        placeholder="192.168.4.1"
                    />
                </div>
                <div className="input-group">
                    <label htmlFor="wsPort">WebSocket Port</label>
                    <input
                        type="number"
                        id="wsPort"
                        value={port}
                        onChange={(e) => setPort(e.target.value)}
                        placeholder="81"
                    />
                </div>
                <button className="btn-connect" id="btnConnect" onClick={onConnect}>
                    <span className="btn-icon">🔌</span>
                    <span>{connected ? 'DISCONNECT' : 'CONNECT'}</span>
                </button>
            </div>
            <div className="connection-info">
                <p>💡 เชื่อมต่อ WiFi ไปที่ <strong>RobotArm_AP</strong></p>
                <p className="wifi-password-row">
                    🔑 Password: <strong className={showPassword ? "" : "password-masked"}>
                        {showPassword ? "12345678" : "••••••••"}
                    </strong>
                    <button
                        type="button"
                        className="btn-toggle-password"
                        onClick={() => setShowPassword(!showPassword)}
                        title="Show/Hide Password"
                    >
                        {showPassword ? (
                            <svg className="eye-icon eye-open" viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                                <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                                <circle cx="12" cy="12" r="3" />
                            </svg>
                        ) : (
                            <svg className="eye-icon eye-closed" viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                                <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24" />
                                <line x1="1" y1="1" x2="23" y2="23" />
                            </svg>
                        )}
                    </button>
                </p>
            </div>
        </section>
    );
}

export default ConnectionSettings;
