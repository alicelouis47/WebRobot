import React from 'react';

function ConnectionSettings({ availablePorts, selectedPort, setSelectedPort, connected, onConnect }) {
    return (
        <section className="settings-panel glass-panel">
            <h2><span className="icon">📡</span> Connection</h2>
            <div className="connection-form">
                <div className="input-group">
                    <label htmlFor="serialPort">COM Port</label>
                    <select
                        id="serialPort"
                        value={selectedPort}
                        onChange={(e) => setSelectedPort(e.target.value)}
                        disabled={connected}
                        style={{ width: '100%', padding: '0.5rem', background: 'rgba(255, 255, 255, 0.1)', color: 'white', border: '1px solid rgba(255, 255, 255, 0.2)', borderRadius: '4px' }}
                    >
                        <option value="" style={{ color: 'black' }}>Select a port...</option>
                        {availablePorts.map((port) => (
                            <option key={port} value={port} style={{ color: 'black' }}>
                                {port}
                            </option>
                        ))}
                    </select>
                </div>
                <button 
                    className={`btn-connect ${connected ? 'connected' : ''}`} 
                    id="btnConnect" 
                    onClick={onConnect}
                    disabled={!selectedPort && !connected}
                >
                    <span className="btn-icon">🔌</span>
                    <span>{connected ? 'DISCONNECT' : 'CONNECT'}</span>
                </button>
            </div>
            <div className="connection-info">
                <p>💡 เลือกพอร์ต <strong>COM</strong> ที่เชื่อมต่อกับ Arduino</p>
                <p>⚡ บอดเรตถูกล็อกไว้ที่ <strong>115200</strong></p>
            </div>
        </section>
    );
}

export default ConnectionSettings;
