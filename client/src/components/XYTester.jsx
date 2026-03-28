import React from 'react';

function XYTester({ position, onPositionChange }) {
    // Top to bottom (Forward to back)
    const xPositions = [150, 140, 130, 120, 110, 100, 90, 80, 70];
    // Left to right
    const yPositions = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

    return (
        <section className="xy-tester-panel glass-panel" style={{ padding: '20px', borderRadius: '12px', background: 'rgba(20, 20, 35, 0.6)' }}>
            <h2><span className="icon">🎛️</span> XY Grid Tester</h2>
            
            <div className="tester-layout" style={{ display: 'flex', gap: '30px', marginTop: '20px', alignItems: 'center', justifyContent: 'center' }}>
                <div className="grid-container" style={{ display: 'flex', flexDirection: 'column', gap: '5px' }}>
                    <p style={{ textAlign: 'center', marginBottom: '10px', color: '#aaa', fontSize: '14px' }}>Top-down View (Click to Move)</p>
                    
                    {xPositions.map(x => (
                        <div key={x} style={{ display: 'flex', gap: '5px', alignItems: 'center' }}>
                            <div style={{ width: '35px', textAlign: 'right', color: '#888', fontWeight: 'bold', fontSize: '12px' }}>X:{x}</div>
                            {yPositions.map(y => {
                                const isActive = position.x === x && position.y === y;
                                return (
                                    <button
                                        key={`${x}-${y}`}
                                        onClick={() => onPositionChange({ x, y })}
                                        style={{
                                            width: '35px',
                                            height: '35px',
                                            borderRadius: '6px',
                                            border: isActive ? '2px solid #00ff88' : '1px solid #444',
                                            background: isActive ? 'rgba(0, 255, 136, 0.2)' : '#222',
                                            color: isActive ? '#00ff88' : '#aaa',
                                            cursor: 'pointer',
                                            transition: 'all 0.2s',
                                            display: 'flex',
                                            flexDirection: 'column',
                                            alignItems: 'center',
                                            justifyContent: 'center',
                                            fontSize: '10px',
                                            boxShadow: isActive ? '0 0 10px rgba(0,255,136,0.3)' : 'none'
                                        }}
                                        onMouseEnter={(e) => {
                                            if (!isActive) e.currentTarget.style.background = '#333';
                                        }}
                                        onMouseLeave={(e) => {
                                            if (!isActive) e.currentTarget.style.background = '#222';
                                        }}
                                    >
                                        {isActive ? '📍' : '+' }
                                    </button>
                                );
                            })}
                        </div>
                    ))}
                    <div style={{ display: 'flex', gap: '5px', marginLeft: '40px', marginTop: '5px' }}>
                        {yPositions.map(y => (
                            <div key={`lbl-${y}`} style={{ width: '35px', textAlign: 'center', color: '#888', fontSize: '10px', fontWeight: 'bold' }}>{y}</div>
                        ))}
                    </div>
                </div>

                <div className="z-control-container" style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '0 30px', borderLeft: '1px solid rgba(255,255,255,0.1)', justifyContent: 'center' }}>
                    <p style={{ marginBottom: '20px', color: '#aaa', fontSize: '14px', whiteSpace: 'nowrap' }}>Z (Height)</p>
                    <input
                        type="range"
                        style={{
                            writingMode: 'vertical-lr',
                            WebkitAppearance: 'slider-vertical',
                            appearance: 'slider-vertical',
                            height: '250px',
                            width: '12px',
                            cursor: 'pointer',
                            accentColor: '#00ff88'
                        }}
                        min="0" max="150"
                        value={position.z}
                        onChange={(e) => onPositionChange('z', parseInt(e.target.value))}
                    />
                    <div style={{ marginTop: '20px', color: '#00ff88', fontWeight: 'bold' }}>
                        <span>Z: {position.z} mm</span>
                    </div>
                </div>
            </div>
        </section>
    );
}

export default XYTester;
