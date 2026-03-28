import React from 'react';

function XYZControls({ position, onPositionChange, offset, onOffsetChange }) {
    const handleSliderChange = (axis, value) => {
        onPositionChange(axis, parseInt(value));
    };

    const handleInputChange = (axis, value) => {
        onPositionChange(axis, parseInt(value));
    };

    const handleAdjust = (axis, amount) => {
        onPositionChange(axis, position[axis] + amount);
    };

    return (
        <section className="control-panel glass-panel">
            <h2><span className="icon">🎮</span> XYZ Coordinates</h2>

            <div className="coordinate-controls">
                {/* X Axis */}
                <div className="axis-control" data-axis="x">
                    <div className="axis-header">
                        <span className="axis-label x-axis">X (Forward/Back)</span>
                        <span className="axis-value" id="xValue">{position.x}</span>
                        <span className="axis-unit">mm</span>
                    </div>
                    <div className="slider-container">
                        <button className="btn-adjust minus" onClick={() => handleAdjust('x', -1)}>−</button>
                        <input
                            type="range"
                            className="slider x-slider"
                            min="0" max="230"
                            value={position.x}
                            onChange={(e) => handleSliderChange('x', e.target.value)}
                        />
                        <button className="btn-adjust plus" onClick={() => handleAdjust('x', 1)}>+</button>
                    </div>
                    <input
                        type="number"
                        className="axis-input"
                        value={position.x}
                        min="0" max="230"
                        onChange={(e) => handleInputChange('x', e.target.value)}
                    />
                </div>

                {/* Y Axis */}
                <div className="axis-control" data-axis="y">
                    <div className="axis-header">
                        <span className="axis-label y-axis">Y (Left/Right)</span>
                        <span className="axis-value" id="yValue">{position.y}</span>
                        <span className="axis-unit">mm</span>
                    </div>
                    <div className="slider-container">
                        <button className="btn-adjust minus" onClick={() => handleAdjust('y', -1)}>−</button>
                        <input
                            type="range"
                            className="slider y-slider"
                            min="-175" max="175"
                            value={position.y}
                            onChange={(e) => handleSliderChange('y', e.target.value)}
                        />
                        <button className="btn-adjust plus" onClick={() => handleAdjust('y', 1)}>+</button>
                    </div>
                    <input
                        type="number"
                        className="axis-input"
                        value={position.y}
                        min="-175" max="175"
                        onChange={(e) => handleInputChange('y', e.target.value)}
                    />
                </div>

                {/* Z Axis */}
                <div className="axis-control" data-axis="z">
                    <div className="axis-header">
                        <span className="axis-label z-axis">Z AXIS</span>
                        <span className="axis-value" id="zValue">{position.z}</span>
                        <span className="axis-unit">mm</span>
                    </div>
                    <div className="slider-container">
                        <button className="btn-adjust minus" onClick={() => handleAdjust('z', -1)}>−</button>
                        <input
                            type="range"
                            className="slider z-slider"
                            min="0" max="150"
                            value={position.z}
                            onChange={(e) => handleSliderChange('z', e.target.value)}
                        />
                        <button className="btn-adjust plus" onClick={() => handleAdjust('z', 1)}>+</button>
                    </div>
                    <input
                        type="number"
                        className="axis-input"
                        value={position.z}
                        min="0" max="150"
                        onChange={(e) => handleInputChange('z', e.target.value)}
                    />
                </div>

                {/* Offset Controls */}
                <div className="axis-control">
                    <div className="axis-header">
                        <span className="axis-label" style={{ color: '#fff' }}>Coordinate Offsets</span>
                    </div>
                    <div className="slider-container" style={{ gap: '10px', marginTop: '10px', display: 'flex' }}>
                        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                            <label style={{ fontSize: '10px', color: '#888', marginBottom: '4px' }}>X Offset</label>
                            <input
                                type="number"
                                className="axis-input"
                                style={{ position: 'relative', width: '100%', right: 0 }}
                                value={offset?.x || 0}
                                onChange={(e) => onOffsetChange && onOffsetChange('x', parseInt(e.target.value) || 0)}
                            />
                        </div>
                        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                            <label style={{ fontSize: '10px', color: '#888', marginBottom: '4px' }}>Y Offset</label>
                            <input
                                type="number"
                                className="axis-input"
                                style={{ position: 'relative', width: '100%', right: 0 }}
                                value={offset?.y || 0}
                                onChange={(e) => onOffsetChange && onOffsetChange('y', parseInt(e.target.value) || 0)}
                            />
                        </div>
                        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                            <label style={{ fontSize: '10px', color: '#888', marginBottom: '4px' }}>Z Offset</label>
                            <input
                                type="number"
                                className="axis-input"
                                style={{ position: 'relative', width: '100%', right: 0 }}
                                value={offset?.z || 0}
                                onChange={(e) => onOffsetChange && onOffsetChange('z', parseInt(e.target.value) || 0)}
                            />
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
}

export default XYZControls;
