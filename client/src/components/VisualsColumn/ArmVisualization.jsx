import React, { useRef, useEffect } from 'react';

function ArmVisualization({ viewMode, setViewMode, position }) {
    const canvasRef = useRef(null);

    // We can port canvas drawing logic here later inside useEffect

    return (
        <section className="visualization-panel glass-panel">
            <div className="viz-header">
                <h2><span className="icon">🎯</span> Arm Visualization</h2>
                <div className="view-selector">
                    {['side', 'top', 'front', '3d'].map((view) => (
                        <button
                            key={view}
                            className={`view-btn ${viewMode === view ? 'active' : ''}`}
                            onClick={() => setViewMode(view)}
                            title={`${view.toUpperCase()} View`}
                        >
                            <span>{view.toUpperCase()}</span>
                        </button>
                    ))}
                </div>
            </div>
            <div className="canvas-container">
                <canvas ref={canvasRef} id="armCanvas" width="400" height="300"></canvas>
            </div>
            <div className="current-position">
                <div className="pos-item">
                    <span className="label">X:</span>
                    <span className="value" id="currentX">{position.x}</span>
                </div>
                <div className="pos-item">
                    <span className="label">Y:</span>
                    <span className="value" id="currentY">{position.y}</span>
                </div>
                <div className="pos-item">
                    <span className="label">Z:</span>
                    <span className="value" id="currentZ">{position.z}</span>
                </div>
            </div>
        </section>
    );
}

export default ArmVisualization;
