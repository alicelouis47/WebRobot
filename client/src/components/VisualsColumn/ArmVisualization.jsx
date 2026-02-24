import React, { useRef, useEffect, useState } from 'react';
import { ArmVisualizer as VisualizerClass } from './ArmVisualizer';

function ArmVisualization({ viewMode, setViewMode, position, angles }) {
    const canvasRef = useRef(null);
    const [visualizer, setVisualizer] = useState(null);

    useEffect(() => {
        if (canvasRef.current && !visualizer) {
            const viz = new VisualizerClass(canvasRef.current);
            setVisualizer(viz);

            const handleResize = () => viz.resize();
            window.addEventListener('resize', handleResize);
            return () => {
                window.removeEventListener('resize', handleResize);
                viz.destroy();
            };
        }
    }, [canvasRef, visualizer]);

    useEffect(() => {
        if (visualizer && angles.length >= 4) {
            visualizer.setView(viewMode);
            visualizer.draw(position.x, position.y, position.z, angles[0], angles[1], angles[2], angles[3]);
        }
    }, [visualizer, viewMode, position, angles]);

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
