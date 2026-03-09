import React from 'react';

function ArucoCalibration({ arucoStatus, onCalibrate, showGrid, onToggleGrid }) {
    const isCalibrated = arucoStatus?.calibrated;

    return (
        <div className="aruco-panel" id="arucoPanel">
            <h3><span className="icon">📍</span> ArUco Calibration</h3>
            <div className="aruco-status" id="arucoStatus">
                <div className="status-row">
                    <span className="status-label">Status:</span>
                    <span className="status-value" id="arucoCalibStatus">
                        {isCalibrated ? 'Calibrated' : 'Not Calibrated'}
                    </span>
                </div>
                <div className="status-row">
                    <span className="status-label">Markers:</span>
                    <span className="status-value" id="arucoMarkerCount">
                        {arucoStatus?.markers_detected || 0}/4
                    </span>
                </div>
            </div>
            <div className="aruco-controls" style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
                <button
                    className="btn-camera btn-aruco"
                    id="btnCalibrateAruco"
                    onClick={onCalibrate}
                // enable calibration even if not 4 markers detected if you want to retry, or disable if you prefer strict mode
                >
                    <span className="btn-icon">🎯</span>
                    <span>Calibrate ArUco</span>
                </button>
                <button
                    className={`btn-camera ${showGrid ? 'active' : ''}`}
                    onClick={onToggleGrid}
                    disabled={!isCalibrated}
                    style={{ backgroundColor: showGrid ? '#007bff' : '' }}
                >
                    <span className="btn-icon">📏</span>
                    <span>{showGrid ? 'Hide 5mm Grid' : 'Show 5mm Grid'}</span>
                </button>
            </div>
            <div className="aruco-info">
                <p>📌 Required: Markers ID 0,1,2,3 at workspace corners</p>
            </div>
        </div>
    );
}

export default ArucoCalibration;
