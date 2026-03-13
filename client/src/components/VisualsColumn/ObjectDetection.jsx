import React from 'react';

function ObjectDetection({
    cameraActive, onStartCamera, onStopCamera, onDetect, availableCameras,
    selectedCamera, setSelectedCamera, availableModels, selectedModel,
    onModelSwitch, isModelSwitching, detectedObjects, onExecutePickPlace
}) {
    const [selectedObjId, setSelectedObjId] = React.useState(null);

    const handleExecute = () => {
        const dropX = parseInt(document.getElementById('dropX').value) || 100;
        const dropY = parseInt(document.getElementById('dropY').value) || 100;
        const dropZ = parseInt(document.getElementById('dropZ').value) || 20;
        if (selectedObjId !== null) {
            onExecutePickPlace(selectedObjId, dropX, dropY, dropZ);
        } else {
            alert('Please select an object first');
        }
    };

    return (
        <section className="camera-panel glass-panel">
            <div className="viz-header">
                <h2><span className="icon">📷</span> Object Detection</h2>
                <div className="camera-status" id="cameraStatus">
                    <span className={`status-dot ${cameraActive ? 'online' : 'offline'}`}></span>
                    <span className="status-text">{cameraActive ? 'Online' : 'Offline'}</span>
                </div>
            </div>

            <div className="camera-view-container">
                {cameraActive ? (
                    <img id="cameraFeed" src="http://localhost:5000/video_feed" className="camera-feed active" alt="Camera Feed" />
                ) : (
                    <div className="camera-overlay" id="cameraOverlay">
                        <span>Click "Start Camera" to begin</span>
                    </div>
                )}
            </div>

            <div className="camera-controls">
                <div className="camera-select-container">
                    <select
                        id="cameraSelect"
                        className="camera-select"
                        value={selectedCamera}
                        onChange={(e) => setSelectedCamera(e.target.value)}
                        disabled={cameraActive}
                    >
                        {availableCameras.length === 0 && <option value="">Loading cameras...</option>}
                        {availableCameras.map(cam => (
                            <option key={cam.index} value={cam.index}>{cam.name}</option>
                        ))}
                    </select>
                </div>
                <div className="camera-select-container">
                    <select
                        id="modelSelect"
                        className="camera-select"
                        value={selectedModel || ''}
                        onChange={(e) => onModelSwitch(e.target.value)}
                        disabled={isModelSwitching}
                    >
                        {availableModels?.length === 0 && <option value="">Loading models...</option>}
                        {availableModels?.map(model => (
                            <option key={model.id} value={model.id}>
                                {model.name}
                            </option>
                        ))}
                    </select>
                </div>
                {!cameraActive ? (
                    <button className="btn-camera" id="btnStartCamera" onClick={onStartCamera}>
                        <span className="btn-icon">▶️</span>
                        <span>Start Camera</span>
                    </button>
                ) : (
                    <button className="btn-camera" id="btnStopCamera" onClick={onStopCamera}>
                        <span className="btn-icon">⏹️</span>
                        <span>Stop</span>
                    </button>
                )}
                <button className="btn-camera btn-detect" id="btnDetect" onClick={onDetect} disabled={!cameraActive}>
                    <span className="btn-icon">🔍</span>
                    <span>Detect</span>
                </button>
            </div>

            <div className="detected-objects" id="detectedObjects">
                <h3>Detected Objects</h3>
                <div className="objects-list" id="objectsList">
                    {detectedObjects.length === 0 ? (
                        <p className="no-objects">No objects detected</p>
                    ) : (
                        detectedObjects.map((obj, i) => (
                            <div 
                                key={i} 
                                className={`object-item ${selectedObjId === obj.id ? 'selected' : ''}`}
                                onClick={() => setSelectedObjId(obj.id)}
                            >
                                <div className="object-info">
                                    <span className="object-name">{obj.class}</span>
                                    <span className="object-coords">X:{obj.robot_coords.x} Y:{obj.robot_coords.y}</span>
                                </div>
                                <span className="object-confidence">{(obj.confidence * 100).toFixed(0)}%</span>
                            </div>
                        ))
                    )}
                </div>
            </div>

            {detectedObjects.length > 0 && (
                <div className="pick-place-controls" id="pickPlaceControls">
                    <h3>Pick & Place</h3>
                    <div className="target-position">
                        <label>Drop Target:</label>
                        <div className="target-inputs">
                            <input type="number" id="dropX" defaultValue="100" placeholder="X" />
                            <input type="number" id="dropY" defaultValue="100" placeholder="Y" />
                            <input type="number" id="dropZ" defaultValue="20" placeholder="Z" />
                        </div>
                    </div>
                    <button 
                        className="btn-action btn-pick" 
                        id="btnPickPlace" 
                        onClick={handleExecute}
                        disabled={selectedObjId === null}
                    >
                        <span className="btn-icon">🤖</span>
                        <span className="btn-text">Execute Pick & Place</span>
                    </button>
                </div>
            )}
        </section>
    );
}

export default ObjectDetection;
