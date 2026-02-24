import React, { useState, useEffect } from 'react';
import './index.css';
import Header from './components/Header';
import ArmVisualization from './components/VisualsColumn/ArmVisualization';
import ObjectDetection from './components/VisualsColumn/ObjectDetection';
import ArucoCalibration from './components/VisualsColumn/ArucoCalibration';
import XYZControls from './components/ControlsColumn/XYZControls';
import QuickActions from './components/ControlsColumn/QuickActions';
import ServoMonitor from './components/ControlsColumn/ServoMonitor';
import ConnectionSettings from './components/ControlsColumn/ConnectionSettings';
import { calculateServoAngles } from './utils/kinematics';

function App() {
  const [connected, setConnected] = useState(false);
  const [viewMode, setViewMode] = useState('side');

  const [position, setPosition] = useState({ x: 0, y: 0, z: 50 });
  const [angles, setAngles] = useState([90, 90, 90, 90, 90]);

  const [cameraActive, setCameraActive] = useState(false);
  const [availableCameras, setAvailableCameras] = useState([]);
  const [selectedCamera, setSelectedCamera] = useState('');
  const [detectedObjects, setDetectedObjects] = useState([]);
  const [arucoStatus, setArucoStatus] = useState({ calibrated: false, markers_detected: 0 });

  const [ip, setIp] = useState('192.168.4.1');
  const [port, setPort] = useState('81');

  // Update servo angles whenever position changes
  useEffect(() => {
    const newAngles = calculateServoAngles(position.x, position.y, position.z);
    setAngles(newAngles);
  }, [position]);

  // We can add WebSocket logic here or in a hook later. 
  // For now, these are dummy handlers for the UI.

  const handlePositionChange = (axis, value) => {
    setPosition(prev => ({ ...prev, [axis]: value }));
  };

  const handleAction = (action) => {
    console.log(`Action: ${action}`);
  };

  const handleConnect = () => {
    setConnected(!connected);
  };

  return (
    <>
      <div className="background-effects">
        <div className="grid-lines"></div>
        <div className="floating-particles" id="particles"></div>
      </div>

      <div className="container">
        <Header connected={connected} />

        <main className="main-content">
          <div className="visuals-column">
            <ArmVisualization
              viewMode={viewMode}
              setViewMode={setViewMode}
              position={position}
            />
            <ObjectDetection
              cameraActive={cameraActive}
              onStartCamera={() => setCameraActive(true)}
              onStopCamera={() => setCameraActive(false)}
              onDetect={() => { }}
              availableCameras={availableCameras}
              selectedCamera={selectedCamera}
              setSelectedCamera={setSelectedCamera}
              detectedObjects={detectedObjects}
              onExecutePickPlace={() => { }}
            />
            <ArucoCalibration
              arucoStatus={arucoStatus}
              onCalibrate={() => { }}
            />
          </div>

          <div className="controls-column">
            <div className="control-row">
              <XYZControls
                position={position}
                onPositionChange={handlePositionChange}
              />
              <QuickActions onAction={handleAction} />
            </div>

            <ServoMonitor angles={angles} />

            <ConnectionSettings
              ip={ip} setIp={setIp}
              port={port} setPort={setPort}
              connected={connected}
              onConnect={handleConnect}
            />
          </div>
        </main>

        <footer className="footer">
          <p>🤖 ESP32 Robot Arm Controller | WebSocket Control System</p>
        </footer>
      </div>
    </>
  );
}

export default App;
