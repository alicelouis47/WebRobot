import React, { useState, useEffect, useRef } from 'react';
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

const DETECTION_SERVER = 'http://localhost:5000';

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

  // Polling intervals
  const detectInterval = useRef(null);
  const arucoInterval = useRef(null);

  // Update servo angles whenever position changes
  useEffect(() => {
    const newAngles = calculateServoAngles(position.x, position.y, position.z);
    setAngles(newAngles);
  }, [position]);

  // Fetch initial cameras
  useEffect(() => {
    fetch(`${DETECTION_SERVER}/cameras`)
      .then(res => res.json())
      .then(data => {
        if (data.success) {
          setAvailableCameras(data.cameras);
          setSelectedCamera(data.current_index.toString());
        }
      })
      .catch(err => console.error("Error fetching cameras:", err));
  }, []);

  // Poll detection API if camera is active
  useEffect(() => {
    if (cameraActive) {
      detectInterval.current = setInterval(() => {
        fetch(`${DETECTION_SERVER}/detect`)
          .then(res => res.json())
          .then(data => {
            if (data.success) {
              setDetectedObjects(data.objects);
            }
          })
          .catch(() => { });
      }, 500);

      arucoInterval.current = setInterval(() => {
        fetch(`${DETECTION_SERVER}/aruco/status`)
          .then(res => res.json())
          .then(data => {
            setArucoStatus(data);
          })
          .catch(() => { });
      }, 2000);
    } else {
      clearInterval(detectInterval.current);
      clearInterval(arucoInterval.current);
    }

    return () => {
      clearInterval(detectInterval.current);
      clearInterval(arucoInterval.current);
    };
  }, [cameraActive]);

  const handleStartCamera = async () => {
    try {
      const res = await fetch(`${DETECTION_SERVER}/camera/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ camera_index: parseInt(selectedCamera) })
      });
      const data = await res.json();
      if (data.success) {
        setCameraActive(true);
      }
    } catch (error) {
      console.error(error);
    }
  };

  const handleStopCamera = async () => {
    try {
      await fetch(`${DETECTION_SERVER}/camera/stop`, { method: 'POST' });
      setCameraActive(false);
      setDetectedObjects([]);
    } catch (error) {
      console.error(error);
    }
  };

  const handleCalibrateAruco = async () => {
    try {
      const res = await fetch(`${DETECTION_SERVER}/aruco/calibrate`, { method: 'POST' });
      const data = await res.json();
      if (data.success) {
        alert("ArUco calibration successful!");
      } else {
        alert(data.message || "Calibration failed.");
      }
    } catch (err) {
      alert("Error reaching calibration server.");
    }
  };

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
              angles={angles}
            />
            <ObjectDetection
              cameraActive={cameraActive}
              onStartCamera={handleStartCamera}
              onStopCamera={handleStopCamera}
              onDetect={() => { }}
              availableCameras={availableCameras}
              selectedCamera={selectedCamera}
              setSelectedCamera={setSelectedCamera}
              detectedObjects={detectedObjects}
              onExecutePickPlace={() => { }}
            />
            <ArucoCalibration
              arucoStatus={arucoStatus}
              onCalibrate={handleCalibrateAruco}
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
