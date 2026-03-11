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
  const [availableModels, setAvailableModels] = useState([]);
  const [selectedModel, setSelectedModel] = useState('');
  const [isModelSwitching, setIsModelSwitching] = useState(false);
  const [showGrid, setShowGrid] = useState(false);

  const [availablePorts, setAvailablePorts] = useState([]);
  const [selectedPort, setSelectedPort] = useState('');

  // Polling intervals
  const detectInterval = useRef(null);
  const arucoInterval = useRef(null);

  // Update servo angles whenever position changes
  useEffect(() => {
    const newAngles = calculateServoAngles(position.x, position.y, position.z);
    setAngles(newAngles);
  }, [position]);

  // Fetch initial cameras and models
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

    fetch(`${DETECTION_SERVER}/models`)
      .then(res => res.json())
      .then(data => {
        if (data.success) {
          setAvailableModels(data.available_models);
          setSelectedModel(data.current_model);
        }
      })
      .catch(err => console.error("Error fetching models:", err));

    // Fetch initial grid status
    fetch(`${DETECTION_SERVER}/grid/status`)
      .then(res => res.json())
      .then(data => {
        if (data.show_grid !== undefined) {
          setShowGrid(data.show_grid);
        }
      })
      .catch(err => console.error("Error fetching grid status:", err));

    // Fetch serial ports
    fetch(`${DETECTION_SERVER}/robot/ports`)
      .then(res => res.json())
      .then(data => {
        if (data.success) {
          setAvailablePorts(data.ports);
        }
      })
      .catch(err => console.error("Error fetching ports:", err));

    // Fetch initial robot status
    fetch(`${DETECTION_SERVER}/robot/status`)
      .then(res => res.json())
      .then(data => {
        setConnected(data.connected);
        if (data.connected && data.port) {
            setSelectedPort(data.port);
        }
      })
      .catch(err => console.error("Error fetching robot status:", err));
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

  const handleModelSwitch = async (modelName) => {
    setIsModelSwitching(true);
    setSelectedModel(modelName);
    try {
      const res = await fetch(`${DETECTION_SERVER}/model/switch`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ model: modelName })
      });
      const data = await res.json();
      if (data.success) {
        console.log(`Switched to model ${data.current_model}`);
      } else {
        alert(data.error || "Failed to switch model");
      }
    } catch (err) {
      console.error(err);
      alert("Error reaching server to switch model");
    } finally {
      setIsModelSwitching(false);
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

  const handleToggleGrid = async () => {
    try {
      const res = await fetch(`${DETECTION_SERVER}/grid/toggle`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ show_grid: !showGrid })
      });
      const data = await res.json();
      if (data.success) {
        setShowGrid(data.show_grid);
      }
    } catch (err) {
      console.error("Error toggling grid:", err);
    }
  };

  const handlePositionChange = (axis, value) => {
    const newPos = { ...position, [axis]: value };
    setPosition(newPos);
    
    // Only send to robot if connected backend
    // Since sliders can fire rapidly, React state update is fine. Let's fire the API call directly.
    fetch(`${DETECTION_SERVER}/robot/move`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: newPos.x, y: newPos.y, z: newPos.z })
    }).catch(err => console.error("Error sending move command:", err));
  };

  const handleAction = async (action) => {
    console.log(`Action: ${action}`);
    if (action === 'home') {
      try {
        await fetch(`${DETECTION_SERVER}/robot/home`, { method: 'POST' });
        setPosition({ x: 0, y: 120, z: 85 });
      } catch(err) {
        console.error("Home command error:", err);
      }
    }
  };

  const handleConnect = async () => {
    if (connected) {
      try {
        await fetch(`${DETECTION_SERVER}/robot/disconnect`, { method: 'POST' });
        setConnected(false);
      } catch (err) {
        console.error("Disconnect error:", err);
      }
    } else {
      if (!selectedPort) return;
      try {
        const res = await fetch(`${DETECTION_SERVER}/robot/connect`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ port: selectedPort, baudrate: 115200 })
        });
        const data = await res.json();
        if (data.success) {
          setConnected(true);
        } else {
          alert(data.error || "Failed to connect to robot");
        }
      } catch (err) {
        console.error("Connect error:", err);
        alert("Error connecting to server");
      }
    }
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
              availableModels={availableModels}
              selectedModel={selectedModel}
              onModelSwitch={handleModelSwitch}
              isModelSwitching={isModelSwitching}
              detectedObjects={detectedObjects}
              onExecutePickPlace={() => { }}
            />
            <ArucoCalibration
              arucoStatus={arucoStatus}
              onCalibrate={handleCalibrateAruco}
              showGrid={showGrid}
              onToggleGrid={handleToggleGrid}
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
              availablePorts={availablePorts}
              selectedPort={selectedPort}
              setSelectedPort={setSelectedPort}
              connected={connected}
              onConnect={handleConnect}
            />
          </div>
        </main>

        <footer className="footer">
          <p>🤖 Python/ESP32 Robot Arm Controller | React Frontend</p>
        </footer>
      </div>
    </>
  );
}

export default App;
