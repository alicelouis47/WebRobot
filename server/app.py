"""
YOLOv11 Object Detection Server
Flask-based server for robot arm object detection and pick-place control
Uses USB webcam for video capture
"""

from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import cv2
import numpy as np
import json
import threading
import time
import platform
import glob
try:
    from pygrabber.dshow_graph import FilterGraph
    PYGRABBER_AVAILABLE = True
except ImportError:
    PYGRABBER_AVAILABLE = False
    print("⚠️ pygrabber not installed. Camera names might not be available.")

# Try to import ultralytics for YOLOv11
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ Ultralytics not installed. Run: pip install ultralytics")

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================
# Configuration
# ============================================
CONFIG = {
    'camera_index': 1,  # USB webcam index (0 = default camera)
    'frame_width': 1920,
    'frame_height': 1080,
    'confidence_threshold': 0.5,
    
    # Camera calibration (adjust based on your setup)
    'camera_height_mm': 300,  # Height of camera above workspace
    'camera_offset_x': 0,     # Camera offset from robot base (X)
    'camera_offset_y': 150,   # Camera offset from robot base (Y)
    'pixels_per_mm': 2.5,     # Calibration factor
    
    # Workspace bounds (mm) (A4 size: 297x210 mm, landscape, starting at X=50)
    'workspace_x_min': 50,
    'workspace_x_max': 260,   # 50 + 210
    'workspace_y_min': -148.5, # -297/2
    'workspace_y_max': 148.5,  # 297/2
    'pick_height': 10,        # Height when picking object
    'safe_height': 80,        # Safe travel height
    
    # ArUco Marker Configuration
    'aruco_dict_type': cv2.aruco.DICT_4X4_50,
    'marker_size_mm': 30,     # Physical marker size in mm
    # Expected marker IDs based on A4 configuration:
    # [Top edge of A4 paper - Landscape 297x210mm]
    #   | 4 cm 
    #  [0]------- 12 cm -------[1]  <- Furthest from robot (Top)
    #   |                       |
    #  8 cm                    8 cm
    #   |                       |
    #  [2]------- 12 cm -------[3]  <- Closest to robot (Bottom)
    'marker_ids': [0, 1, 2, 3],
    
    # Real-world coordinates of marker centers (mm) relative to robot base
    # Robot is at (0,0). X axis is Forward, Y axis is Left.
    # We assume A4 paper (Landscape) is placed with its bottom edge at X = 50 mm from the robot base.
    # Center of paper is at Y = 0.
    # Top edge X = 50 + 210 = 260 mm. 
    # ID:0 and ID:1 are 4 cm (40 mm) from top edge -> X = 260 - 40 = 220 mm.
    # ID:2 and ID:3 are 8 cm (80 mm) closer than ID:0 -> X = 220 - 80 = 140 mm.
    # Y-axis is centered. Width = 120 mm -> Left is +60 mm, Right is -60 mm.
    'marker_real_coords': {
        0: (220, 60),    # Top-Left (ID:0)
        1: (220, -60),   # Top-Right (ID:1)
        2: (140, 60),    # Bottom-Left (ID:2)
        3: (140, -60),   # Bottom-Right (ID:3)
    },
}

# ============================================
# Global State
# ============================================
class DetectionState:
    def __init__(self):
        self.camera = None
        self.is_running = False
        self.current_frame = None
        self.detected_objects = []
        self.model = None
        self.current_model_name = 'yolo11n.pt'
        self.lock = threading.Lock()
        
        # ArUco calibration state
        self.aruco_dict = None
        self.aruco_params = None
        self.homography_matrix = None
        self.aruco_calibrated = False
        self.detected_markers = {}  # {id: center_point}
        self.last_calibration_time = None
        self.show_grid = False  # Display 5mm grid overlay
        
state = DetectionState()

# ============================================
# YOLO Model
# ============================================
def load_yolo_model(model_name='yolo11n.pt'):
    """Load YOLO model"""
    if not YOLO_AVAILABLE:
        print("❌ YOLO not available")
        return None
    
    try:
        model = YOLO(model_name)
        print(f"✅ YOLO model {model_name} loaded successfully")
        return model
    except Exception as e:
        print(f"❌ Failed to load YOLO model {model_name}: {e}")
        return None

# ============================================
# Camera Functions
# ============================================
def init_camera():
    """Initialize USB webcam"""
    if state.camera is not None:
        return True
    
    try:
        state.camera = cv2.VideoCapture(CONFIG['camera_index'])
        # Request MJPG for better performance at high resolutions
        state.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        state.camera.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG['frame_width'])
        state.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG['frame_height'])
        
        if not state.camera.isOpened():
            print("❌ Failed to open camera")
            return False
        
        print(f"✅ Camera initialized (index: {CONFIG['camera_index']})")
        return True
    except Exception as e:
        print(f"❌ Camera error: {e}")
        return False

def release_camera():
    """Release camera resources"""
    if state.camera is not None:
        state.camera.release()
        state.camera = None
        print("📷 Camera released")

def get_available_cameras():
    """Get list of available cameras and their names"""
    cameras = []
    
    # Use pygrabber on Windows to get actual camera names
    if platform.system() == 'Windows' and PYGRABBER_AVAILABLE:
        try:
            graph = FilterGraph()
            devices = graph.get_input_devices()
            for i, name in enumerate(devices):
                cameras.append({'index': i, 'name': name})
            return cameras
        except Exception as e:
            print(f"Failed to get camera names with pygrabber: {e}")
            # Fall back to CV2 method if pygrabber fails
            
    # Fallback/Linux/Mac method: test indices up to 5
    for i in range(5):
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                has_frame, _ = cap.read()
                if has_frame:
                    cameras.append({
                        'index': i, 
                        'name': f"Camera {i}"
                    })
                cap.release()
        except Exception:
            pass
            
    # If no cameras found at all, return at least a default option
    if not cameras:
        cameras = [{'index': 0, 'name': "Default Camera (0)"}, {'index': 1, 'name': "USB Camera 1"}]
        
    return cameras

def get_frame():
    """Capture a frame from webcam"""
    if state.camera is None or not state.camera.isOpened():
        return None
    
    ret, frame = state.camera.read()
    if ret:
        return frame
    return None

# ============================================
# Object Detection
# ============================================
def detect_objects(frame):
    """Run YOLOv11 detection on frame"""
    if state.model is None:
        return frame, []
    
    results = state.model(frame, conf=CONFIG['confidence_threshold'], verbose=False)
    
    detected = []
    annotated_frame = frame.copy()
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Get box coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = state.model.names[class_id]
            
            # Calculate center point
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # Convert pixel to robot coordinates
            robot_coords = pixel_to_robot_coords(center_x, center_y)
            
            detected.append({
                'id': len(detected),
                'class': class_name,
                'confidence': round(confidence, 2),
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'center_pixel': [center_x, center_y],
                'robot_coords': robot_coords
            })
            
            # Draw bounding box
            color = (0, 255, 0)  # Green
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # Draw label
            label = f"{class_name} {confidence:.2f}"
            cv2.putText(annotated_frame, label, (int(x1), int(y1) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Draw robot coordinates
            coord_text = f"X:{robot_coords['x']} Y:{robot_coords['y']}"
            cv2.putText(annotated_frame, coord_text, (int(x1), int(y2) + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    return annotated_frame, detected

def pixel_to_robot_coords(px, py):
    """Convert pixel coordinates to robot arm XYZ coordinates
    Uses ArUco homography if calibrated, otherwise falls back to simple method
    """
    # If ArUco calibrated, use homography transform
    if state.aruco_calibrated and state.homography_matrix is not None:
        return pixel_to_robot_coords_aruco(px, py)
    
    # Fallback: Simple calibration method
    cx = CONFIG['frame_width'] / 2
    cy = CONFIG['frame_height'] / 2
    
    dx = px - cx
    dy = cy - py  # Y is inverted
    
    x_mm = dx / CONFIG['pixels_per_mm'] + CONFIG['camera_offset_x']
    y_mm = dy / CONFIG['pixels_per_mm'] + CONFIG['camera_offset_y']
    
    x_mm = max(CONFIG['workspace_x_min'], min(CONFIG['workspace_x_max'], x_mm))
    y_mm = max(CONFIG['workspace_y_min'], min(CONFIG['workspace_y_max'], y_mm))
    
    return {
        'x': round(x_mm),
        'y': round(y_mm),
        'z': CONFIG['pick_height']
    }

# ============================================
# ArUco Marker Detection
# ============================================
def init_aruco():
    """Initialize ArUco detector"""
    state.aruco_dict = cv2.aruco.getPredefinedDictionary(CONFIG['aruco_dict_type'])
    state.aruco_params = cv2.aruco.DetectorParameters()
    print("✅ ArUco detector initialized")

def detect_aruco_markers(frame):
    """Detect ArUco markers in frame and return their positions"""
    if state.aruco_dict is None:
        init_aruco()
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(state.aruco_dict, state.aruco_params)
    corners, ids, rejected = detector.detectMarkers(gray)
    
    markers = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in CONFIG['marker_ids']:
                # Calculate center of marker
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                markers[int(marker_id)] = (center_x, center_y)
    
    return markers, corners, ids

def calibrate_aruco(frame):
    """Calibrate using detected ArUco markers to compute homography"""
    markers, corners, ids = detect_aruco_markers(frame)
    
    state.detected_markers = markers
    
    # Need all 4 markers for calibration
    required_ids = CONFIG['marker_ids']
    if len(markers) < 4:
        missing = [mid for mid in required_ids if mid not in markers]
        return {
            'success': False,
            'detected': len(markers),
            'total': 4,
            'missing_ids': missing,
            'message': f'Only {len(markers)}/4 markers detected. Missing: {missing}'
        }
    
    # Build correspondence: pixel points -> real world points
    src_points = []  # Pixel coordinates
    dst_points = []  # Real world coordinates (mm)
    
    for marker_id in required_ids:
        if marker_id in markers:
            px, py = markers[marker_id]
            rx, ry = CONFIG['marker_real_coords'][marker_id]
            src_points.append([px, py])
            dst_points.append([rx, ry])
    
    src_points = np.array(src_points, dtype=np.float32)
    dst_points = np.array(dst_points, dtype=np.float32)
    
    # Compute homography matrix
    H, status = cv2.findHomography(src_points, dst_points)
    
    if H is not None:
        state.homography_matrix = H
        state.aruco_calibrated = True
        state.last_calibration_time = time.strftime('%Y-%m-%d %H:%M:%S')
        print(f"✅ ArUco calibration successful at {state.last_calibration_time}")
        return {
            'success': True,
            'detected': 4,
            'total': 4,
            'calibration_time': state.last_calibration_time,
            'message': 'ArUco calibration successful!'
        }
    else:
        return {
            'success': False,
            'message': 'Failed to compute homography matrix'
        }

def pixel_to_robot_coords_aruco(px, py):
    """Convert pixel coordinates using ArUco homography"""
    if state.homography_matrix is None:
        return pixel_to_robot_coords(px, py)
    
    # Apply homography transform
    point = np.array([[[px, py]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, state.homography_matrix)
    
    x_mm = float(transformed[0][0][0])
    y_mm = float(transformed[0][0][1])
    
    # Clamp to workspace bounds
    x_mm = max(CONFIG['workspace_x_min'], min(CONFIG['workspace_x_max'], x_mm))
    y_mm = max(CONFIG['workspace_y_min'], min(CONFIG['workspace_y_max'], y_mm))
    
    return {
        'x': round(x_mm),
        'y': round(y_mm),
        'z': CONFIG['pick_height']
    }

def draw_aruco_overlay(frame):
    """Draw ArUco markers and workspace bounds on frame"""
    markers, corners, ids = detect_aruco_markers(frame)
    
    # Draw detected markers
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Draw marker status
    status_text = f"ArUco: {len(markers)}/4"
    color = (0, 255, 0) if state.aruco_calibrated else (0, 165, 255)
    cv2.putText(frame, status_text, (10, 25), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    if state.aruco_calibrated:
        cv2.putText(frame, "CALIBRATED", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw workspace bounds if all 4 markers detected
        if len(markers) >= 4:
            pts = []
            for mid in CONFIG['marker_ids']:
                if mid in markers:
                    pts.append(markers[mid])
            if len(pts) == 4:
                pts = np.array(pts, dtype=np.int32)
                cv2.polylines(frame, [pts], True, (0, 255, 255), 2)
                
            # Draw 5mm grid if enabled
            if state.show_grid and state.homography_matrix is not None:
                # Calculate inverse homography to map real coordinates back to pixels
                inv_H = np.linalg.inv(state.homography_matrix)
                
                # Get workspace bounds
                min_x = int(CONFIG['workspace_x_min'])
                max_x = int(CONFIG['workspace_x_max'])
                min_y = int(CONFIG['workspace_y_min'])
                max_y = int(CONFIG['workspace_y_max'])
                
                # Generate grid lines at 5mm intervals
                grid_color = (255, 255, 0) # Cyan grid lines
                grid_thickness = 1
                
                # Draw vertical lines (constant Y)
                for y in range(min_y, max_y + 1, 5):
                    # Start and end points in real world
                    pt1_real = np.array([[[min_x, y]]], dtype=np.float32)
                    pt2_real = np.array([[[max_x, y]]], dtype=np.float32)
                    
                    # Convert to pixel coordinates
                    pt1_px = cv2.perspectiveTransform(pt1_real, inv_H)
                    pt2_px = cv2.perspectiveTransform(pt2_real, inv_H)
                    
                    # Draw line
                    cv2.line(frame, 
                             (int(pt1_px[0][0][0]), int(pt1_px[0][0][1])), 
                             (int(pt2_px[0][0][0]), int(pt2_px[0][0][1])), 
                             grid_color, grid_thickness)
                             
                # Draw horizontal lines (constant X)
                for x in range(min_x, max_x + 1, 5):
                    # Start and end points in real world
                    pt1_real = np.array([[[x, min_y]]], dtype=np.float32)
                    pt2_real = np.array([[[x, max_y]]], dtype=np.float32)
                    
                    # Convert to pixel coordinates
                    pt1_px = cv2.perspectiveTransform(pt1_real, inv_H)
                    pt2_px = cv2.perspectiveTransform(pt2_real, inv_H)
                    
                    # Draw line
                    cv2.line(frame, 
                             (int(pt1_px[0][0][0]), int(pt1_px[0][0][1])), 
                             (int(pt2_px[0][0][0]), int(pt2_px[0][0][1])), 
                             grid_color, grid_thickness)

    return frame

# ============================================
# Video Stream
# ============================================
def generate_frames():
    """Generator for MJPEG video stream"""
    while True:
        frame = get_frame()
        if frame is None:
            time.sleep(0.1)
            continue
        
        # Run detection if model is loaded
        if state.model is not None:
            frame, detected = detect_objects(frame)
            with state.lock:
                state.detected_objects = detected
        
        # Draw ArUco overlay
        frame = draw_aruco_overlay(frame)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ============================================
# API Routes
# ============================================
@app.get('/')
def index():
    """API info"""
    return {
        'name': 'Robot Arm Object Detection Server',
        'version': '1.1',
        'yolo_available': YOLO_AVAILABLE,
        'model_loaded': state.model is not None,
        'current_model': state.current_model_name,
        'camera_active': state.camera is not None and state.camera.isOpened(),
        'aruco_calibrated': state.aruco_calibrated,
        'aruco_last_calibration': state.last_calibration_time
    }

@app.get('/models')
def get_models():
    """Get available YOLO models"""
    pt_files = glob.glob('*.pt')
    models = [{'id': f, 'name': f} for f in pt_files]
    
    if not models:
        models = [
            {'id': 'yolo11n.pt', 'name': 'yolo11n.pt'},
            {'id': 'yolo26.pt', 'name': 'yolo26.pt'}
        ]
        
    return {
        'success': True,
        'current_model': state.current_model_name,
        'available_models': models
    }

@app.post('/model/switch')
async def switch_model(request: Request):
    """Switch to a different YOLO model"""
    data = await request.json()
    model_name = data.get('model')
    
    if not model_name:
        return JSONResponse(status_code=400, content={'error': 'Model name required'})
        
    with state.lock:
        new_model = load_yolo_model(model_name)
        if new_model is not None:
            state.model = new_model
            state.current_model_name = model_name
            return {'success': True, 'current_model': model_name}
        else:
            return JSONResponse(status_code=500, content={'error': f'Failed to load model {model_name}'})

@app.get('/video_feed')
def video_feed():
    """MJPEG video stream"""
    if not init_camera():
        return JSONResponse(status_code=500, content={'error': 'Camera not available'})
    
    return StreamingResponse(generate_frames(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.get('/detect')
def detect():
    """Get current detected objects"""
    with state.lock:
        return {
            'success': True,
            'objects': state.detected_objects,
            'count': len(state.detected_objects)
        }

@app.get('/snapshot')
def snapshot():
    """Capture single frame and detect objects"""
    if not init_camera():
        return JSONResponse(status_code=500, content={'error': 'Camera not available'})
    
    frame = get_frame()
    if frame is None:
        return JSONResponse(status_code=500, content={'error': 'Failed to capture frame'})
    
    if state.model is not None:
        _, detected = detect_objects(frame)
        return {
            'success': True,
            'objects': detected,
            'count': len(detected)
        }
    else:
        return {
            'success': False,
            'error': 'YOLO model not loaded',
            'objects': []
        }

@app.get('/pick_sequence/{object_id}')
def pick_sequence(object_id: int):
    """Generate pick sequence for object"""
    with state.lock:
        objects = state.detected_objects
    
    if object_id >= len(objects):
        return JSONResponse(status_code=404, content={'error': 'Object not found'})
    
    obj = objects[object_id]
    coords = obj['robot_coords']
    
    # Generate pick sequence
    sequence = [
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['safe_height'], 'desc': 'Move above object'},
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['pick_height'], 'desc': 'Lower to object'},
        {'action': 'grab', 'desc': 'Close gripper'},
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['safe_height'], 'desc': 'Lift object'},
    ]
    
    return {
        'success': True,
        'object': obj,
        'sequence': sequence
    }

@app.post('/place_sequence')
async def place_sequence(request: Request):
    """Generate place sequence for target location"""
    data = await request.json()
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', CONFIG['pick_height'])
    
    sequence = [
        {'action': 'move', 'x': x, 'y': y, 'z': CONFIG['safe_height'], 'desc': 'Move above target'},
        {'action': 'move', 'x': x, 'y': y, 'z': z, 'desc': 'Lower to target'},
        {'action': 'release', 'desc': 'Open gripper'},
        {'action': 'move', 'x': x, 'y': y, 'z': CONFIG['safe_height'], 'desc': 'Lift arm'},
        {'action': 'home', 'desc': 'Return home'},
    ]
    
    return {
        'success': True,
        'target': {'x': x, 'y': y, 'z': z},
        'sequence': sequence
    }

@app.get('/calibration')
def get_calibration():
    """Get calibration parameters"""
    return {
        'camera_height_mm': CONFIG['camera_height_mm'],
        'camera_offset_x': CONFIG['camera_offset_x'],
        'camera_offset_y': CONFIG['camera_offset_y'],
        'pixels_per_mm': CONFIG['pixels_per_mm'],
    }

@app.post('/calibration')
async def update_calibration(request: Request):
    """Update calibration parameters"""
    data = await request.json()
    for key in ['camera_height_mm', 'camera_offset_x', 'camera_offset_y', 'pixels_per_mm']:
        if key in data:
            CONFIG[key] = data[key]
    return {'success': True, 'config': CONFIG}

@app.get('/aruco/calibrate')
@app.post('/aruco/calibrate')
def aruco_calibrate():
    """Calibrate using ArUco markers"""
    if not init_camera():
        return JSONResponse(status_code=500, content={'error': 'Camera not available'})
    
    frame = get_frame()
    if frame is None:
        return JSONResponse(status_code=500, content={'error': 'Failed to capture frame'})
    
    result = calibrate_aruco(frame)
    return result

@app.get('/aruco/status')
def aruco_status():
    """Get ArUco calibration status"""
    # Get current marker detection if camera is active
    markers = {}
    if state.camera is not None and state.camera.isOpened():
        frame = get_frame()
        if frame is not None:
            markers, _, _ = detect_aruco_markers(frame)
    
    return {
        'calibrated': state.aruco_calibrated,
        'last_calibration': state.last_calibration_time,
        'markers_detected': len(markers),
        'markers_required': 4,
        'detected_ids': list(markers.keys()),
        'required_ids': CONFIG['marker_ids'],
        'marker_coords': CONFIG['marker_real_coords']
    }

@app.get('/aruco/config')
def get_aruco_config():
    """Get ArUco marker configuration"""
    return {
        'marker_size_mm': CONFIG['marker_size_mm'],
        'marker_ids': CONFIG['marker_ids'],
        'marker_real_coords': CONFIG['marker_real_coords']
    }

@app.post('/aruco/config')
async def update_aruco_config(request: Request):
    """Update ArUco marker configuration"""
    data = await request.json()
    if 'marker_real_coords' in data:
        # Update marker real-world coordinates
        for key, value in data['marker_real_coords'].items():
            CONFIG['marker_real_coords'][int(key)] = tuple(value)
    if 'marker_size_mm' in data:
        CONFIG['marker_size_mm'] = data['marker_size_mm']
    # Reset calibration when config changes
    state.aruco_calibrated = False
    state.homography_matrix = None
    return {'success': True, 'message': 'ArUco config updated. Please recalibrate.'}

@app.get('/cameras')
def get_cameras_list():
    """Get list of available cameras"""
    cameras = get_available_cameras()
    return {
        'success': True,
        'cameras': cameras,
        'current_index': CONFIG['camera_index']
    }

@app.get('/grid/status')
def get_grid_status():
    """Get grid display status"""
    return {'show_grid': state.show_grid}

@app.post('/grid/toggle')
async def toggle_grid(request: Request):
    """Toggle grid display over camera feed"""
    data = await request.json()
    if 'show_grid' in data:
        state.show_grid = bool(data['show_grid'])
    else:
        state.show_grid = not state.show_grid
        
    return {'success': True, 'show_grid': state.show_grid}

@app.post('/camera/start')
async def camera_start(request: Request):
    """Start camera"""
    try:
        data = await request.json()
        if 'camera_index' in data:
            new_index = int(data['camera_index'])
            if state.camera is not None and CONFIG['camera_index'] != new_index:
                release_camera()
            CONFIG['camera_index'] = new_index
    except Exception:
        pass

    success = init_camera()
    return {'success': success}

@app.post('/camera/stop')
def camera_stop():
    """Stop camera"""
    release_camera()
    return {'success': True}

# ============================================
# Main
# ============================================
if __name__ == '__main__':
    print("=" * 50)
    print("  Robot Arm Object Detection Server")
    print("=" * 50)
    
    # Load YOLO model
    state.model = load_yolo_model(state.current_model_name)
    
    # Initialize camera
    init_camera()
    
    print("\n📡 Starting server on http://localhost:5000")
    print("📹 Video stream: http://localhost:5000/video_feed")
    print("🎯 Detection API: http://localhost:5000/detect")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        uvicorn.run(app, host='0.0.0.0', port=5000)
    finally:
        release_camera()
