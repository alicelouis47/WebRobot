"""
YOLOv11 Object Detection Server
Flask-based server for robot arm object detection and pick-place control
Uses USB webcam for video capture
"""

from flask import Flask, Response, jsonify, request
from flask_cors import CORS
import cv2
import numpy as np
import json
import threading
import time

# Try to import ultralytics for YOLOv11
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ Ultralytics not installed. Run: pip install ultralytics")

app = Flask(__name__)
CORS(app)

# ============================================
# Configuration
# ============================================
CONFIG = {
    'camera_index': 1,  # USB webcam index (0 = default camera)
    'frame_width': 640,
    'frame_height': 480,
    'confidence_threshold': 0.5,
    
    # Camera calibration (adjust based on your setup)
    'camera_height_mm': 300,  # Height of camera above workspace
    'camera_offset_x': 0,     # Camera offset from robot base (X)
    'camera_offset_y': 150,   # Camera offset from robot base (Y)
    'pixels_per_mm': 2.5,     # Calibration factor
    
    # Workspace bounds (mm)
    'workspace_x_min': -100,
    'workspace_x_max': 100,
    'workspace_y_min': -100,
    'workspace_y_max': 100,
    'pick_height': 10,        # Height when picking object
    'safe_height': 80,        # Safe travel height
    
    # ArUco Marker Configuration
    'aruco_dict_type': cv2.aruco.DICT_4X4_50,
    'marker_size_mm': 30,     # Physical marker size in mm
    # Expected marker IDs at workspace corners (ID: position)
    # Arrange markers like this:
    #   [0]-------[1]
    #    |         |
    #    |         |
    #   [3]-------[2]
    'marker_ids': [0, 1, 2, 3],
    # Real-world coordinates of marker centers (mm) relative to robot base
    'marker_real_coords': {
        0: (-80, 120),   # Top-left
        1: (80, 120),    # Top-right
        2: (80, 40),     # Bottom-right
        3: (-80, 40),    # Bottom-left
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
        self.lock = threading.Lock()
        
        # ArUco calibration state
        self.aruco_dict = None
        self.aruco_params = None
        self.homography_matrix = None
        self.aruco_calibrated = False
        self.detected_markers = {}  # {id: center_point}
        self.last_calibration_time = None
        
state = DetectionState()

# ============================================
# YOLO Model
# ============================================
def load_yolo_model():
    """Load YOLOv11 model"""
    if not YOLO_AVAILABLE:
        print("❌ YOLO not available")
        return None
    
    try:
        # Use YOLOv11 nano for faster inference
        # You can change to 'yolo11s.pt', 'yolo11m.pt' for better accuracy
        model = YOLO('yolo11n.pt')
        print("✅ YOLOv11 model loaded successfully")
        return model
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
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
@app.route('/')
def index():
    """API info"""
    return jsonify({
        'name': 'Robot Arm Object Detection Server',
        'version': '1.1',
        'yolo_available': YOLO_AVAILABLE,
        'model_loaded': state.model is not None,
        'camera_active': state.camera is not None and state.camera.isOpened(),
        'aruco_calibrated': state.aruco_calibrated,
        'aruco_last_calibration': state.last_calibration_time
    })

@app.route('/video_feed')
def video_feed():
    """MJPEG video stream"""
    if not init_camera():
        return jsonify({'error': 'Camera not available'}), 500
    
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detect', methods=['GET'])
def detect():
    """Get current detected objects"""
    with state.lock:
        return jsonify({
            'success': True,
            'objects': state.detected_objects,
            'count': len(state.detected_objects)
        })

@app.route('/snapshot', methods=['GET'])
def snapshot():
    """Capture single frame and detect objects"""
    if not init_camera():
        return jsonify({'error': 'Camera not available'}), 500
    
    frame = get_frame()
    if frame is None:
        return jsonify({'error': 'Failed to capture frame'}), 500
    
    if state.model is not None:
        _, detected = detect_objects(frame)
        return jsonify({
            'success': True,
            'objects': detected,
            'count': len(detected)
        })
    else:
        return jsonify({
            'success': False,
            'error': 'YOLO model not loaded',
            'objects': []
        })

@app.route('/pick_sequence/<int:object_id>', methods=['GET'])
def pick_sequence(object_id):
    """Generate pick sequence for object"""
    with state.lock:
        objects = state.detected_objects
    
    if object_id >= len(objects):
        return jsonify({'error': 'Object not found'}), 404
    
    obj = objects[object_id]
    coords = obj['robot_coords']
    
    # Generate pick sequence
    sequence = [
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['safe_height'], 'desc': 'Move above object'},
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['pick_height'], 'desc': 'Lower to object'},
        {'action': 'grab', 'desc': 'Close gripper'},
        {'action': 'move', 'x': coords['x'], 'y': coords['y'], 'z': CONFIG['safe_height'], 'desc': 'Lift object'},
    ]
    
    return jsonify({
        'success': True,
        'object': obj,
        'sequence': sequence
    })

@app.route('/place_sequence', methods=['POST'])
def place_sequence():
    """Generate place sequence for target location"""
    data = request.json
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
    
    return jsonify({
        'success': True,
        'target': {'x': x, 'y': y, 'z': z},
        'sequence': sequence
    })

@app.route('/calibration', methods=['GET', 'POST'])
def calibration():
    """Get or update calibration parameters"""
    if request.method == 'GET':
        return jsonify({
            'camera_height_mm': CONFIG['camera_height_mm'],
            'camera_offset_x': CONFIG['camera_offset_x'],
            'camera_offset_y': CONFIG['camera_offset_y'],
            'pixels_per_mm': CONFIG['pixels_per_mm'],
        })
    else:
        data = request.json
        for key in ['camera_height_mm', 'camera_offset_x', 'camera_offset_y', 'pixels_per_mm']:
            if key in data:
                CONFIG[key] = data[key]
        return jsonify({'success': True, 'config': CONFIG})

@app.route('/aruco/calibrate', methods=['GET', 'POST'])
def aruco_calibrate():
    """Calibrate using ArUco markers"""
    if not init_camera():
        return jsonify({'error': 'Camera not available'}), 500
    
    frame = get_frame()
    if frame is None:
        return jsonify({'error': 'Failed to capture frame'}), 500
    
    result = calibrate_aruco(frame)
    return jsonify(result)

@app.route('/aruco/status', methods=['GET'])
def aruco_status():
    """Get ArUco calibration status"""
    # Get current marker detection if camera is active
    markers = {}
    if state.camera is not None and state.camera.isOpened():
        frame = get_frame()
        if frame is not None:
            markers, _, _ = detect_aruco_markers(frame)
    
    return jsonify({
        'calibrated': state.aruco_calibrated,
        'last_calibration': state.last_calibration_time,
        'markers_detected': len(markers),
        'markers_required': 4,
        'detected_ids': list(markers.keys()),
        'required_ids': CONFIG['marker_ids'],
        'marker_coords': CONFIG['marker_real_coords']
    })

@app.route('/aruco/config', methods=['GET', 'POST'])
def aruco_config():
    """Get or update ArUco marker configuration"""
    if request.method == 'GET':
        return jsonify({
            'marker_size_mm': CONFIG['marker_size_mm'],
            'marker_ids': CONFIG['marker_ids'],
            'marker_real_coords': CONFIG['marker_real_coords']
        })
    else:
        data = request.json
        if 'marker_real_coords' in data:
            # Update marker real-world coordinates
            for key, value in data['marker_real_coords'].items():
                CONFIG['marker_real_coords'][int(key)] = tuple(value)
        if 'marker_size_mm' in data:
            CONFIG['marker_size_mm'] = data['marker_size_mm']
        # Reset calibration when config changes
        state.aruco_calibrated = False
        state.homography_matrix = None
        return jsonify({'success': True, 'message': 'ArUco config updated. Please recalibrate.'})

@app.route('/camera/start', methods=['POST'])
def camera_start():
    """Start camera"""
    success = init_camera()
    return jsonify({'success': success})

@app.route('/camera/stop', methods=['POST'])
def camera_stop():
    """Stop camera"""
    release_camera()
    return jsonify({'success': True})

# ============================================
# Main
# ============================================
if __name__ == '__main__':
    print("=" * 50)
    print("  Robot Arm Object Detection Server")
    print("=" * 50)
    
    # Load YOLO model
    state.model = load_yolo_model()
    
    # Initialize camera
    init_camera()
    
    print("\n📡 Starting server on http://localhost:5000")
    print("📹 Video stream: http://localhost:5000/video_feed")
    print("🎯 Detection API: http://localhost:5000/detect")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        release_camera()
