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
    """Convert pixel coordinates to robot arm XYZ coordinates"""
    # Center of image
    cx = CONFIG['frame_width'] / 2
    cy = CONFIG['frame_height'] / 2
    
    # Offset from center in pixels
    dx = px - cx
    dy = cy - py  # Y is inverted
    
    # Convert to mm
    x_mm = dx / CONFIG['pixels_per_mm'] + CONFIG['camera_offset_x']
    y_mm = dy / CONFIG['pixels_per_mm'] + CONFIG['camera_offset_y']
    
    # Clamp to workspace bounds
    x_mm = max(CONFIG['workspace_x_min'], min(CONFIG['workspace_x_max'], x_mm))
    y_mm = max(CONFIG['workspace_y_min'], min(CONFIG['workspace_y_max'], y_mm))
    
    return {
        'x': round(x_mm),
        'y': round(y_mm),
        'z': CONFIG['pick_height']
    }

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
        'version': '1.0',
        'yolo_available': YOLO_AVAILABLE,
        'model_loaded': state.model is not None,
        'camera_active': state.camera is not None and state.camera.isOpened()
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
