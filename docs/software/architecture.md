# Software Architecture Overview

This document provides an overview of the software architecture for the robotics projects in this repository.

## System Architecture

### High-Level Components

```
┌─────────────────────────────────────────────────────┐
│                  User Interface                     │
│              (CLI / GUI / ROS tools)                │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│              Application Layer                       │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────┐ │
│  │ Vision       │  │  Control     │  │ Planning  │ │
│  │ Processing   │  │  Systems     │  │ Algorithms│ │
│  └──────────────┘  └──────────────┘  └───────────┘ │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│              Hardware Abstraction Layer             │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────┐ │
│  │ Camera       │  │  Servo       │  │  GPIO     │ │
│  │ Interface    │  │  Controllers │  │  Interface│ │
│  └──────────────┘  └──────────────┘  └───────────┘ │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│                  Hardware Layer                      │
│  [Cameras] [Servos] [Sensors] [Communication]       │
└─────────────────────────────────────────────────────┘
```

## Project Architectures

### ResponsiveArm.pi

**Component Structure:**
```
ResponsiveArm/
├── Hardware Layer
│   ├── PCA9685 (PWM Controller)
│   ├── Servo Motors (16 channels)
│   └── GPIO Interfaces
│
├── Hardware Abstraction
│   ├── ServoKit Wrapper
│   ├── Channel Mapping
│   └── Safety Limits
│
├── Control Layer
│   ├── Position Control
│   ├── Trajectory Planning
│   └── Inverse Kinematics (to be added)
│
├── ROS Integration
│   ├── ROS Node
│   ├── Topic Subscriptions
│   └── Service Interfaces
│
└── Application Layer
    ├── Initialization Routines
    ├── Movement Sequences
    └── Emergency Stop
```

**Data Flow:**
```
ROS Command → Validation → IK Solver → Trajectory → Servo Control → Hardware
                                                            ↓
                                                      Feedback Loop
```

### ThreatDetection.IMX477

**Component Structure:**
```
ThreatDetection/
├── Hardware Layer
│   ├── IMX477 Camera
│   ├── Storage Systems
│   └── Network Interfaces
│
├── Image Acquisition
│   ├── Camera Control
│   ├── Frame Capture
│   └── Buffer Management
│
├── Processing Pipeline
│   ├── Preprocessing
│   ├── Object Detection
│   ├── Classification
│   └── Tracking
│
├── Analysis Layer
│   ├── Threat Assessment
│   ├── Pattern Recognition
│   └── Decision Logic
│
├── Alert System
│   ├── Alert Generation
│   ├── Notification Service
│   └── Logging
│
└── Security Layer
    ├── Encryption
    ├── Access Control
    └── Audit Logging
```

**Data Flow:**
```
Camera → Capture → Preprocess → Detect → Classify → Assess → Alert
                                              ↓
                                         Log & Store
```

### Lab/OpenCV_Robotics

**Learning Module Structure:**
```
OpenCV_Robotics/
├── Core Utilities
│   ├── ImageUtils
│   ├── ROI Tools
│   ├── ColorSpace Converters
│   └── Performance Timers
│
├── Basic Modules
│   ├── Image I/O
│   ├── Color Spaces
│   ├── Filtering
│   └── Thresholding
│
├── Advanced Modules
│   ├── Edge Detection
│   ├── Feature Detection
│   ├── Optical Flow
│   └── Calibration
│
└── Integration Examples
    ├── Robot Vision
    ├── Object Tracking
    └── 3D Reconstruction
```

## Design Patterns

### Hardware Abstraction Pattern

**Purpose:** Isolate hardware-specific code from application logic

```python
class HardwareInterface(ABC):
    """Abstract base class for hardware interfaces."""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize hardware."""
        pass
    
    @abstractmethod
    def read(self) -> Any:
        """Read from hardware."""
        pass
    
    @abstractmethod
    def write(self, data: Any) -> bool:
        """Write to hardware."""
        pass
    
    @abstractmethod
    def shutdown(self):
        """Safely shutdown hardware."""
        pass

class ServoController(HardwareInterface):
    """Concrete implementation for servo control."""
    pass
```

### Observer Pattern for Events

**Purpose:** Decouple event generation from handling

```python
class EventSystem:
    """Event notification system."""
    
    def __init__(self):
        self._observers = {}
    
    def subscribe(self, event_type: str, callback: Callable):
        """Subscribe to event."""
        if event_type not in self._observers:
            self._observers[event_type] = []
        self._observers[event_type].append(callback)
    
    def notify(self, event_type: str, data: Any):
        """Notify observers of event."""
        for callback in self._observers.get(event_type, []):
            callback(data)
```

### Strategy Pattern for Algorithms

**Purpose:** Allow runtime selection of algorithms

```python
class DetectionStrategy(ABC):
    """Abstract detection strategy."""
    
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        pass

class YOLODetection(DetectionStrategy):
    """YOLO-based detection."""
    def detect(self, image: np.ndarray) -> List[Detection]:
        # YOLO implementation
        pass

class TensorFlowDetection(DetectionStrategy):
    """TensorFlow-based detection."""
    def detect(self, image: np.ndarray) -> List[Detection]:
        # TensorFlow implementation
        pass
```

## Data Management

### Configuration Management

**Hierarchical Configuration:**
```python
# config.yaml
system:
  name: "ResponsiveArm"
  version: "1.0.0"
  
hardware:
  servo_controller:
    address: 0x40
    frequency: 50
    
  servos:
    - channel: 0
      min_pulse: 500
      max_pulse: 2500
      
safety:
  emergency_stop_pin: 17
  max_velocity: 180  # deg/s
  workspace_limits:
    x: [-500, 500]
    y: [-500, 500]
    z: [0, 1000]
```

### Data Persistence

**Logging Structure:**
```
logs/
├── system.log           # General system logs
├── errors.log           # Error logs
├── operations.log       # Operational logs
└── security/
    ├── access.log       # Access logs
    └── alerts.log       # Security alerts
```

### State Management

**State Machine Example:**
```python
from enum import Enum, auto

class SystemState(Enum):
    UNINITIALIZED = auto()
    INITIALIZING = auto()
    READY = auto()
    RUNNING = auto()
    PAUSED = auto()
    ERROR = auto()
    EMERGENCY_STOP = auto()

class StateMachine:
    """System state management."""
    
    def __init__(self):
        self._state = SystemState.UNINITIALIZED
        self._transitions = self._define_transitions()
    
    def transition(self, new_state: SystemState) -> bool:
        """Attempt state transition."""
        if new_state in self._transitions.get(self._state, []):
            self._state = new_state
            return True
        return False
```

## Communication Protocols

### ROS Integration

**Node Structure:**
```python
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

class RobotNode:
    """ROS node for robot control."""
    
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # Publishers
        self.status_pub = rospy.Publisher(
            '/robot/status', 
            RobotStatus, 
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            '/robot/command', 
            Pose, 
            self.command_callback
        )
        
        # Services
        rospy.Service(
            '/robot/emergency_stop', 
            Trigger, 
            self.emergency_stop
        )
```

### Network Communication

**Secure Communication:**
```python
import ssl
import socket
from cryptography.fernet import Fernet

class SecureChannel:
    """Encrypted network communication."""
    
    def __init__(self, key: bytes):
        self.cipher = Fernet(key)
        self.context = ssl.create_default_context()
    
    def send_encrypted(self, data: bytes, host: str, port: int):
        """Send encrypted data."""
        encrypted = self.cipher.encrypt(data)
        with socket.create_connection((host, port)) as sock:
            with self.context.wrap_socket(sock) as ssock:
                ssock.sendall(encrypted)
```

## Error Handling

### Exception Hierarchy

```python
class RoboticsException(Exception):
    """Base exception for robotics errors."""
    pass

class HardwareException(RoboticsException):
    """Hardware-related errors."""
    pass

class SafetyException(RoboticsException):
    """Safety-critical errors."""
    pass

class CommunicationException(RoboticsException):
    """Communication errors."""
    pass
```

### Error Recovery

```python
def safe_execute(func: Callable, max_retries: int = 3) -> Any:
    """Execute with automatic retry and recovery."""
    for attempt in range(max_retries):
        try:
            return func()
        except HardwareException as e:
            logger.warning(f"Attempt {attempt+1} failed: {e}")
            if attempt == max_retries - 1:
                emergency_stop()
                raise
            time.sleep(1)
```

## Testing Strategy

### Unit Testing

```python
import pytest
from robot_control import ServoController

class TestServoController:
    """Test servo control functionality."""
    
    @pytest.fixture
    def controller(self):
        return ServoController(mock=True)
    
    def test_angle_validation(self, controller):
        """Test angle range validation."""
        assert controller.set_angle(0, 90)
        assert not controller.set_angle(0, 200)
    
    def test_emergency_stop(self, controller):
        """Test emergency stop."""
        controller.set_angle(0, 90)
        controller.emergency_stop()
        assert controller.state == SystemState.EMERGENCY_STOP
```

### Integration Testing

```python
def test_vision_control_integration():
    """Test vision system integration with control."""
    camera = Camera()
    detector = ObjectDetector()
    controller = ServoController()
    
    # Capture and process
    image = camera.capture()
    objects = detector.detect(image)
    
    # Control based on detection
    if objects:
        target = objects[0]
        controller.move_to_target(target.position)
        
    assert controller.position_reached()
```

## Performance Optimization

### Multithreading

```python
from threading import Thread
from queue import Queue

class VisionPipeline:
    """Multi-threaded vision processing."""
    
    def __init__(self):
        self.frame_queue = Queue(maxsize=10)
        self.result_queue = Queue(maxsize=10)
        
        self.capture_thread = Thread(target=self._capture_loop)
        self.process_thread = Thread(target=self._process_loop)
    
    def start(self):
        self.capture_thread.start()
        self.process_thread.start()
    
    def _capture_loop(self):
        while self.running:
            frame = self.camera.read()
            self.frame_queue.put(frame)
    
    def _process_loop(self):
        while self.running:
            frame = self.frame_queue.get()
            result = self.detector.detect(frame)
            self.result_queue.put(result)
```

## Security Considerations

### Access Control

```python
from functools import wraps

def require_authorization(level: int):
    """Decorator for access control."""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if not check_authorization(level):
                raise SecurityException("Unauthorized access")
            return func(*args, **kwargs)
        return wrapper
    return decorator

@require_authorization(level=ADMIN)
def emergency_shutdown():
    """Controlled system shutdown."""
    pass
```

### Audit Logging

```python
import logging
from datetime import datetime

class AuditLogger:
    """Security audit logging."""
    
    def __init__(self):
        self.logger = logging.getLogger('audit')
        self.logger.setLevel(logging.INFO)
    
    def log_access(self, user: str, resource: str, action: str):
        """Log access attempt."""
        self.logger.info(
            f"{datetime.now()} | User: {user} | "
            f"Resource: {resource} | Action: {action}"
        )
```

## Documentation Standards

### Code Documentation

```python
def calculate_trajectory(
    start: np.ndarray,
    end: np.ndarray,
    duration: float,
    constraints: Optional[Dict] = None
) -> List[np.ndarray]:
    """
    Calculate trajectory between two points.
    
    Args:
        start: Starting position [x, y, z]
        end: Ending position [x, y, z]
        duration: Total time for trajectory (seconds)
        constraints: Optional movement constraints
        
    Returns:
        List of waypoints along trajectory
        
    Raises:
        ValueError: If positions are invalid
        SafetyException: If trajectory violates constraints
        
    Example:
        >>> start = np.array([0, 0, 0])
        >>> end = np.array([100, 100, 100])
        >>> trajectory = calculate_trajectory(start, end, 5.0)
    """
    pass
```

## Future Enhancements

- Machine learning model integration
- Advanced path planning algorithms
- Multi-robot coordination
- Simulation environment integration
- Web-based monitoring dashboard
- Cloud connectivity and remote operation

## References

- ROS Architecture: http://wiki.ros.org/ROS/Concepts
- OpenCV Documentation: https://docs.opencv.org/
- Python Design Patterns: https://refactoring.guru/design-patterns
- Robotics System Architecture: Research papers and textbooks
