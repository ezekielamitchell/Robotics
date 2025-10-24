# ResponsiveArm.pi - Robotic Arm Control System

A responsive robotic arm control system designed for Raspberry Pi with ROS integration and multi-channel servo control.

## Overview

This project provides a complete software stack for controlling a multi-degree-of-freedom robotic arm using a Raspberry Pi and PCA9685 PWM controller. The system is designed for research, development, and testing of autonomous manipulation tasks.

## Features

- **Multi-Channel Control**: 16-channel PWM control via PCA9685
- **ROS Integration**: Compatible with ROS ecosystem for distributed robotics
- **Precise Servo Control**: Configurable pulse width and angle ranges
- **Initialization Routines**: Safe startup and calibration sequences
- **Modular Architecture**: Easy to extend and customize
- **Docker Support**: Containerized deployment for consistent environments

## Hardware Requirements

### Required Components
- Raspberry Pi (3B+ or 4 recommended)
- PCA9685 16-Channel 12-bit PWM/Servo Driver
- Compatible servo motors (up to 16 channels)
- 5V/2A+ power supply for Raspberry Pi
- External power supply for servos (voltage depends on servo specs)

### Optional Components
- Cooling fan for extended operation
- Mounting hardware for robotic arm structure

## Software Dependencies

Install required packages:
```bash
pip install -r requirements.txt
```

Key dependencies:
- `adafruit-circuitpython-servokit`: Servo control library
- `rospy`: ROS Python client library
- `RPi.GPIO`: Raspberry Pi GPIO control

## Installation

1. **Hardware Setup**:
   - Connect PCA9685 to Raspberry Pi I2C pins
   - Connect servos to PCA9685 channels
   - Ensure proper power distribution to servos
   - Verify I2C communication: `i2cdetect -y 1`

2. **Software Setup**:
   ```bash
   cd Projects/ResponsiveArm.pi
   pip install -r requirements.txt
   ```

3. **Configuration**:
   - Edit servo parameters in `src/main.py`:
     - `MIN_IMP`: Minimum pulse width (microseconds)
     - `MAX_IMP`: Maximum pulse width (microseconds)
     - `MIN_ANG`: Minimum angle (degrees)
     - `MAX_ANG`: Maximum angle (degrees)

## Usage

### Basic Operation

Run the initialization sequence:
```bash
python src/main.py
```

This will:
1. Initialize ROS node
2. Center the arm servos
3. Run calibration movements
4. Return to neutral position

### ROS Integration

The system initializes a ROS node named `responsive_arm`. To integrate with other ROS nodes:

```python
import rospy
from src.main import pca

# Your ROS control code here
```

### Testing

Test individual components:
```bash
# Test fan control
python test/fan.py

# Test single channel
python test/simple_channel1.py
```

## Configuration

### Servo Calibration

Each servo channel can be individually configured in `src/main.py`:

```python
MIN_IMP  = [500, 500, 500, ...]  # Min pulse width per channel
MAX_IMP  = [2500, 2500, 2500, ...] # Max pulse width per channel
MIN_ANG  = [0, 0, 0, ...]         # Min angle per channel
MAX_ANG  = [180, 180, 180, ...]   # Max angle per channel
```

### Channel Mapping

Current default mapping:
- Channel 0, 4: Main arm movement (synchronized)
- Channel 5: Arm joint
- Channel 8: Arm joint
- Channel 9: Base rotation

Modify the `init()` function to customize for your arm configuration.

## Docker Deployment

Build and run in container:
```bash
docker build -t responsive-arm .
docker run --privileged -v /dev:/dev responsive-arm
```

Note: `--privileged` flag required for I2C access.

## Safety Guidelines

### Before Operation
1. Ensure adequate clearance around arm workspace
2. Verify all mechanical connections are secure
3. Test emergency stop procedures
4. Start with slow movements and gradually increase speed

### During Operation
1. Never put hands or objects in arm workspace during operation
2. Monitor servo temperatures to avoid overheating
3. Keep emergency stop mechanism accessible
4. Supervise autonomous operation at all times

### Emergency Stop
To immediately stop all servos:
```python
for i in range(16):
    pca.servo[i].angle = None
```

## Troubleshooting

### I2C Communication Issues
```bash
# Enable I2C on Raspberry Pi
sudo raspi-config
# Navigate to Interface Options > I2C > Enable

# Check I2C devices
i2cdetect -y 1
# PCA9685 should appear at address 0x40
```

### Servo Jitter or Erratic Movement
- Check power supply is adequate for all servos
- Verify pulse width settings match servo specifications
- Add capacitors to servo power lines if needed
- Reduce movement speed in code

### ROS Node Issues
```bash
# Check ROS master is running
rostopic list

# View node info
rosnode info responsive_arm
```

## Development

### File Structure
```
ResponsiveArm.pi/
├── src/
│   ├── main.py          # Main control logic
│   └── ros.py           # ROS integration (placeholder)
├── test/
│   ├── fan.py           # Fan control test
│   └── simple_channel1.py  # Single channel test
├── data/
│   └── camera_config/   # Camera calibration data
├── Dockerfile           # Container configuration
└── requirements.txt     # Python dependencies
```

### Adding New Features

To add new movement routines:
1. Add function to `src/main.py`
2. Follow existing patterns for servo control
3. Include safety checks and position validation
4. Test incrementally with reduced speed

## Performance Optimization

- Use threading for concurrent servo control
- Implement smooth trajectory planning
- Add position feedback if encoders available
- Cache servo positions to minimize I2C traffic

## Related Projects

- ROS Moveit: Motion planning framework
- OpenCV Robotics: Vision-guided manipulation
- ThreatDetection: Surveillance integration

## Contributing

Contributions welcome! Please test thoroughly on physical hardware before submitting pull requests.

## License

See main repository LICENSE file.
