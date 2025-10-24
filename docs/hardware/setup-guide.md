# Hardware Setup Guide

This guide covers the hardware setup for the robotics projects in this repository.

## Table of Contents

- [Raspberry Pi Setup](#raspberry-pi-setup)
- [Camera Systems](#camera-systems)
- [Servo Control](#servo-control)
- [Power Management](#power-management)
- [Safety Considerations](#safety-considerations)

## Raspberry Pi Setup

### Initial Configuration

1. **Install Operating System:**
   - Download Raspberry Pi OS (64-bit recommended)
   - Use Raspberry Pi Imager or balenaEtcher
   - Write to microSD card (32GB+ recommended)

2. **First Boot Configuration:**
   ```bash
   sudo raspi-config
   ```
   
   Enable:
   - SSH (Interface Options > SSH)
   - I2C (Interface Options > I2C)
   - Camera (Interface Options > Camera)
   - Serial (if using serial communication)

3. **Update System:**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

4. **Install Essential Tools:**
   ```bash
   sudo apt install -y \
       python3-pip \
       python3-venv \
       git \
       i2c-tools \
       cmake \
       libopencv-dev \
       python3-opencv
   ```

### Network Configuration

**WiFi Setup:**
```bash
sudo raspi-config
# Navigate to: System Options > Wireless LAN
```

**Static IP (optional):**
```bash
sudo nano /etc/dhcpcd.conf

# Add at end:
interface eth0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8
```

### Performance Optimization

1. **Increase Swap (for compilation):**
   ```bash
   sudo nano /etc/dphys-swapfile
   # Change CONF_SWAPSIZE=100 to CONF_SWAPSIZE=2048
   sudo systemctl restart dphys-swapfile
   ```

2. **GPU Memory (for vision applications):**
   ```bash
   sudo raspi-config
   # Navigate to: Performance Options > GPU Memory
   # Set to 256MB or higher
   ```

## Camera Systems

### Raspberry Pi Camera Module (Standard)

**Hardware Connection:**
1. Power off Raspberry Pi
2. Lift camera connector clip
3. Insert ribbon cable (blue side facing ethernet port)
4. Push connector clip down

**Testing:**
```bash
# Test camera
libcamera-hello --list-cameras

# Capture test image
libcamera-jpeg -o test.jpg

# Verify with Python
python3 -c "from picamera2 import Picamera2; print('Camera OK')"
```

### IMX477 High Quality Camera

**Hardware Setup:**
1. Connect IMX477 module to camera port
2. Attach C/CS mount lens
3. Adjust focus ring while viewing preview
4. Secure lens lock ring

**Lens Selection:**
- **6mm**: Wide angle, close range detection
- **16mm**: Standard field of view
- **25mm+**: Long range surveillance

**Testing:**
```bash
# List camera info
libcamera-hello --list-cameras

# High resolution capture
libcamera-still -r -o test_highres.jpg --width 4056 --height 3040
```

### Camera Calibration

1. **Print Checkerboard Pattern:**
   - 9x6 internal corners
   - Square size: 25mm
   - Print on rigid surface

2. **Capture Calibration Images:**
   ```python
   import cv2
   
   cap = cv2.VideoCapture(0)
   count = 0
   
   while count < 20:
       ret, frame = cap.read()
       cv2.imshow('Capture', frame)
       
       if cv2.waitKey(1) & 0xFF == ord('c'):
           cv2.imwrite(f'calib_{count}.jpg', frame)
           count += 1
   ```

3. **Run Calibration:**
   - Use OpenCV calibration tools
   - Save camera matrix and distortion coefficients

## Servo Control

### PCA9685 PWM Controller

**Hardware Wiring:**
```
PCA9685          Raspberry Pi
VCC      →       5V (Pin 2 or 4)
GND      →       GND (Pin 6)
SDA      →       GPIO 2 (SDA, Pin 3)
SCL      →       GPIO 3 (SCL, Pin 5)
```

**Important Notes:**
- Use separate power supply for servos (5-6V, 2A+)
- Connect servo power supply GND to Raspberry Pi GND
- Do NOT power servos from Raspberry Pi 5V pins

**Verification:**
```bash
# Check I2C connection
sudo i2cdetect -y 1

# Should show device at 0x40:
#     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

### Servo Connections

**Wiring Each Servo:**
```
Servo Wire    PCA9685 Channel
Brown/Black → GND (-)
Red         → V+ (middle)
Orange      → Signal (S)
```

**Channel Assignment Example:**
```
Channel 0: Base rotation
Channel 1: Shoulder
Channel 2: Elbow
Channel 3: Wrist
Channel 4: Gripper
Channels 5-15: Additional servos
```

### Initial Servo Test

```python
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# Test single servo (channel 0)
kit.servo[0].angle = 90  # Center position
time.sleep(1)
kit.servo[0].angle = None  # Disable
```

## Power Management

### Power Requirements

**Raspberry Pi 4:**
- Minimum: 5V/3A
- Recommended: 5V/3.5A
- Use official power supply

**Servos:**
- Standard servo: 1-2A peak per servo
- Calculate: N servos × 2A = required capacity
- Example: 6 servos = 12A supply minimum

**Total System:**
- Raspberry Pi: 3A
- 6 Servos: 12A peak
- Camera: 0.5A
- Total: 15.5A minimum

### Power Supply Setup

1. **Separate Power Rails:**
   - Raspberry Pi: 5V/3A USB-C
   - Servos: 5-6V/15A+ DC supply
   - Common ground connection

2. **Power Distribution:**
   ```
   Main 5V Supply (15A+)
   ├─→ PCA9685 V+ terminal
   ├─→ Servo power rails
   └─→ GND to Raspberry Pi GND
   
   Separate 5V Supply (3A)
   └─→ Raspberry Pi USB-C
   ```

3. **Protection:**
   - Use fuses on main power lines
   - Add capacitors near servos (100-1000µF)
   - Emergency cutoff switch in series

### Battery Operation (Optional)

**For Mobile Applications:**
- LiPo battery: 2S (7.4V) or 3S (11.1V)
- Buck converter: Step down to 5V
- Battery management system (BMS)
- Low voltage cutoff protection

## Safety Considerations

### Electrical Safety

1. **Before Connecting Power:**
   - Verify all connections
   - Check for shorts with multimeter
   - Ensure proper polarity
   - Test with low current first

2. **During Operation:**
   - Monitor temperatures
   - Watch for smoke or unusual smells
   - Have fire extinguisher nearby
   - Never leave unattended

### Mechanical Safety

1. **Servo Installation:**
   - Secure mounting prevents vibration
   - Verify mechanical limits
   - Test range of motion slowly
   - Add soft stops if possible

2. **Workspace:**
   - Clear area of 1m radius minimum
   - Secure base to prevent tipping
   - Mark moving areas clearly
   - Emergency stop accessible

### Testing Protocol

1. **Initial Power-On:**
   - Connect without servos first
   - Verify PCA9685 communication
   - Check voltage levels
   - Monitor for issues

2. **Servo Testing:**
   - Connect one servo at a time
   - Test at low angles (±10°) first
   - Gradually increase range
   - Monitor current draw

3. **System Integration:**
   - Test emergency stop
   - Verify software limits
   - Test under load
   - Document safe parameters

## Troubleshooting

### I2C Issues

```bash
# Check I2C is enabled
sudo raspi-config
# Interface Options > I2C > Enable

# Check for device
sudo i2cdetect -y 1

# If not found, check wiring and power
```

### Camera Not Detected

```bash
# Check camera cable connection
vcgencmd get_camera

# Should show: supported=1 detected=1

# If not, check:
# - Cable orientation (blue side toward ethernet)
# - Camera enabled in raspi-config
# - Try different camera port
```

### Servo Jitter

- Increase capacitor size near servos
- Improve power supply quality
- Check ground connections
- Reduce wire length
- Lower PWM frequency if supported

### Overheating

- Add heatsinks to Raspberry Pi
- Add cooling fan
- Reduce system load
- Check ambient temperature
- Monitor with: `vcgencmd measure_temp`

## Maintenance

### Regular Checks

- **Weekly:**
  - Visual inspection of connections
  - Check for loose wires
  - Test emergency stop
  
- **Monthly:**
  - Clean dust from components
  - Check servo operation
  - Verify calibration
  
- **Quarterly:**
  - Replace worn mechanical parts
  - Update software
  - Full system test

## Additional Resources

- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [Adafruit PCA9685 Guide](https://learn.adafruit.com/16-channel-pwm-servo-driver)
- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [ROS Hardware Interface](http://wiki.ros.org/ros_control)

## Support

For hardware issues:
1. Check troubleshooting section above
2. Verify connections match diagrams
3. Test components individually
4. Consult manufacturer documentation
5. Open issue on GitHub with details
