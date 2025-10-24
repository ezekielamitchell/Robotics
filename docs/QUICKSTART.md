# Quick Start Guide

This guide will help you get started with the Robotics R&D repository quickly.

## Prerequisites

Before you begin, ensure you have:
- Python 3.8 or higher installed
- Git installed
- pip package manager
- (Optional) Docker for containerized deployments
- (For hardware projects) Raspberry Pi or compatible hardware

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/ezekielamitchell/Robotics.git
cd Robotics
```

### 2. Set Up Virtual Environment

It's recommended to use a virtual environment to avoid dependency conflicts:

```bash
# Create virtual environment
python3 -m venv env

# Activate virtual environment
# On Linux/Mac:
source env/bin/activate

# On Windows:
env\Scripts\activate
```

### 3. Install Core Dependencies

```bash
# Install core robotics dependencies
pip install -r requirements.txt
```

### 4. Install Project-Specific Dependencies

Depending on which project you're working with:

**For OpenCV Learning Lab:**
```bash
cd Lab/OpenCV_Robotics
pip install -r requirements.txt
```

**For ResponsiveArm (Raspberry Pi):**
```bash
cd Projects/ResponsiveArm.pi
pip install -r requirements.txt
```

**For ThreatDetection:**
```bash
cd Projects/ThreatDetection.IMX477/python
pip install -r requirements.txt
```

## Quick Tests

### Test OpenCV Installation

```bash
python -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
```

### Test Basic Image Operations

```bash
cd Lab/OpenCV_Robotics
python -c "import cv2; import numpy as np; print('OpenCV and NumPy working!')"
```

## Project-Specific Setup

### OpenCV Robotics Lab

The OpenCV lab is designed for learning computer vision:

```bash
cd Lab/OpenCV_Robotics
python camera.py  # Test camera connection
python images.py  # Test image processing
```

### ResponsiveArm.pi

For hardware setup:

1. **Enable I2C on Raspberry Pi:**
```bash
sudo raspi-config
# Navigate to: Interface Options > I2C > Enable
```

2. **Connect Hardware:**
   - Connect PCA9685 to Raspberry Pi I2C pins
   - Connect servos to PCA9685 channels
   - Ensure proper power supply

3. **Verify I2C Connection:**
```bash
i2cdetect -y 1
# Should show device at address 0x40
```

4. **Test Basic Movement:**
```bash
cd Projects/ResponsiveArm.pi
python src/main.py
```

### ThreatDetection.IMX477

For camera setup:

1. **Enable Camera Interface:**
```bash
sudo raspi-config
# Navigate to: Interface Options > Camera > Enable
```

2. **Verify Camera:**
```bash
libcamera-hello --list-cameras
```

3. **Run Detection System:**
```bash
cd Projects/ThreatDetection.IMX477/python
# Configure settings first, then:
python main.py  # (when main.py is implemented)
```

## Docker Setup (Optional)

For projects with Docker support:

### ResponsiveArm.pi

```bash
cd Projects/ResponsiveArm.pi
docker build -t responsive-arm .
docker run --privileged -v /dev:/dev responsive-arm
```

### ThreatDetection.IMX477

```bash
cd Projects/ThreatDetection.IMX477/python
docker build -t threat-detection .
docker run --privileged -v /dev:/dev -v /sys:/sys threat-detection
```

## ROS Integration (Optional)

If you need ROS (Robot Operating System):

### Ubuntu/Debian

```bash
# Install ROS (example for ROS Noetic on Ubuntu 20.04)
# Full instructions: http://wiki.ros.org/noetic/Installation

# Quick install for key packages:
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Initialize ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## Troubleshooting

### Python Version Issues

Ensure you're using Python 3.8+:
```bash
python --version
# or
python3 --version
```

### Permission Issues on Raspberry Pi

For GPIO/I2C access:
```bash
sudo usermod -a -G i2c,spi,gpio $USER
# Log out and back in for changes to take effect
```

### OpenCV Import Errors

If OpenCV fails to import:
```bash
pip uninstall opencv-python opencv-contrib-python
pip install opencv-python opencv-contrib-python
```

### Docker Permission Issues

If Docker commands fail:
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

## Next Steps

1. **Read the Documentation:**
   - Review [README.md](../README.md) for project overview
   - Check [CONTRIBUTING.md](../CONTRIBUTING.md) for development guidelines
   - Review [Safety Guidelines](docs/safety/safety-guidelines.md)

2. **Explore Projects:**
   - Start with Lab/OpenCV_Robotics for learning
   - Review project-specific READMEs
   - Run example code and tests

3. **Hardware Setup:**
   - Follow hardware-specific guides in project directories
   - Test with minimal configurations first
   - Implement safety mechanisms

4. **Development:**
   - Set up your IDE with Python linting
   - Configure git for commits
   - Review coding standards in CONTRIBUTING.md

## Getting Help

- Check project-specific README files
- Review troubleshooting sections
- Open an issue on GitHub
- Consult ROS/OpenCV documentation for specific functions

## Safety Reminder

When working with physical hardware:
- Start with low power/slow movements
- Always have emergency stop accessible
- Never leave autonomous systems unattended
- Follow all safety guidelines in documentation

Happy coding! 🤖
