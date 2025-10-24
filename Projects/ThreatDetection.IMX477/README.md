# ThreatDetection.IMX477 - High-Resolution Surveillance System

A threat detection and surveillance system utilizing the IMX477 12.3MP camera sensor for defense and security applications.

## Overview

This project implements a high-resolution imaging and threat detection system designed for military, defense, and security applications. The system leverages the Sony IMX477 camera sensor's capabilities for long-range surveillance, object detection, and threat assessment.

## Features

- **High-Resolution Imaging**: 12.3MP (4056 x 3040) resolution
- **Computer Vision Processing**: Real-time object detection and classification
- **Edge Computing**: Dockerized deployment for field operations
- **Low-Light Performance**: Enhanced sensitivity for night operations
- **Modular Architecture**: Easy integration with existing security systems
- **Secure Communication**: Encrypted data transmission capabilities

## Hardware Requirements

### Required Components
- Raspberry Pi 4 (4GB+ RAM recommended)
- Sony IMX477 Camera Module (HQ Camera)
- C/CS-mount lens (focal length depends on application)
- High-speed microSD card (32GB+ recommended)
- Reliable power supply (5V/3A minimum)

### Optional Components
- PoE HAT for network power
- Weatherproof enclosure for outdoor deployment
- IR illuminator for night vision
- Pan-tilt mechanism for area coverage
- Backup battery for continuous operation

## Software Dependencies

Install required packages:
```bash
cd Projects/ThreatDetection.IMX477/python
pip install -r requirements.txt
```

Key dependencies:
- OpenCV for computer vision processing
- NumPy for numerical operations
- ROS for system integration
- TensorFlow/PyTorch for ML-based detection (if enabled)

## Installation

1. **Camera Setup**:
   ```bash
   # Enable camera interface
   sudo raspi-config
   # Navigate to Interface Options > Camera > Enable
   
   # Verify camera connection
   libcamera-hello --list-cameras
   ```

2. **Software Installation**:
   ```bash
   cd Projects/ThreatDetection.IMX477/python
   pip install -r requirements.txt
   ```

3. **Configuration**:
   - Configure camera parameters (resolution, framerate, exposure)
   - Set detection thresholds and alert parameters
   - Configure network settings for alert transmission
   - Set up secure communication protocols

## Usage

### Docker Deployment (Recommended)

Build and run the containerized system:
```bash
cd Projects/ThreatDetection.IMX477/python
docker build -t threat-detection-imx477 .
docker run --privileged -v /dev:/dev -v /sys:/sys threat-detection-imx477
```

### Standalone Operation

For development and testing:
```bash
python main.py --config config.yaml
```

## Configuration

### Camera Settings

Optimal settings for different scenarios:

**Daytime Surveillance** (long-range):
- Resolution: 4056x3040
- Framerate: 10 fps
- Exposure: Auto
- Lens: 16mm or longer focal length

**Night Operations**:
- Resolution: 1920x1080 (better low-light performance)
- Framerate: 15 fps
- Exposure: Extended
- IR illuminator: Enabled

**Rapid Detection** (close-range):
- Resolution: 1920x1080
- Framerate: 30 fps
- Exposure: Auto
- Lens: 6mm focal length

### Detection Parameters

Configure in `config.yaml`:
```yaml
detection:
  confidence_threshold: 0.75
  iou_threshold: 0.45
  classes_of_interest: ["person", "vehicle", "weapon"]
  alert_on_detection: true
  
alerts:
  method: "network"  # Options: network, local, both
  endpoints: ["https://alert.example.mil"]
  encryption: true
```

## Security Considerations

### Defense Applications

**CRITICAL**: This system may be subject to export controls and security regulations:

1. **ITAR/EAR Compliance**: Verify compliance before international deployment
2. **Secure Communications**: Use encrypted channels for all data transmission
3. **Access Control**: Implement strong authentication for system access
4. **Audit Logging**: Maintain comprehensive logs of all operations
5. **Data Retention**: Follow organizational policies for recorded data
6. **Fail-Safe Mechanisms**: Implement automatic secure shutdown on compromise

### Data Security

- **Encryption at Rest**: Encrypt stored imagery and detection data
- **Encryption in Transit**: Use TLS 1.3+ for network communications
- **Access Logs**: Track all access to detection data
- **Secure Deletion**: Properly sanitize data when no longer needed

### Physical Security

- Tamper-evident enclosures for field deployments
- Secure mounting to prevent unauthorized removal
- Environmental protection for harsh conditions
- Backup power for continuous operation

## System Architecture

```
IMX477 Camera → Image Capture → Processing Pipeline → Detection → Alert System
                                        ↓
                                  Data Logging
                                        ↓
                                Secure Storage
```

### Processing Pipeline

1. **Image Acquisition**: Capture from IMX477 at configured resolution
2. **Preprocessing**: Noise reduction, exposure correction
3. **Detection**: Run object detection models
4. **Classification**: Identify threats vs. non-threats
5. **Alert Generation**: Trigger alerts based on classification
6. **Logging**: Record events with timestamps and metadata

## Performance Optimization

### Hardware Acceleration

- Enable GPU acceleration for OpenCV operations
- Use hardware-accelerated video encoding
- Optimize memory usage for edge deployment

### Algorithm Optimization

- Use lighter detection models for real-time operation
- Implement region-of-interest processing
- Cache common computations
- Adjust resolution based on detection distance

## Troubleshooting

### Camera Not Detected
```bash
# Check camera connection
vcgencmd get_camera

# List available cameras
libcamera-hello --list-cameras

# Check cable seating and ribbon cable orientation
```

### Poor Detection Performance
- Verify adequate lighting conditions
- Check focus and lens cleanliness
- Adjust detection thresholds
- Validate model compatibility
- Monitor system temperature and throttling

### Network Issues
- Verify network connectivity
- Check firewall rules
- Validate encryption certificates
- Test alert endpoints manually

## Testing and Validation

### Unit Testing
```bash
pytest tests/
```

### Integration Testing
- Test with known objects at various distances
- Validate alert triggering and transmission
- Verify logging and data retention
- Test failover and recovery mechanisms

### Field Testing
- Deploy in representative environments
- Test under various lighting conditions
- Validate detection at maximum operational range
- Verify system reliability over extended periods

## Deployment Guidelines

### Site Survey
1. Identify optimal camera placement
2. Verify network coverage
3. Assess power requirements
4. Plan maintenance access

### Installation
1. Mount hardware securely
2. Optimize camera orientation and focus
3. Configure network connectivity
4. Test detection coverage
5. Document installation parameters

### Maintenance
- Regular lens cleaning
- Firmware updates
- Log review and analysis
- Backup configuration
- Test alert systems periodically

## Ethical and Legal Considerations

- Ensure deployment complies with local privacy laws
- Post appropriate surveillance notices as required
- Maintain data retention policies
- Respect civil liberties and human rights
- Document authorization for deployment
- Follow organizational ethics guidelines

## Integration with Other Systems

This system can integrate with:
- Security Operations Centers (SOC)
- Command and Control (C2) systems
- Geographic Information Systems (GIS)
- Alert notification systems
- Other robotic platforms (ResponsiveArm, autonomous vehicles)

## Performance Metrics

Expected performance (may vary by configuration):
- Detection latency: <200ms
- Alert propagation: <500ms
- Continuous operation: 24/7 with proper cooling
- Detection range: Depends on lens and conditions
- False positive rate: <5% with proper tuning

## Contributing

Due to the sensitive nature of defense applications, contributions must:
1. Be reviewed for security implications
2. Comply with export control regulations
3. Maintain documentation standards
4. Include comprehensive testing

## Support

For technical support, security concerns, or deployment assistance, contact the repository maintainers through secure channels.

## License

See main repository LICENSE file. Additional restrictions may apply for defense-related deployments.

## Disclaimer

This software is provided for legitimate defense and security applications only. Users are responsible for ensuring legal and ethical use in accordance with all applicable laws and regulations.
