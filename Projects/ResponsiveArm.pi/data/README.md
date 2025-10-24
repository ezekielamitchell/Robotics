# ResponsiveArm Data Directory

This directory contains configuration data, calibration files, and camera configurations for the ResponsiveArm robotic system.

## Structure

```
data/
├── camera_config/        # Camera calibration images and parameters
│   ├── camera_config1.jpg
│   ├── camera_config2.jpg
│   └── camera_config3.jpg
├── servo_calibration/    # Servo calibration data (to be added)
└── trajectories/         # Saved movement trajectories (to be added)
```

## Camera Configuration

The `camera_config/` directory contains images used for camera calibration. These images are typically:
- Checkerboard patterns at various angles
- Used for intrinsic camera parameter estimation
- Required for vision-guided manipulation tasks

### Using Camera Calibration

```python
# Example: Load camera calibration
import yaml
import numpy as np

with open('data/camera_config/calibration.yaml', 'r') as f:
    calib = yaml.safe_load(f)
    
camera_matrix = np.array(calib['camera_matrix'])
dist_coeffs = np.array(calib['distortion_coefficients'])
```

## Servo Calibration

Store servo calibration data to ensure consistent and accurate movements:

```yaml
# Example: servo_calibration.yaml
servos:
  - channel: 0
    min_pulse: 500
    max_pulse: 2500
    min_angle: 0
    max_angle: 180
    center_offset: 0
  - channel: 4
    min_pulse: 500
    max_pulse: 2500
    min_angle: 0
    max_angle: 180
    center_offset: 0
```

## Movement Trajectories

Save and replay movement sequences:

```python
# Example: Save trajectory
trajectory = {
    'name': 'pick_and_place',
    'waypoints': [
        {'time': 0.0, 'positions': [90, 45, 120, 60]},
        {'time': 1.0, 'positions': [110, 60, 100, 80]},
        {'time': 2.0, 'positions': [90, 45, 120, 60]},
    ]
}

import yaml
with open('data/trajectories/pick_and_place.yaml', 'w') as f:
    yaml.dump(trajectory, f)
```

## Best Practices

1. **Version Control**: Commit calibration configurations but not large datasets
2. **Backup**: Regularly backup calibration data
3. **Documentation**: Document calibration procedures and dates
4. **Validation**: Verify calibration accuracy after loading
5. **Safety**: Store safe home positions and limits

## File Formats

- **YAML**: Configuration and calibration parameters
- **JSON**: Alternative for structured data
- **NPY**: NumPy arrays for efficient storage
- **JPG/PNG**: Calibration images

## Maintenance

- Recalibrate camera after any physical changes
- Update servo calibration if you notice drift
- Clean up old trajectory files periodically
- Document any hardware modifications

## Security Note

Do not commit:
- Sensitive operational data
- Proprietary trajectories
- System-specific credentials
- Mission-critical configurations
