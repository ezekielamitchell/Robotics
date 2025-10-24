# Deployment Guide

This guide covers deployment of robotics systems for production and field operations.

## Table of Contents

- [Pre-Deployment Checklist](#pre-deployment-checklist)
- [Docker Deployment](#docker-deployment)
- [Systemd Services](#systemd-services)
- [Security Hardening](#security-hardening)
- [Monitoring and Logging](#monitoring-and-logging)
- [Backup and Recovery](#backup-and-recovery)

## Pre-Deployment Checklist

### Hardware Validation

- [ ] All hardware connections verified
- [ ] Power supplies tested under load
- [ ] Cable management completed
- [ ] Thermal management verified
- [ ] Emergency stop tested
- [ ] Mechanical limits confirmed
- [ ] Calibration completed and documented

### Software Validation

- [ ] All tests passing
- [ ] Dependencies installed and verified
- [ ] Configuration files reviewed
- [ ] Logging configured
- [ ] Security settings applied
- [ ] Performance benchmarks met
- [ ] Failover mechanisms tested

### Documentation

- [ ] System documentation complete
- [ ] Operating procedures documented
- [ ] Emergency procedures defined
- [ ] Contact information current
- [ ] Maintenance schedule established

## Docker Deployment

### Building Production Images

**ResponsiveArm.pi:**
```dockerfile
# Production Dockerfile
FROM python:3.9-slim-bullseye

# Install system dependencies
RUN apt-get update && apt-get install -y \
    i2c-tools \
    python3-smbus \
    && rm -rf /var/lib/apt/lists/*

# Create app directory
WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/
COPY data/ ./data/

# Non-root user for security
RUN useradd -m -u 1000 robot && \
    chown -R robot:robot /app
USER robot

# Health check
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD python -c "import sys; sys.exit(0)"

# Start application
CMD ["python", "src/main.py"]
```

**Build and Tag:**
```bash
# Build image
docker build -t responsive-arm:1.0.0 .

# Tag for registry
docker tag responsive-arm:1.0.0 registry.example.com/responsive-arm:1.0.0

# Push to registry
docker push registry.example.com/responsive-arm:1.0.0
```

### Docker Compose for Multi-Service

```yaml
# docker-compose.yml
version: '3.8'

services:
  robot-control:
    image: responsive-arm:1.0.0
    container_name: robot_control
    restart: unless-stopped
    privileged: true
    volumes:
      - /dev:/dev
      - ./config:/app/config:ro
      - ./logs:/app/logs
    environment:
      - LOG_LEVEL=INFO
      - CONFIG_PATH=/app/config/production.yaml
    networks:
      - robot_net
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"

  monitoring:
    image: grafana/grafana:latest
    container_name: monitoring
    restart: unless-stopped
    ports:
      - "3000:3000"
    volumes:
      - grafana_data:/var/lib/grafana
    networks:
      - robot_net

volumes:
  grafana_data:

networks:
  robot_net:
    driver: bridge
```

**Deploy:**
```bash
docker-compose up -d
docker-compose logs -f
```

## Systemd Services

### Creating System Service

**Service File: `/etc/systemd/system/responsive-arm.service`**
```ini
[Unit]
Description=ResponsiveArm Robot Control Service
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=robot
Group=robot
WorkingDirectory=/opt/robotics/ResponsiveArm.pi
ExecStart=/opt/robotics/env/bin/python src/main.py
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

# Security settings
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/opt/robotics/logs /opt/robotics/data

# Resource limits
MemoryLimit=512M
CPUQuota=50%

# Environment
Environment="PYTHONUNBUFFERED=1"
Environment="LOG_LEVEL=INFO"

[Install]
WantedBy=multi-user.target
```

**Installation:**
```bash
# Copy service file
sudo cp responsive-arm.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service
sudo systemctl enable responsive-arm.service

# Start service
sudo systemctl start responsive-arm.service

# Check status
sudo systemctl status responsive-arm.service

# View logs
sudo journalctl -u responsive-arm.service -f
```

### Auto-Start on Boot

```bash
# Enable service
sudo systemctl enable responsive-arm.service

# Verify it will start on boot
systemctl is-enabled responsive-arm.service
```

## Security Hardening

### System Hardening

**1. Firewall Configuration:**
```bash
# Install ufw
sudo apt install ufw

# Default policies
sudo ufw default deny incoming
sudo ufw default allow outgoing

# Allow SSH (change port if non-standard)
sudo ufw allow 22/tcp

# Allow ROS communication (if needed)
sudo ufw allow 11311/tcp

# Enable firewall
sudo ufw enable

# Check status
sudo ufw status
```

**2. SSH Hardening:**
```bash
# Edit SSH config
sudo nano /etc/ssh/sshd_config

# Recommended settings:
PermitRootLogin no
PasswordAuthentication no
PubkeyAuthentication yes
X11Forwarding no
MaxAuthTries 3
LoginGraceTime 30

# Restart SSH
sudo systemctl restart ssh
```

**3. User Permissions:**
```bash
# Create robot user
sudo useradd -m -G i2c,spi,gpio,video robot

# Set up sudo with restrictions
sudo visudo
# Add: robot ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart responsive-arm
```

### Application Security

**Configuration File Permissions:**
```bash
# Sensitive configurations
sudo chown robot:robot /opt/robotics/config/
sudo chmod 700 /opt/robotics/config/
sudo chmod 600 /opt/robotics/config/*.yaml

# Logs
sudo chown robot:robot /opt/robotics/logs/
sudo chmod 750 /opt/robotics/logs/
```

**Environment Variables:**
```bash
# Never store secrets in code
# Use environment variables or secret management

# Example: .env file
API_KEY=your_api_key_here
DATABASE_URL=postgresql://user:pass@localhost/db

# Restrict permissions
chmod 600 .env
```

### Network Security

**TLS/SSL Configuration:**
```python
# Use TLS for all network communications
import ssl

context = ssl.create_default_context()
context.check_hostname = True
context.verify_mode = ssl.CERT_REQUIRED

# Load certificates
context.load_verify_locations('/path/to/ca-cert.pem')
context.load_cert_chain('/path/to/client-cert.pem', 
                       '/path/to/client-key.pem')
```

## Monitoring and Logging

### System Monitoring

**Install Monitoring Tools:**
```bash
# Install monitoring packages
sudo apt install prometheus-node-exporter

# Start and enable
sudo systemctl start prometheus-node-exporter
sudo systemctl enable prometheus-node-exporter
```

**Custom Metrics:**
```python
from prometheus_client import Counter, Gauge, Histogram, start_http_server

# Define metrics
operation_counter = Counter('robot_operations_total', 
                           'Total robot operations')
position_gauge = Gauge('robot_position', 
                      'Current robot position',
                      ['axis'])
operation_duration = Histogram('robot_operation_duration_seconds',
                              'Operation duration')

# Start metrics server
start_http_server(8000)

# Use metrics
operation_counter.inc()
position_gauge.labels(axis='x').set(100.5)
```

### Logging Configuration

**Python Logging:**
```python
import logging
import logging.handlers

def setup_logging():
    """Configure production logging."""
    
    # Create logger
    logger = logging.getLogger('robotics')
    logger.setLevel(logging.INFO)
    
    # Console handler
    console = logging.StreamHandler()
    console.setLevel(logging.WARNING)
    console_fmt = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console.setFormatter(console_fmt)
    
    # File handler with rotation
    file_handler = logging.handlers.RotatingFileHandler(
        '/opt/robotics/logs/robot.log',
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.INFO)
    file_fmt = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - '
        '%(filename)s:%(lineno)d - %(message)s'
    )
    file_handler.setFormatter(file_fmt)
    
    # Add handlers
    logger.addHandler(console)
    logger.addHandler(file_handler)
    
    return logger
```

### Log Rotation

**Logrotate Configuration:**
```bash
# Create: /etc/logrotate.d/robotics
/opt/robotics/logs/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0640 robot robot
}
```

## Backup and Recovery

### Backup Strategy

**Configuration Backup:**
```bash
#!/bin/bash
# backup.sh

BACKUP_DIR="/opt/backups/robotics"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create backup directory
mkdir -p "$BACKUP_DIR"

# Backup configurations
tar -czf "$BACKUP_DIR/config_$TIMESTAMP.tar.gz" \
    /opt/robotics/config/ \
    /opt/robotics/data/calibration/

# Backup logs (last 24 hours)
find /opt/robotics/logs -name "*.log" -mtime -1 \
    -exec tar -czf "$BACKUP_DIR/logs_$TIMESTAMP.tar.gz" {} +

# Remove old backups (keep 30 days)
find "$BACKUP_DIR" -name "*.tar.gz" -mtime +30 -delete

echo "Backup completed: $TIMESTAMP"
```

**Automated Backups:**
```bash
# Add to crontab
crontab -e

# Daily backup at 2 AM
0 2 * * * /opt/robotics/scripts/backup.sh
```

### Recovery Procedures

**System Recovery:**
```bash
# 1. Stop service
sudo systemctl stop responsive-arm.service

# 2. Restore configuration
tar -xzf /opt/backups/robotics/config_TIMESTAMP.tar.gz -C /

# 3. Verify configuration
python -c "import yaml; yaml.safe_load(open('/opt/robotics/config/production.yaml'))"

# 4. Restart service
sudo systemctl start responsive-arm.service

# 5. Verify operation
sudo systemctl status responsive-arm.service
```

**Disaster Recovery:**
```bash
# 1. Reinstall OS on new SD card
# 2. Install dependencies
sudo apt update && sudo apt install python3 python3-pip i2c-tools

# 3. Restore application
cd /opt
sudo git clone https://github.com/ezekielamitchell/Robotics.git robotics

# 4. Restore configuration from backup
sudo tar -xzf backup/config_TIMESTAMP.tar.gz -C /opt/robotics/

# 5. Install Python packages
cd /opt/robotics/Projects/ResponsiveArm.pi
pip3 install -r requirements.txt

# 6. Reinstall systemd service
sudo cp responsive-arm.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable responsive-arm.service
sudo systemctl start responsive-arm.service
```

## Performance Tuning

### Raspberry Pi Optimization

```bash
# Increase GPU memory
sudo raspi-config
# Performance Options > GPU Memory > 256

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon

# CPU Governor for performance
echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Python Optimization

```python
# Use compiled libraries
pip install numpy scipy opencv-python  # Pre-compiled wheels

# Enable multiprocessing for CPU-bound tasks
from multiprocessing import Pool

def process_frame(frame):
    # Processing code
    return result

with Pool(processes=4) as pool:
    results = pool.map(process_frame, frames)
```

## Troubleshooting Production Issues

### Common Issues

**Service Won't Start:**
```bash
# Check logs
sudo journalctl -u responsive-arm.service -n 50

# Check permissions
ls -la /opt/robotics/

# Verify dependencies
pip list | grep -E 'adafruit|opencv|numpy'

# Test manually
cd /opt/robotics/Projects/ResponsiveArm.pi
python src/main.py
```

**High CPU/Memory Usage:**
```bash
# Monitor resources
htop

# Check process
ps aux | grep python

# Check logs for errors
tail -f /opt/robotics/logs/robot.log
```

## Maintenance Schedule

### Daily
- Check system logs
- Verify service status
- Monitor resource usage

### Weekly
- Review security logs
- Test emergency stop
- Check backup completion
- Update if needed

### Monthly
- Full system test
- Review and archive logs
- Test recovery procedures
- Update documentation

### Quarterly
- Security audit
- Performance review
- Hardware inspection
- Software updates

## Documentation Requirements

For each deployment, document:
- Hardware configuration
- Software versions
- Network configuration
- Calibration data
- Test results
- Known issues
- Contact information

## Support and Escalation

**Level 1: Local Support**
- Check logs
- Restart service
- Verify hardware connections

**Level 2: Remote Support**
- SSH access for diagnostics
- Review system metrics
- Update configurations

**Level 3: Developer Support**
- Code changes required
- Hardware replacement
- System redesign

## Compliance and Audit

For defense applications:
- Maintain access logs
- Document all changes
- Regular security audits
- Compliance verification
- Incident reporting procedures

---

For additional support or questions about deployment, refer to the main documentation or contact the development team.
