# Use a base image compatible with Raspberry Pi 5
FROM arm64v8/python:3.9-slim

# Set working directory
WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libi2c-dev \
    && apt-get clean

# Install PCA9685 driver library
RUN pip install adafruit-circuitpython-servokit adafruit-blinka

# Copy application code
COPY . .
# RUN pip install -r requirements.txt
COPY src/ .


# Set command to run application
CMD ["python3", "./main.py"]
