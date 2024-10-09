import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set pin 18 as output
fan_pin = 18
GPIO.setup(fan_pin, GPIO.OUT)

try:
    while True:
        # Turn the fan on
        GPIO.output(fan_pin, GPIO.HIGH)
        print("Fan ON")
        time.sleep(5)  # Keep the fan on for 5 seconds

        # Turn the fan off
        GPIO.output(fan_pin, GPIO.LOW)
        print("Fan OFF")
        time.sleep(5)  # Keep the fan off for 5 seconds

except KeyboardInterrupt:
    # Clean up GPIO on exit
    GPIO.cleanup()