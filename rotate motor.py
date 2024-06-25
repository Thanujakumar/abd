import RPi.GPIO as GPIO
import time

# Pin definitions
IN1 = 17  # GPIO pin for IN1 on L298N
IN2 = 18  # GPIO pin for IN2 on L298N
IN3 = 22  # GPIO pin for IN3 on L298N
IN4 = 23  # GPIO pin for IN4 on L298N
ENA = 24  # GPIO pin for ENA on L298N
ENB = 25  # GPIO pin for ENB on L298N

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM on ENA and ENB
pwmA = GPIO.PWM(ENA, 100)  # PWM frequency of 100 Hz
pwmB = GPIO.PWM(ENB, 100)  # PWM frequency of 100 Hz

# Start PWM with 50% duty cycle
pwmA.start(50)
pwmB.start(50)

def rotate_right():
    print("Rotating right")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def rotate_left():
    print("Rotating left")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def stop():
    print("Stopping")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    while True:
        command = input("Enter command (r for right, l for left, s for stop, q to quit): ")
        if command == 'r':
            rotate_right()
        elif command == 'l':
            rotate_left()
        elif command == 's':
            stop()
        elif command == 'q':
            break
        else:
            print("Invalid command")

except KeyboardInterrupt:
    pass

finally:
    # Clean up GPIO settings
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
