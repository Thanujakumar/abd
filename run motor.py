import RPi.GPIO as GPIO
import time

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO pins connected to the motor driver
IN1 = 17
IN2 = 18
IN3 = 22
IN4 = 23
ENA = 27
ENB = 22

# Set all pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Initialize PWM for motor speed control
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

def set_motor_speed(speedA, speedB):
    pwmA.ChangeDutyCycle(speedA)
    pwmB.ChangeDutyCycle(speedB)

def move_forward(speed=50):
    set_motor_speed(speed, speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def move_backward(speed=50):
    set_motor_speed(speed, speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def turn_left(speed=50):
    set_motor_speed(speed, speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right(speed=50):
    set_motor_speed(speed, speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    set_motor_speed(0, 0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    while True:
        command = input("Enter command (w: forward, s: backward, a: left, d: right, q: quit): ")

        if command == 'w':
            move_forward(75)
        elif command == 's':
            move_backward(75)
        elif command == 'a':
            turn_left(75)
        elif command == 'd':
            turn_right(75)
        elif command == 'q':
            break
        else:
            stop()
        time.sleep(0.1)
        stop()

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
