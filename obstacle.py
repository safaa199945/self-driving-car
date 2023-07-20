import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define GPIO pins
ENA = 32
ENB = 33
IN1 = 3
IN2 = 22
IN3 = 5
IN4 = 12
TRIG = 7
ECHO = 11

# Set up GPIO pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Set motor speeds
pwm_left = GPIO.PWM(ENA, 100)
pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0)
pwm_right.start(0)

# Function to move the robot forward
def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Function to stop the robot
def stop_robot():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Function to calculate distance from the obstacle
def measure_distance():
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
  
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
  
    return distance

# Main program
try:
    while True:
        distance = measure_distance()
        print("Distance:", distance, "cm")
        
        if distance > 20:
            move_forward()
            pwm_left.ChangeDutyCycle(50)
            pwm_right.ChangeDutyCycle(50)
        else:
            stop_robot()
            while distance <= 20:
                time.sleep(0.1)
                distance = measure_distance()
                print("Waiting for obstacle to be removed...")
                
            move_forward()
            pwm_left.ChangeDutyCycle(50)
            pwm_right.ChangeDutyCycle(50)
  
except KeyboardInterrupt:
    GPIO.cleanup()
