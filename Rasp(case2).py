#After running the file lanedetection(case2) on the computer we run this one on the raspberry pi, once the connection between the two is established the car would move according to
#the instructions sent from the computer

import RPi.GPIO as GPIO
import socket

# Configure GPIO pins for motor driver
IN1 = 3
IN2 = 22
IN3 = 5
IN4 = 12
ENA = 32
ENB = 33

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

motor_pwm_a = GPIO.PWM(ENA, 100)
motor_pwm_b = GPIO.PWM(ENB, 100)
motor_pwm_a.start(0)
motor_pwm_b.start(0)

# Function to control the motors
def control_motors(left_speed, right_speed):
    # Control left motor
    if left_speed >= 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    motor_pwm_a.ChangeDutyCycle(abs(left_speed))

    # Control right motor
    if right_speed >= 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    motor_pwm_b.ChangeDutyCycle(abs(right_speed))

# Create a socket connection
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = ''  # Empty string to listen on all available interfaces
port = 12345  # The same port as in the computer code

try:
    # Bind the socket to the host and port
    server_socket.bind((host, port))

    # Listen for incoming connections
    server_socket.listen(1)

    print("Waiting for connection...")

    # Accept the client connection
    client_socket, address = server_socket.accept()
    print("Connected to:", address)

    while True:
        # Receive the data from the computer
        data = client_socket.recv(1024).decode()
        if data:
            # Strip leading/trailing whitespaces and convert to lowercase
            msg = data.strip().lower()

            # Control the motors based on the received msg
           
            if msg == "turn right":
                print("Turning right")
                control_motors(30, -30)  # Left motor set to 30% speed, right motor set to -30% speed (opposite direction)
            elif msg == "turn left":
                print("Turning left")
                control_motors(-30, 30)  # Left motor set to -30% speed (opposite direction), right motor set to 30% speed
            else:
                print("Unknown command:", msg)
                control_motors(0, 0)  # Stop the motors for unknown commands

except KeyboardInterrupt:
    # Clean up on keyboard interrupt
    GPIO.cleanup()
    server_socket.close()
