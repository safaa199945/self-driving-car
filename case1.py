import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

# Define GPIO pins
ENA = 32
ENB = 33
IN1 = 3
IN2 = 22
IN3 = 5
IN4 = 12
TRIG = 7
ECHO = 11

# Set up GPIO mode and ultrasonic sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Set up motor control GPIO pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Set up PWM for motor control
motor_left = GPIO.PWM(ENA, 100)
motor_right = GPIO.PWM(ENB, 100)
motor_left.start(0)
motor_right.start(0)

# Function to measure distance using ultrasonic sensor
def measure_distance():
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
    
    pulse_start = time.time()
    pulse_end = time.time()
    
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    return distance

# Lane detection function
def detect_lane(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection to detect edges
    edges = cv2.Canny(blurred, 50, 150)

    # Create a region of interest (ROI) mask
    height, width = edges.shape[:2]
    roi_vertices = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, roi_vertices, 255)

    # Apply the ROI mask to the edge image
    masked_edges = cv2.bitwise_and(edges, mask)

    # Perform Hough line detection
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)

    # Calculate the center of the detected lanes
    lane_center = width // 2
    if lines is not None:
        lane_points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            lane_points.append((x1 + x2) // 2)
        if lane_points:
            lane_center = int(np.mean(lane_points))

    # Draw the lane center line on the frame
    cv2.line(frame, (lane_center, height), (lane_center, height // 2), (0, 0, 255), 2)

    # Determine the deviation of the car from the lane center
    deviation = lane_center - width // 2

    return frame, deviation

# Motor control functions
def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Main program loop
try:
    # Open the video capture (use the appropriate camera index)
    cap = cv2.VideoCapture(0)

    # Set the distance threshold for obstacle detection (in centimeters)
    obstacle_distance_threshold = 20

    while True:
        # Read the current frame from the video capture
        ret, frame = cap.read()

        # Perform lane detection
        modified_frame, deviation = detect_lane(frame)

        # Perform obstacle detection
        obstacle_detected = measure_distance(obstacle_distance_threshold)

        # Control the car's movement based on obstacle detection and lane deviation
        if obstacle_detected:
            stop()
        elif deviation < -10:  # Deviation to the left
            turn_left()
        elif deviation > 10:  # Deviation to the right
            turn_right()
        else:  # Within acceptable deviation range
            move_forward()

        # Display the modified frame
        cv2.imshow("Self-Driving Car", modified_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and clean up GPIO
    cap.release()
    GPIO.cleanup()

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()
