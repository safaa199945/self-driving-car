#This code is for the second case where we used Socket; We run this code first on our computer and then we run another code in the raspberry pi once the connection is established 
#we see the results

import cv2
import numpy as np
import socket

# Function to detect lanes and calculate deviation
def detect_lane(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection to detect edges
    edges = cv2.Canny(blurred, 50, 150)

    # Create a region of interest (ROI) mask
    height, width = edges.shape[:2]
    roi_vertices = np.array([[(0, height), (width//2, height//2), (width, height)]], dtype=np.int32)
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
    deviation_threshold = 10  # Adjust this threshold as needed
    if abs(lane_center - width // 2) <= deviation_threshold:
        deviation = 0
    else:
        deviation = lane_center - width // 2

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges for traffic lights (red, yellow, and green)
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    green_lower = np.array([60, 100, 100])
    green_upper = np.array([90, 255, 255])

    # Threshold the frame to get binary images of each color
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    # Count the number of non-zero pixels in each binary image
    red_pixels = cv2.countNonZero(red_mask)
    yellow_pixels = cv2.countNonZero(yellow_mask)
    green_pixels = cv2.countNonZero(green_mask)

    # Determine the traffic light status based on the highest pixel count
    if red_pixels > yellow_pixels and red_pixels > green_pixels:
        traffic_light_status = "Stop"
    elif yellow_pixels > red_pixels and yellow_pixels > green_pixels:
        traffic_light_status = "Wait"
    else:
        traffic_light_status = "Go"

    return frame, deviation, traffic_light_status

# Create a socket connection
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '192.168.43.142'  # Replace with the Raspberry Pi IP address
port = 12345  # Choose an available port

try:
    # Connect to the Raspberry Pi server
    client_socket.connect((host, port))

    # Open the video capture (use the appropriate camera index)
    cap = cv2.VideoCapture(0)

    while True:
        # Read the current frame from the video capture
        ret, frame = cap.read()

        # Perform lane detection and traffic light detection
        modified_frame, deviation, traffic_light_status = detect_lane(frame)

        # Display the modified frame
        cv2.imshow("Lane Detection", modified_frame)
        if deviation == 0:
            msg = ("go on")
        elif deviation < 0:
            msg = "Turn left"
        else:
            msg = "Turn right"

        # Print the deviation and traffic light status
        print("Deviation:", deviation)
        print("Status:", msg)

        # Send the deviation and traffic light status to the Raspberry Pi
        data = str(msg) 
        client_socket.sendall(data.encode())
      
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the socket connection
    cap.release()
    client_socket.close()

except KeyboardInterrupt:
    # Clean up on keyboard interrupt
    cv2.destroyAllWindows()
    client_socket.close()
