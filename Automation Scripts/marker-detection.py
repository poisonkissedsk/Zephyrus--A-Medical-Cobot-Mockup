import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(1)

# Load the predefined ArUco dictionary using the correct function [5]
# Ensure the dictionary matches the markers you are using.
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # <-- Fix applied here
# Use DetectorParameters() for newer OpenCV versions. Older versions might use DetectorParameters_create()
# Let's assume newer version based on the error hinting at newer structures.
try:
    # OpenCV 4.7.0 and later
    arucoParams = cv2.aruco.DetectorParameters()
    # If you need to set parameters, do it like this:
    # arucoParams.adaptiveThreshConstant = 7
except AttributeError:
    # Older OpenCV versions
    arucoParams = cv2.aruco.DetectorParameters_create()


# Initialize the detector using the dictionary and parameters
# For OpenCV 4.7.0 and later:
try:
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
except AttributeError:
    # For older versions, detection is done directly with detectMarkers function
    detector = None # No separate detector object needed


while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Detect ArUco markers in the frame
    if detector:
        # Newer OpenCV (4.7.0+) style
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        # Older OpenCV style
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)


    # Check if exactly four markers were detected
    if ids is not None and len(ids) == 4:
        # Extract the corner points of all detected markers into a single list
        all_corners = np.concatenate(corners, axis=1).reshape(-1, 2)

        # Find the convex hull of all corner points
        hull = cv2.convexHull(all_corners.astype(np.float32))
        hull = hull.reshape(-1, 1, 2).astype(int) # Reshape for polylines function

        # Draw the boundary polygon (convex hull) in blue color
        cv2.polylines(frame, [hull], isClosed=True, color=(255, 0, 0), thickness=2)

    # Optionally, draw the detected markers themselves for visualization
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Show the frame with detections and boundary
    cv2.imshow('ArUco Detection and Boundary', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all OpenCV windows
cap.release()
cv2.destroyAllWindows()
