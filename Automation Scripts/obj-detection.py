import cv2
import numpy as np

# --- Configuration ---
# Define the IDs for the boundary markers (use a set for efficient checking)
BOUNDARY_MARKER_IDS = {1, 2, 3, 4}
# Define the ID for the object marker
OBJECT_MARKER_ID = 0 # CHANGE THIS if your object marker has a different ID

# Choose the ArUco dictionary
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
# --- End Configuration ---

# Initialize the webcam
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Cannot open webcam.")
    exit()

# Load the predefined ArUco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)

# Create detector parameters
# Use DetectorParameters() for newer OpenCV versions (>= 4.7.0)
# Use DetectorParameters_create() for older versions
try:
    arucoParams = cv2.aruco.DetectorParameters()
    # Example: Set a parameter if needed
    # arucoParams.adaptiveThreshConstant = 7
    print("Using cv2.aruco.DetectorParameters()")
except AttributeError:
    arucoParams = cv2.aruco.DetectorParameters_create()
    print("Using cv2.aruco.DetectorParameters_create()")


# Initialize the detector (for newer OpenCV versions >= 4.7.0)
# Older versions don't use a separate detector object for detectMarkers
try:
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    print("Using cv2.aruco.ArucoDetector()")
    use_detector_object = True
except AttributeError:
    detector = None
    use_detector_object = False
    print("Using cv2.aruco.detectMarkers() directly.")


while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Detect ArUco markers in the frame
    if use_detector_object:
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # --- Process Detections ---
    boundary_corners_list = []
    detected_boundary_ids = set()
    object_marker_corners = None
    object_detected = False

    if ids is not None:
        # Flatten the ids array for easier iteration
        flat_ids = ids.flatten()

        # Iterate through detected markers to find boundary and object markers
        for i, marker_id in enumerate(flat_ids):
            if marker_id in BOUNDARY_MARKER_IDS:
                detected_boundary_ids.add(marker_id)
                boundary_corners_list.append(corners[i]) # Store corners for this boundary marker
            elif marker_id == OBJECT_MARKER_ID:
                object_marker_corners = corners[i] # Store corners for the object marker
                object_detected = True

        # Check if ALL required boundary markers were detected
        if len(detected_boundary_ids) == len(BOUNDARY_MARKER_IDS):
            # Concatenate corners ONLY from the detected boundary markers
            all_boundary_corners = np.concatenate(boundary_corners_list, axis=1).reshape(-1, 2)

            # Find the convex hull of the boundary marker corners
            hull = cv2.convexHull(all_boundary_corners.astype(np.float32))
            hull = hull.reshape(-1, 1, 2).astype(int) # Reshape for polylines function

            # Draw the boundary polygon (convex hull) in blue color
            cv2.polylines(frame, [hull], isClosed=True, color=(255, 0, 0), thickness=2)

        # Draw outlines and IDs for ALL detected markers (boundary and object)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Optional: Add specific visual cue for the object marker if detected
        if object_detected:
            # Example: Draw a green circle at the center of the object marker
            if object_marker_corners is not None:
                 center_x = int(np.mean(object_marker_corners[0, :, 0]))
                 center_y = int(np.mean(object_marker_corners[0, :, 1]))
                 cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1) # Green dot

    # --- Display ---
    # Show the frame with detections and boundary
    cv2.imshow('ArUco Detection and Boundary', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
# Release the webcam and destroy all OpenCV windows
cap.release()
cv2.destroyAllWindows()
