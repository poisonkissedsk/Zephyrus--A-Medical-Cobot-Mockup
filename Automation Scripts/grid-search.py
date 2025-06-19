import cv2
import numpy as np
import math # For ceiling function

# --- Configuration ---
# Define the IDs and their role for the boundary corners
# IMPORTANT: Update these IDs and their corner positions (TL, TR, BR, BL)
# based on YOUR physical marker setup. The example uses IDs from the image provided.
BOUNDARY_MARKER_IDS_MAP = {
    1: 'TL', # Top-Left corner marker ID
    2: 'TR', # Top-Right corner marker ID
    3: 'BR', # Bottom-Right corner marker ID
    4: 'BL'  # Bottom-Left corner marker ID
}
BOUNDARY_MARKER_IDS = set(BOUNDARY_MARKER_IDS_MAP.keys())

# Define the ID for the object marker (ID=0 in the example image)
OBJECT_MARKER_ID = 0 # CHANGE THIS if your object marker has a different ID

# Grid dimensions (rows, columns) based on the user image
GRID_ROWS = 5
GRID_COLS = 5

# Target size for the flattened perspective-corrected grid view (in pixels)
FLAT_GRID_WIDTH = 500
FLAT_GRID_HEIGHT = 500

# Choose the ArUco dictionary (ensure it matches your markers)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
# --- End Configuration ---

# Initialize the webcam
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Cannot open webcam.")
    exit()

# Load the predefined ArUco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)

# Create detector parameters (handling potential version differences)
try:
    arucoParams = cv2.aruco.DetectorParameters()
    print("Using cv2.aruco.DetectorParameters()")
except AttributeError:
    arucoParams = cv2.aruco.DetectorParameters_create()
    print("Using cv2.aruco.DetectorParameters_create()")

# Initialize the detector (handling potential version differences)
try:
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    print("Using cv2.aruco.ArucoDetector()")
    use_detector_object = True
except AttributeError:
    detector = None
    use_detector_object = False
    print("Using cv2.aruco.detectMarkers() directly.")

# Define the destination points for the perspective transform (corners of the flat grid)
dst_pts = np.array([
    [0, 0],                                # Top-Left
    [FLAT_GRID_WIDTH - 1, 0],              # Top-Right
    [FLAT_GRID_WIDTH - 1, FLAT_GRID_HEIGHT - 1], # Bottom-Right
    [0, FLAT_GRID_HEIGHT - 1]              # Bottom-Left
], dtype='float32')

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Detect ArUco markers
    if use_detector_object:
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # --- Process Detections ---
    boundary_corners_dict = {} # Store corners keyed by ID
    object_marker_corners = None
    object_marker_center = None

    if ids is not None:
        flat_ids = ids.flatten()

        # Find and store corners for boundary and object markers
        for i, marker_id in enumerate(flat_ids):
            if marker_id in BOUNDARY_MARKER_IDS:
                boundary_corners_dict[marker_id] = corners[i]
            elif marker_id == OBJECT_MARKER_ID:
                object_marker_corners = corners[i]
                # Calculate object center [3]
                obj_corners_reshaped = object_marker_corners.reshape((4, 2))
                object_marker_center = tuple(map(int, obj_corners_reshaped.mean(axis=0)))


        # --- Grid Logic ---
        # Check if all required boundary markers were detected
        if len(boundary_corners_dict) == len(BOUNDARY_MARKER_IDS):
            # Extract the specific outer corners needed for perspective transform
            # based on the BOUNDARY_MARKER_IDS_MAP
            src_pts_list = [None] * 4 # TL, TR, BR, BL

            for marker_id, corner_role in BOUNDARY_MARKER_IDS_MAP.items():
                marker_corners = boundary_corners_dict[marker_id].reshape((4, 2)) # Get corners for this marker [3]
                if corner_role == 'TL':
                    src_pts_list[0] = marker_corners[0] # Top-left corner of TL marker [3]
                elif corner_role == 'TR':
                    src_pts_list[1] = marker_corners[1] # Top-right corner of TR marker [3]
                elif corner_role == 'BR':
                    src_pts_list[2] = marker_corners[2] # Bottom-right corner of BR marker [3]
                elif corner_role == 'BL':
                    src_pts_list[3] = marker_corners[3] # Bottom-left corner of BL marker [3]

            # Check if all source points were found (should always be true if len is correct)
            if all(pt is not None for pt in src_pts_list):
                src_pts = np.array(src_pts_list, dtype='float32')

                # Calculate the perspective transformation matrix
                matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

                # Draw the outer boundary using the extracted corners (optional)
                cv2.polylines(frame, [np.int32(src_pts)], isClosed=True, color=(255, 0, 0), thickness=2)

                # If the object marker is detected, find its grid cell
                if object_marker_center is not None:
                    # Transform the object marker's center point
                    obj_center_np = np.array([[object_marker_center]], dtype='float32')
                    transformed_point = cv2.perspectiveTransform(obj_center_np, matrix)

                    if transformed_point is not None:
                        tx, ty = transformed_point[0][0]

                        # Calculate grid cell size in the flattened coordinate system
                        cell_width = FLAT_GRID_WIDTH / GRID_COLS
                        cell_height = FLAT_GRID_HEIGHT / GRID_ROWS

                        # Calculate column and row index (0-based)
                        col_index = math.floor(tx / cell_width)
                        row_index = math.floor(ty / cell_height)

                        # Ensure indices are within bounds
                        col_index = max(0, min(col_index, GRID_COLS - 1))
                        row_index = max(0, min(row_index, GRID_ROWS - 1))

                        # Display the grid cell coordinates on the frame
                        text = f"Grid: ({row_index}, {col_index})"
                        cv2.putText(frame, text, (object_marker_center[0] + 10, object_marker_center[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        # --- Optional: Draw the flattened grid for visualization ---
                        # warped_frame = cv2.warpPerspective(frame, matrix, (FLAT_GRID_WIDTH, FLAT_GRID_HEIGHT))
                        # # Draw grid lines on warped image
                        # for r in range(1, GRID_ROWS):
                        #     cv2.line(warped_frame, (0, int(r * cell_height)), (FLAT_GRID_WIDTH - 1, int(r * cell_height)), (150, 150, 150), 1)
                        # for c in range(1, GRID_COLS):
                        #     cv2.line(warped_frame, (int(c * cell_width), 0), (int(c * cell_width), FLAT_GRID_HEIGHT - 1), (150, 150, 150), 1)
                        # # Draw transformed object center
                        # cv2.circle(warped_frame, (int(tx), int(ty)), 5, (0, 0, 255), -1)
                        # cv2.imshow("Warped Grid View", warped_frame)
                        # --- End Optional Visualization ---

            else:
                 print("Warning: Could not extract all required boundary corners.")

        # Draw outlines and IDs for ALL detected markers [4]
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # --- Display ---
    cv2.imshow('ArUco Detection and Grid Location', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
