import cv2
import numpy as np
import math
import time
import serial # For communicating with Arduino

# --- Configuration ---
# Boundary marker IDs and roles from the image [1]
BOUNDARY_MARKER_IDS_MAP = { 1: 'TL', 2: 'TR', 3: 'BR', 4: 'BL' }
BOUNDARY_MARKER_IDS = set(BOUNDARY_MARKER_IDS_MAP.keys())

# Object marker ID from the image [1]
OBJECT_MARKER_ID = 0

# Target grid cell to trigger pick-and-place (Row, Column) from the image [1]
TARGET_ROW = 2
TARGET_COL = 2

# Grid dimensions (Rows, Columns) - Must match physical setup
GRID_ROWS = 5
GRID_COLS = 5

# Target size for flattened grid view (pixels) - Doesn't affect angles
FLAT_GRID_WIDTH = 500
FLAT_GRID_HEIGHT = 500

# ArUco dictionary - Ensure this matches your markers
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# --- Serial Port Configuration (Match your Python GUI code) ---
SERIAL_PORT = 'COM5' # <<< CHANGE TO YOUR ARDUINO PORT (e.g., 'COM3', 'COM5')
BAUD_RATE = 9600

# --- Pre-calculated Servo Angles (REPLACE WITH YOUR ACTUAL VALUES) ---
# Each list MUST have 6 angles in the order:
# [Waist(s1), Shoulder(s2), Elbow(s3), WristRoll(s4), WristPitch(s5), Gripper(s6)]

target_cell_pick_angles = {
    # Angles for picking from TARGET CELL (2, 2)
    "approach_pick": [ 70, 50,  77, 36,  49,  64], # Example: Above cell (2,2), gripper wide open
    "pick":          [ 70, 50,  77, 36,  49,  64], # Example: Lowered into cell (2,2), gripper open
    "close_gripper": [ 70, 50,  77, 36,  49,  0], # Example: Gripper closed at pick location
    "lift":          [  65, 72,  77, 36,  49,  0], # Example: Lifted slightly, gripper closed
}

common_angles = {
    # Angles for common positions (Home, Drop)
    "home":          [ 62, 118,  153, 36,  49,  10], # Matches Arduino setup() values, gripper closed slightly
    "approach_drop": [3, 85,  100, 36,  49,  0], # Example: Above drop point, gripper closed
    "drop":          [3, 108,  135, 36,  49,  0], # Example: At drop point, gripper closed
    "open_gripper":  [3, 108,  135, 36,  49,  45], # Example: At drop point, gripper open
}
# --- End Pre-calculated Angles ---


# --- Robot Control Functions (Serial Communication - Adopts GUI logic) ---
ser = None # Global serial object

def initialize_robot_serial():
    """Initializes the serial connection."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[SERIAL] Attempting connection to {SERIAL_PORT}...")
        time.sleep(2) # Crucial delay for Arduino reset/bootloader
        if ser.is_open:
            print(f"[SERIAL] Connected successfully to {SERIAL_PORT} at {BAUD_RATE} baud.")
            return True
        else:
             print(f"[SERIAL] Failed to open port {SERIAL_PORT}, but no exception.")
             return False
    except serial.SerialException as e:
        print(f"[SERIAL] Error connecting to {SERIAL_PORT}: {e}")
        ser = None
        return False
    except Exception as e:
        print(f"[SERIAL] Non-serial error during connection: {e}")
        ser = None
        return False

def close_robot_serial():
    """Closes the serial connection."""
    global ser
    if ser and ser.is_open:
        ser.close()
        print("[SERIAL] Connection closed.")
        ser = None

def send_servo_angles(angle_list):
    """Formats command like s1XXXs2XXX... and sends over serial."""
    global ser
    if not (ser and ser.is_open):
        print("[SERIAL] Error: Not connected.")
        return False

    if len(angle_list) != 6:
        print(f"[SERIAL] Error: Expected 6 angles, got {len(angle_list)}")
        return False

    try:
        # Format command EXACTLY like the Python GUI and Arduino expects
        # s1<angle1>s2<angle2>...s6<angle6>\n, with 3-digit zero-padded angles
        cmd = ("s1{:03d}s2{:03d}s3{:03d}s4{:03d}s5{:03d}s6{:03d}"
               .format(*angle_list))
        print(f"[SERIAL] Sending: {cmd}")
        ser.write((cmd + "\n").encode('utf-8')) # Send command + newline, encoded

        # ** Adjust this delay based on your robot's speed and stability **
        # It needs to be long enough for the arm to physically complete the move.
        time.sleep(1.5)

        # Optional: Read acknowledgment from Arduino if implemented there
        # response = ser.readline().decode().strip()
        # print(f"[SERIAL] Received: {response}")
        return True

    except serial.SerialException as e:
        print(f"[SERIAL] Error sending data: {e}")
        return False
    except Exception as e:
        print(f"[SERIAL] Unexpected error during send: {e}")
        return False
# --- End Robot Control Functions ---

# --- Main Program ---

# 1. Initialize Robot Communication
if not initialize_robot_serial():
    print("ERROR: Cannot connect to robot controller. Please check port and connection. Exiting.")
    exit()

# 2. Initialize Camera and ArUco Detector
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Cannot open webcam.")
    close_robot_serial()
    exit()

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
try: # Handle OpenCV version differences
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    use_detector_object = True
except AttributeError:
    arucoParams = cv2.aruco.DetectorParameters_create()
    detector = None
    use_detector_object = False

# Define destination points for perspective transform
dst_pts = np.array([[0, 0], [FLAT_GRID_WIDTH - 1, 0], [FLAT_GRID_WIDTH - 1, FLAT_GRID_HEIGHT - 1], [0, FLAT_GRID_HEIGHT - 1]], dtype='float32')

# State flag
object_picked_up = False

# 3. Move Robot to Initial Home Position
print("[ROBOT] Moving to Home Position...")
if not send_servo_angles(common_angles["home"]):
    print("[ROBOT] Error sending Home command. Check connection.")
    # Decide whether to continue or exit if homing fails
    # exit() # Or just print a warning and continue
print("[INFO] Robot homed (or command sent). Starting detection loop.")

# --- Main Loop ---
try: # Use try...finally to ensure cleanup happens
    while True:
        # 4. Read Frame and Detect Markers
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break # Exit loop if camera fails

        if use_detector_object:
            corners, ids, rejected = detector.detectMarkers(frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        boundary_corners_dict = {}
        object_marker_center = None
        object_row, object_col = -1, -1

        # 5. Process Detections
        if ids is not None:
            flat_ids = ids.flatten()
            for i, marker_id in enumerate(flat_ids):
                if marker_id in BOUNDARY_MARKER_IDS:
                    boundary_corners_dict[marker_id] = corners[i]
                elif marker_id == OBJECT_MARKER_ID:
                    obj_corners_reshaped = corners[i].reshape((4, 2))
                    object_marker_center = tuple(map(int, obj_corners_reshaped.mean(axis=0)))

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 6. Calculate Grid Position (Requires all boundary markers)
            if len(boundary_corners_dict) == len(BOUNDARY_MARKER_IDS):
                src_pts_list = [None] * 4
                valid_corners = True
                for marker_id, corner_role in BOUNDARY_MARKER_IDS_MAP.items():
                    if marker_id not in boundary_corners_dict:
                        valid_corners = False; break
                    marker_corners = boundary_corners_dict[marker_id].reshape((4, 2))
                    if corner_role == 'TL': src_pts_list[0] = marker_corners[0]
                    elif corner_role == 'TR': src_pts_list[1] = marker_corners[1]
                    elif corner_role == 'BR': src_pts_list[2] = marker_corners[2]
                    elif corner_role == 'BL': src_pts_list[3] = marker_corners[3]

                if valid_corners and all(pt is not None for pt in src_pts_list):
                    src_pts = np.array(src_pts_list, dtype='float32')
                    cv2.polylines(frame, [np.int32(src_pts)], isClosed=True, color=(255, 0, 0), thickness=2)
                    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

                    if object_marker_center is not None:
                        obj_center_np = np.array([[object_marker_center]], dtype='float32')
                        transformed_point = cv2.perspectiveTransform(obj_center_np, matrix)
                        if transformed_point is not None:
                            tx, ty = transformed_point[0][0]
                            cell_width = FLAT_GRID_WIDTH / GRID_COLS
                            cell_height = FLAT_GRID_HEIGHT / GRID_ROWS
                            col_index = max(0, min(math.floor(tx / cell_width), GRID_COLS - 1))
                            row_index = max(0, min(math.floor(ty / cell_height), GRID_ROWS - 1))
                            object_row, object_col = row_index, col_index

                            text = f"Grid: ({object_row}, {object_col})"
                            cv2.putText(frame, text, (object_marker_center[0] + 10, object_marker_center[1]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            # 7. Check if Object is in TARGET CELL and Trigger Sequence
                            if object_row == TARGET_ROW and object_col == TARGET_COL:
                                cv2.putText(frame, f"TARGET ({object_row}, {object_col})", (10, 30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Highlight target

                                if not object_picked_up:
                                    print(f"\n>>> Object detected at TARGET ({object_row}, {object_col}). Starting Sequence <<<")
                                    object_picked_up = True # Prevent re-triggering

                                    # --- Execute Pick Sequence for Target Cell ---
                                    print("[ROBOT] Moving to Approach Pick")
                                    success = send_servo_angles(target_cell_pick_angles["approach_pick"])
                                    if success: print("[ROBOT] Moving to Pick Height"); success = send_servo_angles(target_cell_pick_angles["pick"])
                                    if success: print("[ROBOT] Closing Gripper"); success = send_servo_angles(target_cell_pick_angles["close_gripper"])
                                    if success: print("[ROBOT] Lifting Object"); success = send_servo_angles(target_cell_pick_angles["lift"])

                                    # --- Execute Common Drop Sequence ---
                                    if success: print("[ROBOT] Moving to Approach Drop"); success = send_servo_angles(common_angles["approach_drop"])
                                    if success: print("[ROBOT] Moving to Drop Height"); success = send_servo_angles(common_angles["drop"])
                                    if success: print("[ROBOT] Opening Gripper"); success = send_servo_angles(common_angles["open_gripper"])

                                    # --- Return Home ---
                                    print("[ROBOT] Returning to Home")
                                    if not send_servo_angles(common_angles["home"]):
                                         print("[ROBOT] Warning: Failed to send final home command.")

                                    if success:
                                         print("--- Sequence Complete Successfully ---")
                                    else:
                                         print("--- Sequence Interrupted due to Send Error ---")

        # 8. Reset State if Object Marker is Lost
        if object_marker_center is None and object_picked_up:
            print("Object marker lost. Ready for next pick.")
            object_picked_up = False

        # 9. Display Frame
        cv2.imshow('ArUco Detection and Robot Control (Single Box)', frame)

        # 10. Check for Quit Key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quit key pressed. Exiting...")
            break

finally:
    # --- Cleanup ---
    print("[INFO] Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    # Ensure robot is sent home before closing serial if connection is still open
    if ser and ser.is_open:
        print("[ROBOT] Sending final home command before exit.")
        send_servo_angles(common_angles["home"])
        time.sleep(2) # Wait for final move
    close_robot_serial() # Close serial port
    print("Program terminated.")

