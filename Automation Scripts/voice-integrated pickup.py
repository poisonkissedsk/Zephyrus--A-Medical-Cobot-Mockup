import cv2
import numpy as np
import math
import time
import serial
import speech_recognition as sr # Import speech recognition library [3]

# --- Configuration ---
# Boundary marker IDs and roles [1]
BOUNDARY_MARKER_IDS_MAP = { 1: 'TL', 2: 'TR', 3: 'BR', 4: 'BL' }
BOUNDARY_MARKER_IDS = set(BOUNDARY_MARKER_IDS_MAP.keys())
# Object marker ID [1]
OBJECT_MARKER_ID = 0
# Target grid cell (Row, Column) [1]
TARGET_ROW = 2
TARGET_COL = 2
# Grid dimensions
GRID_ROWS = 5
GRID_COLS = 5
# Flattened grid view size
FLAT_GRID_WIDTH = 500
FLAT_GRID_HEIGHT = 500
# ArUco dictionary
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# --- Serial Port Configuration ---
SERIAL_PORT = '/dev/ttyACM0' # <<< CHANGE TO YOUR ARDUINO PORT (e.g., 'COM3', 'COM5')
BAUD_RATE = 9600

# --- Voice Command Configuration ---
TRIGGER_PHRASE = "start pick" # Command to initiate pick & place
# Optional: Adjust microphone listening time / sensitivity
LISTEN_TIMEOUT = 5 # Max seconds to listen for command
PHRASE_TIME_LIMIT = 5 # Max seconds for the phrase itself

# --- Pre-calculated Servo Angles (REPLACE WITH YOUR ACTUAL VALUES) ---
# Order: [Waist(s1), Shoulder(s2), Elbow(s3), WristRoll(s4), WristPitch(s5), Gripper(s6)]
target_cell_pick_angles = {
    "approach_pick": [ 76, 52,  79, 36,  49,  55],
    "pick":          [ 76, 52,  79, 36,  49,  55],
    "close_gripper": [ 76, 52,  79, 36,  49,  0],
    "lift":          [ 76, 76,  79, 36,  49,  0],
}
common_angles = {
    "home":          [ 62, 118,  153, 36,  49,  10],
    "approach_drop": [3, 108,  135, 36,  49,  0],
    "drop":          [3, 108,  135, 36,  49,  0],
    "open_gripper":  [3, 108,  135, 36,  49,  45],
}
# --- End Pre-calculated Angles ---

# --- Robot Control & Serial Functions (Unchanged from previous) ---
ser = None
def initialize_robot_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[SERIAL] Attempting connection to {SERIAL_PORT}...")
        time.sleep(2)
        if ser.is_open:
            print(f"[SERIAL] Connected successfully.")
            return True
        else:
             print(f"[SERIAL] Failed to open port.")
             return False
    except serial.SerialException as e:
        print(f"[SERIAL] Error connecting: {e}")
        ser = None
        return False
    except Exception as e:
        print(f"[SERIAL] Non-serial error during connection: {e}")
        ser = None
        return False

def close_robot_serial():
    global ser
    if ser and ser.is_open:
        ser.close()
        print("[SERIAL] Connection closed.")
        ser = None

def send_servo_angles(angle_list):
    global ser
    if not (ser and ser.is_open): return False
    if len(angle_list) != 6: return False
    try:
        cmd = ("s1{:03d}s2{:03d}s3{:03d}s4{:03d}s5{:03d}s6{:03d}"
               .format(*angle_list))
        print(f"[SERIAL] Sending: {cmd}")
        ser.write((cmd + "\n").encode('utf-8'))
        time.sleep(1.5) # Adjust delay as needed
        return True
    except Exception as e:
        print(f"[SERIAL] Error sending data: {e}")
        return False
# --- End Robot Control Functions ---

# --- Voice Recognition Functions ---
recognizer = None
microphone = None

def initialize_speech_recognition():
    """Initializes the speech recognizer and microphone."""
    global recognizer, microphone
    recognizer = sr.Recognizer()
    try:
        microphone = sr.Microphone()
        # Optional: Adjust energy threshold dynamically if needed
        # with microphone as source:
        #     print("[VOICE] Adjusting for ambient noise...")
        #     recognizer.adjust_for_ambient_noise(source, duration=1)
        print("[VOICE] Speech Recognition initialized.")
        return True
    except AttributeError:
         print("[VOICE] ERROR: PyAudio not found. Install it with 'pip install PyAudio'.")
         return False
    except Exception as e:
        print(f"[VOICE] ERROR: Could not find microphone or initialize recognizer: {e}")
        return False

def listen_for_command(timeout=LISTEN_TIMEOUT, phrase_limit=PHRASE_TIME_LIMIT):
    """Listens for a voice command and returns the recognized text."""
    global recognizer, microphone
    if not (recognizer and microphone):
        print("[VOICE] Error: Recognizer or Microphone not initialized.")
        return None

    with microphone as source:
        print(f"[VOICE] Listening for command (max {timeout}s)...")
        # Optional: uncomment if you want noise adjustment each time
        # recognizer.adjust_for_ambient_noise(source, duration=0.5)
        try:
            # Listen for audio input with timeout and phrase limits [2]
            audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_limit)
        except sr.WaitTimeoutError:
            print("[VOICE] No speech detected within timeout.")
            return None

    try:
        print("[VOICE] Recognizing...")
        # Use Google Web Speech API for recognition (requires internet) [3, 5, 6]
        text = recognizer.recognize_google(audio)
        print(f"[VOICE] You said: '{text}'")
        return text.lower() # Return lowercase text for easier matching
    except sr.UnknownValueError:
        print("[VOICE] Sorry, I could not understand the audio.")
        return None
    except sr.RequestError as e:
        print(f"[VOICE] Could not request results from Google Speech Recognition service; {e}")
        return None
    except Exception as e:
        print(f"[VOICE] Unexpected error during recognition: {e}")
        return None
# --- End Voice Recognition Functions ---

# --- Main Program ---

# 1. Initialize Systems
print("[INFO] Initializing systems...")
robot_ok = initialize_robot_serial()
speech_ok = initialize_speech_recognition()

if not robot_ok:
    print("FATAL: Robot communication failed. Exiting.")
    exit()
if not speech_ok:
    print("WARNING: Speech recognition failed to initialize. Voice commands disabled.")
    # Allow running without voice if desired, or exit()

# 2. Initialize Camera and ArUco Detector
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("FATAL: Cannot open webcam. Exiting.")
    if robot_ok: close_robot_serial()
    exit()

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
try:
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    use_detector_object = True
except AttributeError:
    arucoParams = cv2.aruco.DetectorParameters_create()
    detector = None
    use_detector_object = False

# Define destination points for perspective transform
dst_pts = np.array([[0, 0], [FLAT_GRID_WIDTH - 1, 0], [FLAT_GRID_WIDTH - 1, FLAT_GRID_HEIGHT - 1], [0, FLAT_GRID_HEIGHT - 1]], dtype='float32')

# State flags
object_picked_up = False
listening_active = False # Flag to indicate if currently listening

# 3. Move Robot Home
print("[ROBOT] Moving to Home Position...")
send_servo_angles(common_angles["home"])
print("[INFO] Robot homed. Starting main loop.")

# --- Main Loop ---
try:
    while True:
        # 4. Read Frame and Detect Markers
        ret, frame = cap.read()
        if not ret: break

        if use_detector_object:
            corners, ids, rejected = detector.detectMarkers(frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        boundary_corners_dict = {}
        object_marker_center = None
        object_row, object_col = -1, -1
        object_in_target_cell = False
        status_text = ""

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

            # 6. Calculate Grid Position (if boundary markers found)
            if len(boundary_corners_dict) == len(BOUNDARY_MARKER_IDS):
                src_pts_list = [None] * 4
                valid_corners = True
                # ... (corner extraction logic - same as before) ...
                for marker_id, corner_role in BOUNDARY_MARKER_IDS_MAP.items():
                     if marker_id not in boundary_corners_dict: valid_corners = False; break
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
                        # ... (transform object center, calculate row/col - same as before) ...
                        obj_center_np = np.array([[object_marker_center]], dtype='float32')
                        transformed_point = cv2.perspectiveTransform(obj_center_np, matrix)
                        if transformed_point is not None:
                            tx, ty = transformed_point[0][0]
                            cell_width = FLAT_GRID_WIDTH / GRID_COLS
                            cell_height = FLAT_GRID_HEIGHT / GRID_ROWS
                            col_index = max(0, min(math.floor(tx / cell_width), GRID_COLS - 1))
                            row_index = max(0, min(math.floor(ty / cell_height), GRID_ROWS - 1))
                            object_row, object_col = row_index, col_index

                            # Check if object is in the specific target cell
                            object_in_target_cell = (object_row == TARGET_ROW and object_col == TARGET_COL)

                            # Display grid location
                            loc_text = f"Grid: ({object_row}, {object_col})"
                            loc_color = (0, 0, 255) if object_in_target_cell else (0, 255, 0)
                            cv2.putText(frame, loc_text, (object_marker_center[0] + 10, object_marker_center[1]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, loc_color, 2)

        # 7. Voice Command Logic & Robot Triggering
        command_recognized = None
        if speech_ok and object_in_target_cell and not object_picked_up:
            status_text = f"Object in Target ({TARGET_ROW},{TARGET_COL}). Ready."
            # Only listen if not currently busy with a sequence
            if not listening_active:
                 status_text += f" Say '{TRIGGER_PHRASE}'"
                 # Trigger listening in this iteration (non-blocking approach is more complex)
                 listening_active = True
                 command_recognized = listen_for_command()
                 listening_active = False # Reset after listening attempt

                 # Check if the recognized command contains the trigger phrase [4, 5, 6]
                 if command_recognized and TRIGGER_PHRASE in command_recognized:
                     print(f"\n>>> TRIGGER PHRASE '{TRIGGER_PHRASE}' DETECTED! Starting Sequence <<<")
                     object_picked_up = True # Prevent re-triggering
                     status_text = "SEQUENCE INITIATED!"

                     # --- Execute Pick and Place Sequence ---
                     print("[ROBOT] Moving to Approach Pick"); success = send_servo_angles(target_cell_pick_angles["approach_pick"])
                     if success: print("[ROBOT] Moving to Pick Height"); success = send_servo_angles(target_cell_pick_angles["pick"])
                     if success: print("[ROBOT] Closing Gripper"); success = send_servo_angles(target_cell_pick_angles["close_gripper"])
                     if success: print("[ROBOT] Lifting Object"); success = send_servo_angles(target_cell_pick_angles["lift"])
                     if success: print("[ROBOT] Moving to Approach Drop"); success = send_servo_angles(common_angles["approach_drop"])
                     if success: print("[ROBOT] Moving to Drop Height"); success = send_servo_angles(common_angles["drop"])
                     if success: print("[ROBOT] Opening Gripper"); success = send_servo_angles(common_angles["open_gripper"])
                     print("[ROBOT] Returning to Home")
                     if not send_servo_angles(common_angles["home"]): print("[ROBOT] Warning: Failed final home.")
                     # --- Sequence End ---

                     if success: print("--- Sequence Complete Successfully ---")
                     else: print("--- Sequence Interrupted due to Send Error ---")
                     # Keep object_picked_up = True until marker disappears
                 elif command_recognized is not None: # Heard something, but not the trigger
                      status_text = f"Heard '{command_recognized}', waiting for '{TRIGGER_PHRASE}'"

        elif object_picked_up:
             status_text = "Sequence complete or in progress..."
        elif not speech_ok:
             status_text = "Voice control disabled."
        else:
             status_text = "Waiting for object in target cell..."


        # 8. Reset State if Object Marker is Lost
        if object_marker_center is None and object_picked_up:
            print("Object marker lost. Ready for next pick.")
            object_picked_up = False # Allow triggering again

        # 9. Display Status on Frame
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # 10. Display Frame
        cv2.imshow('ArUco + Voice Control', frame)

        # 11. Check for Quit Key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quit key pressed. Exiting...")
            break

finally:
    # --- Cleanup ---
    print("[INFO] Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        print("[ROBOT] Sending final home command.")
        send_servo_angles(common_angles["home"])
        time.sleep(2)
    close_robot_serial()
    print("Program terminated.")
