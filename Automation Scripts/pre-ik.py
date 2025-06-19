import cv2
import numpy as np
import math
import time
import serial

# --- configuration ---
boundary_marker_ids_map = {1: 'tl', 2: 'tr', 3: 'br', 4: 'bl'}
boundary_marker_ids = set(boundary_marker_ids_map.keys())

object_marker_id = 0

target_row = 2
target_col = 2

grid_rows = 5
grid_cols = 5

flat_grid_width = 500
flat_grid_height = 500

aruco_dict_type = cv2.aruco.DICT_4X4_50

serial_port = 'COM5'
baud_rate = 9600

# --- pre-calculated servo angles (to be replaced with inverse kinematics) ---
target_cell_pick_angles = {
    "approach_pick": [70, 50, 77, 36, 49, 64],
    "pick": [70, 50, 77, 36, 49, 64],
    "close_gripper": [70, 50, 77, 36, 49, 0],
    "lift": [65, 72, 77, 36, 49, 0],
}

common_angles = {
    "home": [62, 118, 153, 36, 49, 10],
    "approach_drop": [3, 85, 100, 36, 49, 0],
    "drop": [3, 108, 135, 36, 49, 0],
    "open_gripper": [3, 108, 135, 36, 49, 45],
}

ser = None

def initialize_robot_serial():
    global ser
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"[serial] Attempting connection to {serial_port}...")
        time.sleep(2)
        if ser.is_open:
            print(f"[serial] Connected successfully to {serial_port} at {baud_rate} baud.")
            return True
        else:
            print(f"[serial] Failed to open port {serial_port}, but no exception.")
            return False
    except serial.SerialException as e:
        print(f"[serial] Error connecting to {serial_port}: {e}")
        ser = None
        return False
    except Exception as e:
        print(f"[serial] Non-serial error during connection: {e}")
        ser = None
        return False

def close_robot_serial():
    global ser
    if ser and ser.is_open:
        ser.close()
        print("[serial] Connection closed.")
        ser = None

def send_servo_angles(angle_list):
    global ser
    if not (ser and ser.is_open):
        print("[serial] Error: Not connected.")
        return False

    if len(angle_list) != 6:
        print(f"[serial] Error: Expected 6 angles, got {len(angle_list)}")
        return False

    try:
        cmd = ("s1{:03d}s2{:03d}s3{:03d}s4{:03d}s5{:03d}s6{:03d}".format(*angle_list))
        print(f"[serial] Sending: {cmd}")
        ser.write((cmd + "\n").encode('utf-8'))
        time.sleep(1.5)
        return True
    except serial.SerialException as e:
        print(f"[serial] Error sending data: {e}")
        return False
    except Exception as e:
        print(f"[serial] Unexpected error during send: {e}")
        return False

if not initialize_robot_serial():
    print("error: cannot connect to robot controller. please check port and connection. exiting.")
    exit()

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("error: cannot open webcam.")
    close_robot_serial()
    exit()

aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
try:
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    use_detector_object = True
except AttributeError:
    aruco_params = cv2.aruco.DetectorParameters_create()
    detector = None
    use_detector_object = False

dst_pts = np.array([[0, 0], [flat_grid_width - 1, 0], [flat_grid_width - 1, flat_grid_height - 1], [0, flat_grid_height - 1]], dtype='float32')

object_picked_up = False

print("[robot] Moving to home position...")
if not send_servo_angles(common_angles["home"]):
    print("[robot] Error sending home command. Check connection.")
print("[info] Robot homed (or command sent). Starting detection loop.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("error: failed to capture frame.")
            break

        if use_detector_object:
            corners, ids, rejected = detector.detectMarkers(frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        boundary_corners_dict = {}
        object_marker_center = None
        object_row, object_col = -1, -1

        if ids is not None:
            flat_ids = ids.flatten()
            for i, marker_id in enumerate(flat_ids):
                if marker_id in boundary_marker_ids:
                    boundary_corners_dict[marker_id] = corners[i]
                elif marker_id == object_marker_id:
                    obj_corners_reshaped = corners[i].reshape((4, 2))
                    object_marker_center = tuple(map(int, obj_corners_reshaped.mean(axis=0)))

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if len(boundary_corners_dict) == len(boundary_marker_ids):
                src_pts_list = [None] * 4
                valid_corners = True
                for marker_id, corner_role in boundary_marker_ids_map.items():
                    if marker_id not in boundary_corners_dict:
                        valid_corners = False
                        break
                    marker_corners = boundary_corners_dict[marker_id].reshape((4, 2))
                    if corner_role == 'tl':
                        src_pts_list[0] = marker_corners[0]
                    elif corner_role == 'tr':
                        src_pts_list[1] = marker_corners[1]
                    elif corner_role == 'br':
                        src_pts_list[2] = marker_corners[2]
                    elif corner_role == 'bl':
                        src_pts_list[3] = marker_corners[3]

                if valid_corners and all(pt is not None for pt in src_pts_list):
                    src_pts = np.array(src_pts_list, dtype='float32')
                    cv2.polylines(frame, [np.int32(src_pts)], isClosed=True, color=(255, 0, 0), thickness=2)
                    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

                    if object_marker_center is not None:
                        obj_center_np = np.array([[object_marker_center]], dtype='float32')
                        transformed_point = cv2.perspectiveTransform(obj_center_np, matrix)
                        if transformed_point is not None:
                            tx, ty = transformed_point[0][0]
                            cell_width = flat_grid_width / grid_cols
                            cell_height = flat_grid_height / grid_rows
                            col_index = max(0, min(math.floor(tx / cell_width), grid_cols - 1))
                            row_index = max(0, min(math.floor(ty / cell_height), grid_rows - 1))
                            object_row, object_col = row_index, col_index

                            text = f"grid: ({object_row}, {object_col})"
                            cv2.putText(frame, text, (object_marker_center[0] + 10, object_marker_center[1]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            if object_row == target_row and object_col == target_col:
                                cv2.putText(frame, f"target ({object_row}, {object_col})", (10, 30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                                if not object_picked_up:
                                    print(f"\n>>> object detected at target ({object_row}, {object_col}). starting sequence <<<")
                                    object_picked_up = True

                                    print("[robot] moving to approach pick")
                                    success = send_servo_angles(target_cell_pick_angles["approach_pick"])
                                    if success:
                                        print("[robot] moving to pick height")
                                        success = send_servo_angles(target_cell_pick_angles["pick"])
                                    if success:
                                        print("[robot] closing gripper")
                                        success = send_servo_angles(target_cell_pick_angles["close_gripper"])
                                    if success:
                                        print("[robot] lifting object")
                                        success = send_servo_angles(target_cell_pick_angles["lift"])

                                    if success:
                                        print("[robot] moving to approach drop")
                                        success = send_servo_angles(common_angles["approach_drop"])
                                    if success:
                                        print("[robot] moving to drop height")
                                        success = send_servo_angles(common_angles["drop"])
                                    if success:
                                        print("[robot] opening gripper")
                                        success = send_servo_angles(common_angles["open_gripper"])

                                    print("[robot] returning to home")
                                    if not send_servo_angles(common_angles["home"]):
                                        print("[robot] warning: failed to send final home command.")

                                    if success:
                                        print("--- sequence complete successfully ---")
                                    else:
                                        print("--- sequence interrupted due to send error ---")

        if object_marker_center is None and object_picked_up:
            print("object marker lost. ready for next pick.")
            object_picked_up = False

        cv2.imshow('aruco detection and robot control (single box)', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("quit key pressed. exiting...")
            break

finally:
    print("[info] cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        print("[robot] sending final home command before exit.")
        send_servo_angles(common_angles["home"])
        time.sleep(2)
    close_robot_serial()
    print("program terminated.")
