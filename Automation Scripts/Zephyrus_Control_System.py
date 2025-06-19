"""
Zephyrus Robotic Control System
"""

import cv2
import numpy as np
import math
import time
import serial
from typing import Optional, Tuple, Dict
from dataclasses import dataclass

# --------------------------
# Configuration Constants
# --------------------------
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
SERIAL_PORT = 'COM5'
BAUD_RATE = 9600
CAMERA_INDEX = 1


@dataclass
class GridConfig:
    rows: int = 5
    cols: int = 5
    target_cell: Tuple[int, int] = (2, 2)
    flat_size: Tuple[int, int] = (500, 500)


@dataclass
class ArmPresets:
    home: Tuple[int, ...] = (62, 118, 153, 36, 49, 10)
    pick_approach: Tuple[int, ...] = (70, 50, 77, 36, 49, 64)
    pick_grasp: Tuple[int, ...] = (70, 50, 77, 36, 49, 0)
    pick_lift: Tuple[int, ...] = (65, 72, 77, 36, 49, 0)
    drop_approach: Tuple[int, ...] = (3, 85, 100, 36, 49, 0)
    drop_release: Tuple[int, ...] = (3, 108, 135, 36, 49, 45)


# --------------------------
# Vision Processing Module
# --------------------------
class ArucoProcessor:
    def __init__(self):
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE),
            cv2.aruco.DetectorParameters()
        )
        self.boundary_ids = {1: 'TL', 2: 'TR', 3: 'BR', 4: 'BL'}
        self.object_id = 0

    def process_frame(self, frame: np.ndarray) -> Tuple[Optional[Dict[int, np.ndarray]], Optional[Tuple[int, int]]]:
        """Detect markers and calculate grid position"""
        corners, ids, _ = self.detector.detectMarkers(frame)
        boundary_markers = {}
        object_position = None

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.boundary_ids:
                    boundary_markers[marker_id] = corners[i]
                elif marker_id == self.object_id:
                    object_position = self._calculate_grid_position(corners[i], boundary_markers)

        return boundary_markers, object_position

    def _calculate_grid_position(self, corners: np.ndarray, boundaries: Dict[int, np.ndarray]) -> Optional[
        Tuple[int, int]]:
        """Transform object coordinates to grid space"""
        if len(boundaries) != 4:
            return None

        src_pts = self._extract_boundary_points(boundaries)
        if src_pts is None:
            return None

        matrix = cv2.getPerspectiveTransform(src_pts, np.array([
            [0, 0],
            [GridConfig.flat_size[0] - 1, 0],
            [GridConfig.flat_size[0] - 1, GridConfig.flat_size[1] - 1],
            [0, GridConfig.flat_size[1] - 1]
        ], dtype='float32'))

        obj_center = np.mean(corners.reshape(4, 2), axis=0)
        transformed = cv2.perspectiveTransform(np.array([[obj_center]], dtype='float32'), matrix)

        cell_width = GridConfig.flat_size[0] / GridConfig.cols
        cell_height = GridConfig.flat_size[1] / GridConfig.rows

        return (
            min(int(transformed[0][0][1] // cell_height), GridConfig.rows - 1),
            min(int(transformed[0][0][0] // cell_width), GridConfig.cols - 1)
        )

    def _extract_boundary_points(self, boundaries: Dict[int, np.ndarray]) -> Optional[np.ndarray]:
        """Extract ordered boundary points for perspective transform"""
        pts = []
        for marker_id in [1, 2, 3, 4]:
            if marker_id not in boundaries:
                return None
            corner_index = {'TL': 0, 'TR': 1, 'BR': 2, 'BL': 3}[self.boundary_ids[marker_id]]
            pts.append(boundaries[marker_id].reshape(4, 2)[corner_index])
        return np.array(pts, dtype='float32')


# --------------------------
# Robot Control Module
# --------------------------
class ZephyrusController:
    def __init__(self):
        self.ser = None
        self.connected = False

    def connect(self) -> bool:
        """Initialize serial connection"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Arduino bootloader delay
            self.connected = self.ser.is_open
            return self.connected
        except serial.SerialException as e:
            print(f"Connection error: {str(e)}")
            return False

    def send_command(self, angles: Tuple[int, ...]) -> bool:
        """Send servo angles to robotic arm"""
        if not self.connected:
            return False

        if len(angles) != 6:
            raise ValueError("Invalid angle count")

        try:
            cmd = f"s1{angles[0]:03d}s2{angles[1]:03d}s3{angles[2]:03d}" \
                  f"s4{angles[3]:03d}s5{angles[4]:03d}s6{angles[5]:03d}\n"
            self.ser.write(cmd.encode('utf-8'))
            time.sleep(1.5)  # Movement completion delay
            return True
        except serial.SerialException as e:
            print(f"Command failed: {str(e)}")
            return False

    def execute_sequence(self, sequence: Tuple[Tuple[int, ...], ...]) -> bool:
        """Execute a movement sequence"""
        for step in sequence:
            if not self.send_command(step):
                return False
        return True

    def shutdown(self):
        """Cleanup resources"""
        if self.connected:
            self.ser.close()


# --------------------------
# Main Application
# --------------------------
class ZephyrusApp:
    def __init__(self):
        self.vision = ArucoProcessor()
        self.robot = ZephyrusController()
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.running = False

    def run(self):
        """Main execution loop"""
        if not self._initialize():
            return

        self.running = True
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                _, obj_pos = self.vision.process_frame(frame)
                self._handle_object_position(obj_pos)

                cv2.imshow('Zephyrus Control', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self._shutdown()

    def _initialize(self) -> bool:
        """Initialize system components"""
        if not self.robot.connect():
            print("Failed to connect to robot arm")
            return False

        if not self.cap.isOpened():
            print("Failed to initialize camera")
            return False

        self.robot.execute_sequence((ArmPresets.home,))
        return True

    def _handle_object_position(self, position: Optional[Tuple[int, int]]):
        """Handle object detection events"""
        if position and position == GridConfig.target_cell:
            self._execute_pick_sequence()

    def _execute_pick_sequence(self):
        """Full pick-and-place routine"""
        sequence = (
            ArmPresets.pick_approach,
            ArmPresets.pick_grasp,
            ArmPresets.pick_lift,
            ArmPresets.drop_approach,
            ArmPresets.drop_release,
            ArmPresets.home
        )

        if self.robot.execute_sequence(sequence):
            print("Pick-and-place sequence completed successfully")
        else:
            print("Sequence failed - check hardware connections")

    def _shutdown(self):
        """Cleanup resources"""
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        self.robot.shutdown()


if __name__ == "__main__":
    app = ZephyrusApp()
    app.run()
