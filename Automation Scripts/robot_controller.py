"""
Robotic Arm Serial Controller
Handles low-level communication with Zephyrus
"""

import serial
import time
from typing import List, Optional


class ZephyrusController:
    def __init__(self, port: str = 'COM4', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.joint_limits = {
            'base': (0, 180),
            'shoulder': (15, 165),
            'elbow': (0, 180),
            'wrist': (0, 180),
            'wrist_rot': (0, 180),
            'gripper': (73, 0)
        }

    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            self.connection = serial.Serial(
                self.port,
                self.baudrate,
                timeout=5
            )
            time.sleep(2)  # Arduino bootloader delay
            return self.connection.is_open
        except serial.SerialException as e:
            print(f"Connection error: {str(e)}")
            return False

    def send_angles(self, angles: List[int], speed: int = 200) -> bool:
        """Send joint angles to robotic arm"""
        if not self.connection or not self.connection.is_open:
            print("No active connection")
            return False

        if len(angles) != 6:
            print("Invalid angle count")
            return False

        # Apply hardware-specific inversions
        adjusted = [
            180 - angles[0],  # Base inversion
            angles[1],  # Shoulder
            angles[2],  # Elbow
            180 - angles[3],  # Wrist inversion
            angles[4],  # Wrist rotation
            angles[5]  # Gripper
        ]

        command = f"P{','.join(map(str, adjusted))},{speed}\n"

        try:
            self.connection.write(command.encode())
            time.sleep(0.1)  # Minimum command interval
            return True
        except serial.SerialException as e:
            print(f"Command failed: {str(e)}")
            return False

    def home(self, speed: int = 20) -> None:
        """Return to home position"""
        self.send_angles([90, 90, 90, 90, 90, 73], speed)

    def disconnect(self) -> None:
        """Cleanup serial connection"""
        if self.connection and self.connection.is_open:
            self.connection.close()


# Example usage
if __name__ == "__main__":
    arm = ZephyrusController()

    if arm.connect():
        arm.home()
        arm.send_angles([45, 100, 120, 30, 90, 73])
        arm.disconnect()
