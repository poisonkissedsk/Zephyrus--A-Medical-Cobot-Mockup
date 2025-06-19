"""
Robotic Arm Kinematics Solver
Handles forward/inverse kinematics and backlash compensation
"""

import math
import numpy as np
from typing import List, Tuple


class ArmKinematics:
    def __init__(self):
        self.dimensions = {
            'base_height': 71.5,  # mm
            'upper_arm': 125,
            'forearm': 125,
            'wrist_offset': 192  # 60+132mm
        }
        self.backlash_compensation = {
            'base_cw': 8,
            'base_ccw': np.linspace(0, 14, 135)
        }

    def forward_kinematics(self, angles: List[float]) -> Tuple[float, float, float]:
        """Calculate end effector position from joint angles"""
        theta1, theta2, theta3 = np.radians(angles[:3])

        x = (self.dimensions['upper_arm'] * math.cos(theta2) +
             self.dimensions['forearm'] * math.cos(theta2 + theta3)) * math.cos(theta1)

        y = (self.dimensions['upper_arm'] * math.cos(theta2) +
             self.dimensions['forearm'] * math.cos(theta2 + theta3)) * math.sin(theta1)

        z = (self.dimensions['base_height'] +
             self.dimensions['upper_arm'] * math.sin(theta2) +
             self.dimensions['forearm'] * math.sin(theta2 + theta3) -
             self.dimensions['wrist_offset'])

        return x, y, z

    def inverse_kinematics(self, x: float, y: float, z: float) -> List[float]:
        """Calculate joint angles for target position"""
        z_eff = z + 15  # Backlash compensation
        r_hor = math.hypot(x, y)
        r_total = math.hypot(r_hor, z_eff - self.dimensions['base_height']) * 1.02

        # Base angle calculation
        theta_base = 90 - math.degrees(math.atan2(x, y)) if y != 0 else 180 if x < 0 else 0

        # Geometric solution for arm angles
        try:
            alpha1 = math.acos((r_total - self.dimensions['forearm']) /
                               (self.dimensions['upper_arm'] + self.dimensions['wrist_offset']))
        except ValueError:
            raise ValueError("Target position unreachable")

        theta_shoulder = math.degrees(alpha1)
        alpha3 = math.asin(math.sin(alpha1) *
                           (self.dimensions['wrist_offset'] - self.dimensions['upper_arm']) /
                           self.dimensions['forearm'])

        theta_elbow = 90 - math.degrees(alpha1) + math.degrees(alpha3) + 5
        theta_wrist = 90 - math.degrees(alpha1) - math.degrees(alpha3) + 5

        # Z-axis compensation
        if z != self.dimensions['base_height']:
            theta_shoulder += math.degrees(math.atan(
                (z - self.dimensions['base_height']) / r_total
            ))

        return [
            round(theta_base),
            round(theta_shoulder),
            round(theta_elbow),
            round(theta_wrist)
        ]

    def compensate_backlash(self, current_angles: List[int], previous_angles: List[int]) -> List[int]:
        """Apply backlash compensation to base joint"""
        delta = current_angles[0] - previous_angles[0]

        if delta > 1 and current_angles[0] > 45:
            index = int(round(current_angles[0] - 46))
            current_angles[0] += self.backlash_compensation['base_ccw'][index]
        elif delta < -1:
            current_angles[0] -= self.backlash_compensation['base_cw']

        return current_angles


# Example usage
if __name__ == "__main__":
    solver = ArmKinematics()
    target_pos = (100, 50, 150)
    angles = solver.inverse_kinematics(*target_pos)
    print(f"Target position {target_pos} requires angles: {angles}")
