import tkinter as tk
import serial
import time

# Change this to match your Arduino’s port, e.g., 'COM3' on Windows or '/dev/ttyACM0' on Linux/Mac
ARDUINO_PORT = 'COM5'
BAUD_RATE = 9600

class MechatronicsArmGUI:
    def __init__(self, master):
        self.master = master
        master.title("Mechatronics Robotic Arm Control")

        # Try opening the Arduino serial port
        try:
            self.ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # give Arduino a moment to reset
        except:
            print("ERROR: Could not open serial port. Check ARDUINO_PORT or your USB connection.")
            self.ser = None

        # Define each servo's name and initial angle
        # (Matches the Arduino code's initial positions)
        servo_info = [
            ("Waist (s1)", 90),
            ("Shoulder (s2)", 150),
            ("Elbow (s3)", 35),
            ("Wrist Roll (s4)", 140),
            ("Wrist Pitch (s5)", 85),
            ("Gripper (s6)", 80)
        ]

        self.slider_vars = []

        for i, (label, init_val) in enumerate(servo_info):
            # Label
            lbl = tk.Label(master, text=label)
            lbl.grid(row=i, column=0, padx=10, pady=5, sticky="w")

            # IntVar for the slider
            var = tk.IntVar(value=init_val)
            self.slider_vars.append(var)

            # Slider (Scale)
            scl = tk.Scale(master, from_=0, to=180, orient=tk.HORIZONTAL,
                           variable=var, length=300)
            scl.grid(row=i, column=1, padx=10, pady=5)

        # Schedule the first update
        self.update_interval_ms = 200
        self.send_command()

    def send_command(self):
        # Build a command string "s1XYZs2XYZ..."
        angles = [v.get() for v in self.slider_vars]  # read each slider’s IntVar
        # s1 => waist, s2 => shoulder, s3 => elbow, s4 => wristRoll, s5 => wristPitch, s6 => gripper
        cmd = ("s1{:03d}s2{:03d}s3{:03d}s4{:03d}s5{:03d}s6{:03d}"
               .format(*angles))

        print("Sending:", cmd)

        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode('utf-8'))

        # Schedule the next call
        self.master.after(self.update_interval_ms, self.send_command)


def main():
    root = tk.Tk()
    app = MechatronicsArmGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
