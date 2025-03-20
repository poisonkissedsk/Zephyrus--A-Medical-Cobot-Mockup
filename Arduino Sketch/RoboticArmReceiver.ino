#include <Servo.h>

// Create servo objects for six servos
Servo waist;       // s1
Servo shoulder;    // s2
Servo elbow;       // s3
Servo wristRoll;   // s4
Servo wristPitch;  // s5
Servo gripper;     // s6

// For storing incoming serial data
String inputString = "";
bool stringComplete = false;

// Setup pins according to the “How To Mechatronics” diagram
// Waist -> Pin 5
// Shoulder -> Pin 6
// Elbow -> Pin 7
// Wrist Roll -> Pin 8
// Wrist Pitch -> Pin 9
// Gripper -> Pin 10

void setup() {
  Serial.begin(9600);

  waist.attach(5);
  shoulder.attach(6);
  elbow.attach(7);
  wristRoll.attach(8);
  wristPitch.attach(9);
  gripper.attach(10);

  // (Optional) Move servos to initial positions
  waist.write(90);
  shoulder.write(150);
  elbow.write(35);
  wristRoll.write(140);
  wristPitch.write(85);
  gripper.write(80);
}

void loop() {
  // Collect incoming serial data
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }

  // Once we get a newline, parse the command
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// We expect commands like: s1090s2150s3035s4140s5085s6080
void parseCommand(String cmd) {
  parseAndWriteServo(cmd, "s1", waist);
  parseAndWriteServo(cmd, "s2", shoulder);
  parseAndWriteServo(cmd, "s3", elbow);
  parseAndWriteServo(cmd, "s4", wristRoll);
  parseAndWriteServo(cmd, "s5", wristPitch);
  parseAndWriteServo(cmd, "s6", gripper);
}

void parseAndWriteServo(String cmd, String token, Servo &servoObj) {
  int idx = cmd.indexOf(token);
  // If found and there's enough room for a 3-digit angle
  if (idx != -1 && cmd.length() >= idx + 5) {
    String angleStr = cmd.substring(idx + 2, idx + 5); // 3 digits
    int angle = angleStr.toInt();
    servoObj.write(angle);
  }
}
