# Zephyrus: Robotic Arm for Surgical Assistance

Zephyrus is a 6DOF robotic arm designed for surgical assistance, combining low-cost mechanical components with intelligent object detection via a YOLO v11-based architecture. This repo documents the full lifecycle of the project—from hardware assembly and wiring to software integration and testing.

![Zephyrus Arm Assembled](https://github.com/poisonkissedsk/Data/blob/main/Zephyrus/Robotic%20Arm%20Assembly.jpg?raw=true)

---

## Overview

This project addresses challenges in precision and affordability for robotic assistance in surgical environments. Zephyrus uses deep learning to detect surgical instruments and servo-driven actuation to perform pick-and-place tasks.

---

## Hardware Components

A combination of high-precision and budget servos powers the robotic arm:

### Servos

- **Servo G90**
  - Dimensions: 22.2 × 11.8 × 31 mm
  - Torque: 1.8 kgf·cm
  - Speed: 0.1 s/60°
  - Weight: 9g

- **Servo MG996RS**
  - Torque: Up to 11 kgf·cm
  - Metal Gear | Speed: 0.17 s/60°
  - Weight: 55g | 5–6V Input

### Microcontroller

- **Arduino UNO**
  - Flash: 32 KB | 5V Operation
  - Chip: ATMega328P | 16 MHz

### Power Supply

- **MB102 Breadboard PSU**
  - Input: 6.5–12V DC
  - Output: 3.3V/5V
  - Max Current: <700 mA

### Wiring

Wiring is compact and modular for rapid debugging.

![Wiring Layout](https://github.com/poisonkissedsk/Data/blob/main/Zephyrus/Robotic%20Arm%20Wiring.jpg?raw=true)

---

## Deep Learning Workflow

Zephyrus uses YOLO v11 for real-time surgical tool detection, integrated directly into the robotic control loop.

### Dataset

- Source: [Dataset Ninja - Surgical Tools](https://datasetninja.com/labeled-surgical-tools-and-images)
- Images: 2,620  
- Labeled Objects: 3,639  
- Classes: Curved Mayo Scissor, Scalpel, Straight Dissection Clamp, Straight Mayo Scissor

### YOLO v11 Architecture

- **Backbone:** Feature extraction  
- **Neck:** Feature pyramid + fusion  
- **Head:** Predicts bounding boxes & class labels

### Methodology

1. **Data Preprocessing:** Normalization, augmentation, and split
2. **Training:** Custom YOLO v11 model on surgical dataset
3. **Validation:** mAP and IoU metrics evaluated per class
4. **Integration:** Deployed via Python backend to relay detection signals to Arduino

---

## Vision Integration Snapshot

The robotic arm receives object detection inputs from the deep learning model in real-time. Here's a visual of the integration in action:

![Computer Vision Integration](https://github.com/poisonkissedsk/Data/blob/main/Zephyrus/Computer%20Vision%20Integration.jpg?raw=true)


---

## Demo

Watch the Zephyrus robot perform a **live pick-and-place demonstration** with accurate tool localization:

 [Click to Watch the Demo Video](https://github.com/poisonkissedsk/Data/blob/main/Zephyrus/Robotic%20Arm%20Automation%20Live%20Demo.mp4?raw=true)

*(Right-click → Open in new tab if it doesn't autoplay)*
---

## Results & Discussion

Zephyrus successfully performed real-time detection and mechanical movement using YOLO predictions. Results were:

- **Detection Accuracy:** ~89% mAP across all classes.
- **Latency:** ~210ms from detection to actuation.
- **Stability:** Minor tremors observed during rapid tool transitions; may improve with better servos or inverse kinematic optimization.

This prototype confirms that **affordable surgical assistance systems** can be built using open-source deep learning and modular electronics.

---
