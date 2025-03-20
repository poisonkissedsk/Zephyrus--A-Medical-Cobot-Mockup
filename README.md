# 🤖 Zephyrus: Robotic Arm for Surgical Assistance

Zephyrus is an innovative project integrating a robotic arm with a deep learning-based object detection system to assist in surgical procedures. This repository contains the code, documentation, and workflow details for both the hardware setup and the YOLO v11-based deep learning architecture that drives the system.

---

## 🔍 Overview

The project aims to overcome limitations in surgical and industrial settings by providing an affordable, reliable, and precise robotic arm solution. The system uses real-time object detection to identify surgical tools, ensuring enhanced accuracy and safety during operations.

---

## 📝 Problem Statement

- **💰 Cost & Reliability:** Address high costs and reliability issues in surgical environments.
- **🎯 Precision:** Ensure precise control mechanisms required for surgical assistance.
- **💡 Affordability:** Deliver a cost-effective solution without compromising on performance.

---

## 🛠 Hardware Components

The project integrates several hardware components to build the robotic arm:

- **Servo G90**
  - 📏 **Dimensions:** 22.2 x 11.8 x 31 mm (approx.)
  - ⚖️ **Weight:** 9 g 
  - 💪 **Stall Torque:** 1.8 kgf.cm
  - ⏱ **Operating Speed:** 0.1 s/60°
  - 🔌 **Operating Voltage:** 4.8V (~5V)
  - 🎚 **Dead Band Width:** 10 µs
  - 🌡 **Temperature Range:** 0°C – 55°C

- **Servo MG996RS**
  - 🔌 **Operating Voltage:** +5V typically
  - ⚡ **Current:** 2.5A (6V)
  - 💪 **Stall Torque:** 9.4 kgf·cm (at 4.8V); Maximum: 11 kgf·cm (6V)
  - ⏱ **Operating Speed:** 0.17 s/60°
  - ⚙️ **Gear Type:** Metal
  - 🔄 **Rotation Range:** 0°-180°
  - ⚖️ **Weight:** 55 g

- **HC-05**
  - 📡 **Typical Sensitivity:** 80 dBm
  - 🔋 **Transmit Power:** Up to +4 dBm RF
  - 🔌 **Operating Voltage:** 1.8V (with 1.8–3.6V I/O)
  - ⌨️ **Default Baud Rate:** 38400 (supports additional rates like 9600, 19200, 57600, 115200, 230400, 460800)

- **Arduino UNO**
  - 🧠 **Processor:** ATMega328P
  - ⏱ **CPU Speed:** Up to 16 MHz
  - 💾 **Flash Memory:** 32kB
  - 🔌 **Operating Voltage:** 2.7-5.5V

- **MB102 PSU**
  - 🔌 **Input Voltage:** 6.5-12 V (DC) or 5V via USB
  - ⚡ **Output Voltage:** Switchable between 3.3V and 5V
  - 🔋 **Maximum Output Current:** <700 mA
  - 🔗 **Features:** Onboard connectors for external devices

---

## 🧠 Deep Learning Workflow

The deep learning component leverages a YOLO v11-based architecture for real-time object detection. The workflow can be summarized as follows:

### 📂 Dataset

- **Source:** Labeled Surgical Tools and Images – [Dataset Ninja](https://datasetninja.com/labeled-surgical-tools-and-images)
- **Details:**
  - 📸 **Images:** 2,620
  - 🔍 **Labeled Objects:** 3,639
  - 🏷 **Classes:** Curved Mayo Scissor, Scalpel, Straight Dissection Clamp, Straight Mayo Scissor

### 🧩 YOLO v11 Architecture

YOLO v11 divides the detection process into three main segments:

- **Backbone:** Extracts essential features from input images.
- **Neck:** Aggregates multi-scale features to support detection.
- **Head:** Performs the final object detection, outputting bounding boxes and class probabilities.

### 🛤 Methodology

1. **Data Preparation:** 
   - Preprocess images and labels.
   - Split data into training and validation sets.
2. **Model Training:**
   - Fine-tune YOLO v11 on the surgical tools dataset.
   - Optimize hyperparameters to improve detection accuracy.
3. **Evaluation:**
   - Validate the model using standard object detection metrics.
4. **Deployment:**
   - Integrate the trained model with the robotic arm system.
   - Enable real-time detection during surgical procedures.

### 📅 Timeline

The project was executed in distinct phases—from hardware assembly to software integration—culminating in a live demo of the system.

---

## 🚀 Installation & Usage

### 📋 Prerequisites

- **Python 3.x**  
- Required Python libraries (see `requirements.txt`), which may include:
  - TensorFlow or PyTorch
  - OpenCV
  - NumPy

### ⚙️ Setup

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/srish-cmd/zephyrus.git
   cd zephyrus
