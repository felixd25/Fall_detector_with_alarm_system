# Fall Detector with Alarm System
This repository contains the source code and documentation for a fall detection system with an integrated alarm, developed as part of the ECE198 course project.

![Demo: Connect the two microcontrollers to power](https://github.com/user-attachments/assets/f51e8e73-879e-42e3-919e-c6257f04e733)

<p align="center"><em>Demo: Connect the two microcontrollers to power</em></p>


## Overview
The Fall Detector with Alarm System is designed to monitor and detect falls using sensor data, triggering an alarm when a fall is detected. This system aims to assist in ensuring the safety of individuals by providing immediate alerts in case of falls.

## Features
Real-time Fall Detection: Utilizes sensor inputs to monitor movements and detect falls in real-time.

Alarm System: Activates an audible alarm upon detecting a fall to alert nearby individuals.

Efficient Processing: Implemented in C for efficient performance on embedded systems.

## Getting Started
### Prerequisites
Microcontroller or development board compatible with C programming (STM32F401 is used in this demo).

Necessary sensors (e.g., accelerometer) connected to the microcontroller (MPU6050 is used in this demo).

Development environment set up for compiling and uploading C code to the microcontroller (STM32CubeIDE is used in this demo).

### Installation
  1. Clone the repository:
  
  ```
  git clone https://github.com/felixd25/Fall_detector_with_alarm_system.git
  ```
  
  2. Navigate to the project directory:
  
  ```
  cd Fall_detector_with_alarm_system
  ```
  
  3. Compile the source code using your development environment or preferred compiler.
  4. Upload the compiled code to your microcontroller.

### Usage
Once the system is set up and running:

The sensors continuously monitor for movements indicative of a fall.

Upon detecting a fall, the system triggers the alarm to alert others **(Preset: Alarm will be triggered once it exceeds 45 degrees).**

**IMPORTANT: Since the MPU6050 can only detect vertical acceleration and angle, the yaw angle is not accurate. Feel free to add a magnetometer if you need an accurate yaw angle.**

## Contributing
Contributions are welcome! If you have suggestions or improvements, feel free to fork the repository and submit a pull request.


