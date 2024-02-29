# SINGULAR FACE TRACKING ROBOT

## Overview
The Singular Face Tracking Robot project aimed to create a robot that identifies and follows a specific face it recognizes, regardless of the presence of other people around. This innovative project consists of two main components: a training module to train a model using a video of the recognized person, and a tracking module that utilizes the trained model to identify and follow the designated face with the help of two motors.

## Requirements
- **Programming Language**: The project is developed in C++.
- **Libraries and Tools**:
  - OpenCV 4: For image processing and face recognition functionalities.
  - Arduino Software (IDE): To upload the necessary code to the Arduino Uno board.
  
## Installation and Setup
1. **OpenCV 4 Installation**: Ensure OpenCV 4 is installed on your system for the project to function correctly.
2. **Arduino Software Setup**:
   - Download and install the Arduino IDE from the official website.
   - Connect your Arduino Uno board to your computer and select the corresponding port in the Arduino IDE.
   - Upload the "Servo_Parse_int.ino" program to your Arduino Uno board to control the motors.

## Training the Model
1. **Preparing the Training Video**:
   - In the "training" folder, place a 15-second video of the person to be recognized and name it "video.mp4".
   - Modify line 27 of the "training.cpp" file to reflect the exact duration of the video.
2. **Generating the Model**:
   - In the terminal, navigate to the "training" folder and execute the makefile to compile the program.
   - Run the compiled program to generate a model named "modele.yml" and place this file in the "prog" directory.

## Running the Tracking Module
1. **Setting Up the Camera and Arduino Port**:
   - In the "prog.cpp" file, modify line 138 to set the correct USB port for the camera (with "0" typically representing the webcam).
   - On line 86, specify the previously identified Arduino port.
2. **Launching the Program**:
   - Compile and run the program by executing the makefile in the "prog" directory.
   - Once started, the robot will begin to identify and follow the recognized face using the camera and motors.

## Demonstration Video
To see the Singular Face Tracking Robot in action, check out our demonstration video on YouTube:

[![Watch the video](https://img.youtube.com/vi/VA9WC24NqdE/0.jpg)](https://www.youtube.com/watch?v=VA9WC24NqdE)


## Authors
- Pierre&Louis
