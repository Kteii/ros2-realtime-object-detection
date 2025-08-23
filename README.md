## 📌 Project Overview
This project involves creating a complete robotics perception system in ROS 2.  
The goal is to develop a Python-based ROS 2 node that:

- Subscribes to a camera’s video stream  
- Performs real-time object detection with **YOLOv8**  
- Publishes structured detection data (labels, confidence, bounding boxes) to a custom ROS 2 topic  
- Displays detections in real-time on the video feed  

---

## Core Tasks
1. Set up the ROS 2 development environment  
2. Define and build a custom ROS 2 message interface  
3. Write the Python detection node from scratch  
4. Implement YOLOv8 inference on incoming frames  
5. Visualize bounding boxes on the camera feed  
6. Test and validate the package  

---

## Expected Outcome
A fully functional ROS 2 package featuring:  
- Real-time YOLOv8 object detection from a camera feed  
- Detection results published via a custom ROS 2 topic  
- A visualization window with bounding boxes  

---

## Technologies to be Used
-g high-performance librariROS 2 Humble  
-erview
This proPython  
-ect Overview
ThiOpenCV, CvBridge, Ultralytics  
-ct involves creYOLOv8

---

## Project Guide: Building the Node

### Step 1: System Prerequisites 
Install the required packages:

```bash
# Update packages
sudo apt update

# Camera driver
sudo apt install ros-humble-v4l2-camera

# Image processing
sudo apt install python3-opencv ros-humble-cv-bridge

# AI libraries
pip install ultralytics
pip install numpy==1.26.4 --force-reinstall

```

### Step 2: Create the Workspace & Custom Messages 

### Step 3: Create the Vision Node Package

### Step 4: Write object_detector.py 

### Step 5: Build and Run

```bash

# Build
cd ~/ros2_ws
colcon build

# Source
source ~/ros2_ws/install/setup.bash

Run in 3 terminals:

Terminal 1 (Camera):

ros2 run v4l2_camera v4l2_camera_node

Terminal 2 (Detector):

ros2 run my_vision_node object_detector

Terminal 3 (Output):

ros2 topic echo /object_detections
