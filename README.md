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
⚙️ Install and Run
Follow these steps to set up and run the project.

---
Step 1: Create ROS 2 Workspace & Clone
First, create a ROS 2 workspace and clone your repository into it.

# Create the directory structure
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone your repository here (replace with your actual URL)
# git clone <YOUR_REPOSITORY_URL>

