# Real-Time Object Detection Node in ROS 2 (YOLOv8)

## Project Overview
This project involves creating a complete robotics perception system in **ROS 2**. The goal is to develop a Python-based **ROS 2** node that:
- Subscribes to a camera’s video stream  
- Performs real-time object detection with **YOLOv8**  
- Publishes structured detection data (labels, confidence, bounding boxes) to a custom **ROS 2** topic  
- Displays detections in real-time on the video feed  

## Core Tasks
1. Set up the **ROS 2** development environment
2. Define and build a custom **ROS 2** message interface for detection results
3. Implement the Python detection node that subscribes to the camera feed and runs the detection
4. Perform **YOLOv8** inference on incoming video frames from the camera
5. Visualize detection results (bounding boxes, labels) on the camera feed
6. Test the system and validate its performance

## Expected Outcome
By following these instructions, you should be able to:
- Run **YOLOv8** object detection on a camera stream in real-time
- Publish the detection results (labels, confidence, bounding box coordinates) via a custom **ROS 2** topic
- Visualize the camera feed with bounding boxes drawn around detected objects

## Used Technologies
- **Robotics Middleware:** ROS 2 Humble (supports Ubuntu 22.04)
- **Programming Language:** Python
- **Libraries:** OpenCV, CvBridge, Ultraytics (YOLOv8)
- **AI Model:** YOLOv8 (pre-trained weights auto-download)

## Install and Run

***Follow these steps to set up and run the project.***

### Step 1: Create ROS 2 Workspace & Clone the Repository
First, create a **ROS 2 workspace** on your machine:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
Then, clone the repository into the workspace:

```bash
git clone https://github.com/Kteii/ros2-realtime-object-detection.git
cd ..
```
### Step 2: Install System Dependencies
_Update system packages_
```bash
sudo apt update
```
_Install camera driver_
```bash
sudo apt install ros-humble-v4l2-camera
```
_Install image processing libraries_
```bash
sudo apt install python3-opencv ros-humble-cv-bridge
```
### Step 3: Install Package Dependencies (rosdep) + Python deps
Now, install **ROS package dependencies** using ***rosdep*** and **Python dependencies**:

_Install ROS package dependencies (from package.xml)_
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
_(Optional) Create Python virtual environment and install required Python packages_
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip
pip install -r src/ros2-realtime-object-detection/requirements.txt
```

### Step 4: Build the Workspace
Once all dependencies are installed, build the workspace using ***colcon***:
```bash
colcon build --symlink-install
source install/setup.bash
```
### Step 5: Run in 3 Terminals
After building the workspace, you will need to open three terminals to run the system:

**Terminal 1 (Camera)**

In Terminal 1, run the camera driver to start streaming the video feed:

***Note:*** _Assuming this is the terminal you have been working on since the first step_
```bash
ros2 run v4l2_camera v4l2_camera_node
```
**Terminal 2 (Detector)**

In Terminal 2, run your object detection node to process each video frame:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_vision_node object_detector
```
**Terminal 3 (Output)**

In Terminal 3, subscribe to the ***/object_detections*** topic to see the output data:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /object_detections
```

Once you have completed these steps, you will have a fully functional **ROS 2** package that uses **YOLOv8** to perform real-time object detection from a live camera feed. The results are published on a custom **ROS 2** topic and can be visualized with bounding boxes in real-time.
