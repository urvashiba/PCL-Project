# PCL-Project

## Overview
The PCL-Project is a robotics-focused application that combines 3D object detection and point cloud preprocessing. It utilizes **YOLOv8** for detecting objects in 2D images and **PCL (Point Cloud Library)** for processing 3D point cloud data. This project is designed for applications in robotics, autonomous vehicles and 3D scene reconstruction.

## Features
- **Object Detection**:
  - Uses YOLOv8 to detect objects in 2D images.
  - Outputs bounding boxes, confidence scores and class labels for detected objects.
- **Point Cloud Preprocessing**:
  - Applies a voxel grid filter to downsample 3D point clouds.
  - Outputs a filtered point cloud for further processing or visualization.

## Use Cases
- Robotics and autonomous systems.
- 3D scene reconstruction and mapping.
- Object detection and tracking in 3D environments.

## Requirements
- **ROS (Robot Operating System)**: For managing nodes and communication.
- **PCL (Point Cloud Library)**: For 3D point cloud processing.
- **Python 3.8**: For running the YOLOv8-based object detection.
- Dependencies listed in `requirements.txt`.

## Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/urvashiba/PCL-Project.git
   cd PCL-Project

**Install Python dependencies**: pip install -r requirements.txt

**Build the ROS package**: catkin_make

## Usage :

**Launch the object detection node**: 
```bash
roslaunch pcl_ai_3d_detection detection.launch```

**Run the point cloud preprocessing node**: 
rosrun pcl_ai_3d_detection pcl_preprocessing_node input.pcd filtered_output.pcd

# possible usecase:
we can train a Custom YOLOv8 Model:

If you want to detect objects not included in the COCO dataset, you can train YOLOv8 on a custom dataset. This requires a labeled dataset with bounding boxes for your custom objects.
