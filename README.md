# CS-6341 Project — Group 14

Team:

- Akshara
- Aditya
- Dhanush
- Sid

Overview:

This repository contains code used for the CS-6341 project. It includes:

- Camera streaming utilities (`camera/stream.py`, `camera/mjpeg_camera_sync.py`)
- AprilTag distance estimator (`apriltags/tag_distance_estimator.py`)
- Motion capture and waypoint scripts (`motion/capture_current_pose.py`, `motion/go_to_waypoints.py`)

Quick start:

1. Install system/ROS2 dependencies (see notes below).
2. Install Python packages from `requirements.txt` using `pip install -r requirements.txt`.
3. To run the MJPEG stream server (dev/test):
   - `python camera/stream.py`
4. To bridge the MJPEG stream into ROS2 topics (requires ROS2 and cv_bridge):
   - run the node `apriltags/mjpeg_camera_sync.py` under a ROS2 Python package or execute with `ros2 run` after packaging.
5. To estimate AprilTag distances, run the `tag_distance_estimator` ROS2 node (the file is in `apriltags/`).
6. For robot motion, use the scripts in `motion/`. These expect the `lerobot` driver and a physical robot connection.

Notes / Dependencies:

- Python packages (install via pip): see `requirements.txt`.
- ROS2 packages (install via your ROS2 distro's package manager / rosdep): `rclpy`, `sensor_msgs`, `cv_bridge`, and any AprilTag message packages (`apriltag_msgs`).
- Some hardware-specific packages (e.g. `lerobot`) may need to be installed separately or are part of the robot SDK.

License & Contributing:

Please add a license if you want to open-source this repository. For collaboration, use branches and make pull requests to the `main` branch.

Contact:

Group 14
