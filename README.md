# CS-6341 Project — Group 14

Team: Akshara, Aditya, Dhanush, Sid

## Overview
Automated vision-based pick-and-place for the SO-101 robot arm. The core logic now lives in [`VisionSystem`](vision_system.py) inside [vision_system.py](vision_system.py), which handles camera setup, red-object detection, pixel-to-robot mapping, and the end-to-end pick/place workflow.

## Project Layout
- [vision_system.py](vision_system.py) — Main entry point and full vision + pick/place pipeline (menu UI, calibration, camera feed, automated stacking).
- [utils/urdf_ik.py](utils/urdf_ik.py) — Robot setup, IK helpers, gripper control, and preset poses.
- [utils/kinematics.py](utils/kinematics.py) — Placo-based forward/inverse kinematics wrapper.
- [utils/cam_test.py](utils/cam_test.py) — Simple HSV red-cube detector for camera testing.
- [motion/capture_current_pose.py](motion/capture_current_pose.py) — Teach-and-record waypoints with torque toggle.
- [motion/go_to_waypoints.py](motion/go_to_waypoints.py) — Play back recorded waypoints with smooth interpolation.
- [motion/move_arm.py](motion/move_arm.py) — Manual pose capture and calibration helper.

## Quick Start
1) Install deps:
```bash
pip install -r requirements.txt
```
2) Run the main system:
```bash
python vision_system.py
```

Menu options:
- **1** Automated Stacking (pick & place loop)
- **2** Show Camera Feed (moves to vision pose first)
- **3** Calibrate Pixels Per Meter (interactive two-point measurement)
- **4** Exit

## Key Vision Features
- Camera setup at 1080p with region updates.
- Red-object detection with HSV masking, contour filtering, and pickup-zone tagging (`detect_red_objects`).
- Pixel-to-robot offset conversion with perspective correction (`pixel_to_robot_offset`).
- Target computation from the vision pose (`get_robot_target_position`).
- Automated stacking loop (`automated_stacking`) with sequenced place positions and safety return to home.
- Interactive pixel-per-meter calibration with on-image point marking (`measure_pixels_per_meter_with_validation`).
- Live camera feed overlay (`show_camera_feed`).

## Robot Control Highlights
Provided by [`utils/urdf_ik.py`](utils/urdf_ik.py):
- [`setup_robot_and_kinematics`](utils/urdf_ik.py) to connect and initialize IK.
- [`go_home`](utils/urdf_ik.py), [`go_vision_pose`](utils/urdf_ik.py), [`go_to_xyz_with_ik`](utils/urdf_ik.py) for motion.
- [`open_gripper`](utils/urdf_ik.py) / [`close_gripper`](utils/urdf_ik.py) for end-effector control.
- [`safe_shutdown`](utils/urdf_ik.py) for cleanup.

## Calibration
- Run menu option **3** to capture two points with known distance; saves `pixel_calibration.json`.
- Default orientation mapping and pixels-per-meter fallback are loaded automatically in [`VisionSystem.load_calibration`](vision_system.py).

## Notes
- Default camera index: 0; default resolution: 1920×1080.
- Place positions are defined in [`VisionSystem.__init__`](vision_system.py) and consumed in the stacking loop.
- Press Ctrl+C during automated stacking to safely stop; system attempts to return home and disconnect.