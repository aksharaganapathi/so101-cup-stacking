# CS-6341 Project — Group 14

Team:

- Akshara
- Aditya
- Dhanush
- Sid

## Overview

This repository contains an **Automated Vision-Based Pick and Place System** for a robotic arm. The system uses computer vision to detect objects, calculate pick positions, and execute coordinated pick-and-place operations.

### Key Components

- **Vision System** (`vision_system.py`) — Core automated picking & placing with red object detection, calibration, and place position management
- **Robot Control** (`utils/urdf_ik_fixed.py`) — Inverse kinematics, motion planning, and gripper control
- **Camera Utilities** (`camera/stream.py`, `camera/mjpeg_camera_sync.py`) — Camera streaming and ROS2 integration
- **Motion Scripts** (`motion/capture_current_pose.py`, `motion/go_to_waypoints.py`) — Waypoint recording and playback
- **AprilTag Tools** (`apriltags/tag_distance_estimator.py`) — Distance estimation from AprilTag detections

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

For ROS2-dependent features, also install via your ROS2 package manager:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

### 2. Run the Automated Pick & Place System

```bash
python vision_system.py
```

This launches an interactive menu with options to:
- **Run automated pick and place** — Continuously picks objects from the entire vision area and places them in a predefined sequence
- **Manual control** — Test individual operations (single pick, camera feed, calibration)
- **Calibrate pixels per meter** — Interactive tool to set camera calibration
- **Exit**

### 3. Camera Streaming (Optional)

For a live MJPEG stream server:

```bash
python camera/stream.py
```

Access at `http://localhost:8090/`

## System Features

### Vision System (`vision_system.py`)

The `VisionSystem` class handles:

- **Red Object Detection** — HSV-based color filtering with contour analysis
- **Camera Calibration** — Interactive pixel-to-meter conversion tool
- **Pickup Zone Management** — Configurable regions for object detection
- **Place Position Sequencing** — Fixed list of 4 predefined place positions
- **Camera Stabilization** — Warm-up period before image capture to reduce noise

#### Key Methods

| Method | Description |
|--------|-------------|
| `detect_red_objects()` | Find all red objects in frame; returns list with pixel positions and areas |
| `pixel_to_robot_offset()` | Convert pixel coordinates to robot workspace offsets |
| `get_robot_target_position()` | Calculate absolute pick position from vision pose and pixel detection |
| `capture_stable_image()` | Capture frame after camera stabilization |
| `find_best_pickup_object()` | Select largest object in pickup zone for picking |
| `get_next_place_position()` | Retrieve next available place location |
| `mark_place_position_used()` | Move to the next place position after successful placement |

#### Automated Workflow

The `automated_pick_and_place()` function executes:

1. Connect to robot and initialize vision system
2. Move to **vision pose** (calibrated viewing angle)
3. Capture stabilized image
4. Detect red objects in pickup zone
5. Calculate pick position from pixel coordinates
6. Execute pick sequence:
   - Move to approach height
   - Open gripper
   - Move to grasp height
   - Close gripper
   - Lift item
7. Execute place sequence:
   - Move to next place position
   - Lower to place height
   - Open gripper to release
   - Lift from place spot
8. Repeat until no objects found or all place positions filled

### Robot Control (`utils/urdf_ik_fixed.py`)

Handles robot kinematics and motion:

- **Inverse Kinematics** — Compute joint angles from target end-effector pose
- **Motion Planning** — Smooth trajectory generation between waypoints
- **Gripper Control** — Open/close operations with smooth interpolation
- **Preset Poses** — Home and vision poses

#### Key Functions

| Function | Purpose |
|----------|---------|
| `setup_robot_and_kinematics()` | Initialize robot connection and kinematics solver |
| `go_to_xyz_with_ik()` | Move to XYZ position using inverse kinematics |
| `go_to_pose_with_ik()` | Move to full pose (position + orientation) |
| `go_home()` | Return to home position |
| `go_vision_pose()` | Move to calibrated vision observation pose |
| `open_gripper()` / `close_gripper()` | Gripper control |
| `get_current_joint_vector()` | Read current joint angles |
| `safe_shutdown()` | Gracefully shutdown robot |

## Calibration

### Pixels Per Meter

To calibrate camera resolution:

1. From the main menu, select **Option 3**
2. Place a ruler or known-dimension object in the camera view
3. Click two points with a known distance between them
4. Enter the real distance in centimeters
5. The tool computes `pixels_per_meter` and saves to `pixel_calibration.json`

### Place Positions

Edit the `place_positions` list in the `VisionSystem.__init__()` method to customize where objects are placed:

```python
self.place_positions = [
    {'x': -0.02, 'y': -0.3, 'z': 0.15},   # Position 1
    {'x': 0.02, 'y': -0.27, 'z': 0.15},  # Position 2
    {'x': -0.007, 'y': -0.28, 'z': 0.21},# Position 3
    {'x': 0.1, 'y': -0.25, 'z': 0.1331}, # Position 4
]
```

## File Structure

```
.
├── README.md
├── requirements.txt
├── vision_system.py              # Main automated pick & place system
├── apriltags/
│   └── tag_distance_estimator.py # ROS2 AprilTag distance node
├── camera/
│   ├── mjpeg_camera_sync.py      # ROS2 MJPEG camera bridge
│   └── stream.py                 # Standalone MJPEG stream server
├── motion/
│   ├── capture_current_pose.py   # Record waypoint poses
│   ├── go_to_waypoints.py        # Execute waypoint sequences
│   └── move_arm.py               # Manual arm control & calibration
└── utils/
    ├── cam_test.py               # Basic camera red detection test
    ├── kinematics.py             # Kinematics wrapper (placo)
    └── urdf_ik_fixed.py          # Robot control & IK functions
```

## Configuration

### Environment Variables / Paths

- **URDF Path**: `~/ros2_ws/src/dora-bambot/URDF/so101.urdf`
- **Robot Port**: `/dev/ttyACM0`
- **Robot ID**: `armzilla`

Modify these in `utils/urdf_ik_fixed.py` if using a different setup.

### Camera Parameters

- **Default Resolution**: 1080p (1920×1080)
- **Red Detection Ranges** (HSV):
  - Lower red: H=[0–10], S=[100–255], V=[100–255]
  - Upper red: H=[160–179], S=[100–255], V=[100–255]
- **Minimum Object Area**: 500 pixels²
- **Camera Stabilization Time**: 2.0 seconds

## Usage Examples

### Run Full Automated System

```bash
python vision_system.py
# Select: 1 (Automated pick and place)
# Press Enter to start
```

### Manual Testing

```bash
python vision_system.py
# Select: 2 (Manual control)
# From submenu:
#   1 - Single pick and place (manual target selection)
#   2 - Show camera feed
#   3 - Calibrate camera
#   4 - Go to vision pose
#   5 - Go home
#   6 - Run automated system
```

### Test Camera Detection

```bash
python utils/cam_test.py
# Shows red cube detection in real-time
```

### Record Waypoints

```bash
python motion/capture_current_pose.py
# Use 't' to toggle torque, Enter to capture pose, 'q' to quit
```

## Dependencies

### Python Packages (pip)

- `opencv-python` — Computer vision & image processing
- `numpy` — Numerical computations
- `pyyaml` — Configuration file parsing
- `flask` — Web server for camera streaming
- `apriltag` — AprilTag detection
- `lerobot` — Robot driver interface (SO-101 Follower)

### ROS2 Packages

- `rclpy` — ROS2 Python client
- `sensor_msgs` — Image and camera info messages
- `cv_bridge` — OpenCV ↔ ROS image conversion
- `apriltag_msgs` — AprilTag detection messages (if available)

### External Libraries

- `placo` — Kinematics solver with URDF support
- **Hardware**: SO-101 Follower arm, USB camera

## Troubleshooting

### Camera Not Opening
- Check USB camera connection: `ls /dev/video*`
- Verify `CAM_INDEX` in `camera/stream.py` (default: 0)
- Check camera permissions: may need `sudo` or add user to `video` group

### IK Solver Failing
- Ensure URDF path is correct
- Check robot joint limits in URDF
- Verify current pose is not in collision

### Gripper Not Responding
- Confirm robot connection with `python utils/urdf_ik_fixed.py`
- Check gripper servo power
- Verify gripper joint name in `JOINT_NAME_MAP`

### Poor Object Detection
- Lighting conditions affect HSV thresholds; adjust `lower_red1`, `upper_red1`, etc. in `detect_red_objects()`
- Ensure objects are sufficiently large (>500 px²)
- Re-calibrate `pixel_calibration.json` if camera moved

## Notes

- All coordinates are in **meters** unless otherwise noted
- Robot operates in **degrees** for joint angles
- Vision pose should be a stable, known reference point for accurate calibration
- Place positions are **fixed sequences**; objects always fill in order
- Press `Ctrl+C` to safely interrupt and return to home

## License & Contributing

Please add a license if you want to open-source this repository. For collaboration, use branches and make pull requests to the `main` branch.

## Contact

Group 14 — CS-6341 Project