import os
import time
import traceback
import numpy as np
from kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

JOINT_NAME_MAP = {
	'1': 'shoulder_pan.pos',
	'2': 'shoulder_lift.pos',
	'3': 'elbow_flex.pos',
	'4': 'wrist_flex.pos',
	'5': 'wrist_roll.pos',
	'6': 'gripper.pos',
}

URDF_PATH = os.path.expanduser("~/ros2_ws/src/dora-bambot/URDF/so101.urdf")

R_DOWN = np.array([
	[1, 0, 0],
	[0, 0, 1],
	[0, -1, 0]
])

FIXED_WRIST_ROLL = -1.27

def get_current_joint_vector(robot: SO101Follower, kin: RobotKinematics) -> np.ndarray:
	obs = robot.get_observation()
	q = []
	for name in kin.joint_names:
		key = JOINT_NAME_MAP[name]
		q.append(float(obs[key]))
	return np.array(q, dtype=float)


def print_current_ee_pose(robot: SO101Follower, kin: RobotKinematics) -> np.ndarray:
	q = get_current_joint_vector(robot, kin)
	T = kin.forward_kinematics(q)
	pos = T[:3, 3]
	print(f"Current EE position (frame '{kin.target_frame_name}'):")
	print(f" x={pos[0]:.4f}m, y={pos[1]:.4f}m, z={pos[2]:.4f}m")
	return T


def go_to_joint_pose(robot: SO101Follower, target_joints: dict, duration: float = 3.0):
	obs = robot.get_observation()
	joint_keys = list(target_joints.keys())

	q_current = np.array([float(obs.get(k, 0.0)) for k in joint_keys], dtype=float)
	q_target = np.array([float(target_joints[k]) for k in joint_keys], dtype=float)

	steps = max(int(duration * 40), 10)
	dt = duration / steps

	for i in range(1, steps + 1):
		alpha = i / steps
		q = (1.0 - alpha) * q_current + alpha * q_target

		action = {k: float(val) for k, val in zip(joint_keys, q)}

		if "gripper.pos" not in action:
			action["gripper.pos"] = float(obs.get("gripper.pos", 0.0))

		robot.send_action(action)
		time.sleep(dt)


def go_home(robot: SO101Follower, duration: float = 3.0):
	HOME_JOINTS = {
		'shoulder_pan.pos': -1.4945054945054945,
		'shoulder_lift.pos': -32.747252747252745,
		'elbow_flex.pos': 35.34065934065934,
		'wrist_flex.pos': 93.58241758241758,
		'wrist_roll.pos': -1.2747252747252746,
		'gripper.pos': 20.0
	}
	print("=== Moving to HOME position ===")
	go_to_joint_pose(robot, HOME_JOINTS, duration=duration)


def go_vision_pose(robot: SO101Follower, duration: float = 3.0):
	VISION_JOINTS = {'shoulder_pan.pos': -2.10989010989011, 'shoulder_lift.pos': -56.043956043956044, 'elbow_flex.pos': 21.36263736263736, 'wrist_flex.pos': 99.56043956043956, 'wrist_roll.pos': -0.9230769230769231, 'gripper.pos': 60.0}
	print("=== Moving to VISION position ===")
	go_to_joint_pose(robot, VISION_JOINTS, duration=duration)


def set_gripper(robot: SO101Follower, target_value: float, duration: float = 1.0):
	obs = robot.get_observation()

	current_grip = float(obs.get("gripper.pos", 0.0))

	base_action = {}
	for name in JOINT_NAME_MAP.values():
		if name != "gripper.pos":
			base_action[name] = float(obs.get(name, 0.0))

	steps = max(int(duration * 40), 10)
	dt = duration / steps

	for i in range(1, steps + 1):
		alpha = i / steps
		g = (1.0 - alpha) * current_grip + alpha * target_value
		action = dict(base_action)
		action["gripper.pos"] = g
		robot.send_action(action)
		time.sleep(dt)


def open_gripper(robot: SO101Follower, open_value: float = 40.0, duration: float = 1.0):
	print(f"Opening gripper to {open_value}")
	set_gripper(robot, target_value=open_value, duration=duration)


def close_gripper(robot: SO101Follower, close_value: float = 12.0, duration: float = 1.0):
	print(f"Closing gripper to {close_value}")
	set_gripper(robot, target_value=close_value, duration=duration)


def make_pose_with_R(x: float, y: float, z: float, R: np.ndarray) -> np.ndarray:
	T = np.eye(4)
	T[:3, :3] = R
	T[:3, 3] = [x, y, z]
	return T


def go_to_pose_with_ik(robot: SO101Follower, kin: RobotKinematics,
						T_des: np.ndarray, duration: float = 2.0):
	current_q = get_current_joint_vector(robot, kin)

	try:
		wrist_roll_index = kin.joint_names.index('5')
		if FIXED_WRIST_ROLL is not None:
			current_q[wrist_roll_index] = FIXED_WRIST_ROLL
	except ValueError:
		wrist_roll_index = None

	q_des = kin.inverse_kinematics(
		current_joint_pos=current_q,
		desired_ee_pose=T_des,
		position_weight=1.0,
		orientation_weight=0.1 if R_DOWN is not None else 0.0,
	)

	if wrist_roll_index is not None and FIXED_WRIST_ROLL is not None:
		q_des[wrist_roll_index] = FIXED_WRIST_ROLL

	steps = max(int(duration * 40), 10)
	dt = duration / steps

	for i in range(1, steps + 1):
		alpha = i / steps
		q = (1.0 - alpha) * current_q + alpha * q_des

		action = {}
		for name, value in zip(kin.joint_names, q):
			action[JOINT_NAME_MAP[name]] = float(value)

		obs = robot.get_observation()
		action["gripper.pos"] = float(obs.get("gripper.pos", 0.0))

		robot.send_action(action)
		time.sleep(dt)


def go_to_xyz_with_ik(robot: SO101Follower, kin: RobotKinematics,
						x: float, y: float, z: float, duration: float = 2.0):
	current_q = get_current_joint_vector(robot, kin)
	T_current = kin.forward_kinematics(current_q)
	R_current = T_current[:3, :3]

	T_target = make_pose_with_R(x, y, z, R_current)

	go_to_pose_with_ik(robot, kin, T_target, duration=duration)


def go_to_xyz_with_down_orientation(robot: SO101Follower, kin: RobotKinematics,
										x: float, y: float, z: float,
										duration: float = 2.0):
	if R_DOWN is None:
		print("Warning: R_DOWN not defined. Using current orientation.")
		go_to_xyz_with_ik(robot, kin, x, y, z, duration=duration)
		return

	T_target = make_pose_with_R(x, y, z, R_DOWN)

	go_to_pose_with_ik(robot, kin, T_target, duration=duration)


def setup_robot_and_kinematics(urdf_path: str = None, target_frame: str = "gripper"):
	if urdf_path is None:
		urdf_path = URDF_PATH
	print(f"Using URDF: {urdf_path}")
	print(f"Target frame: {target_frame}")

	config = SO101FollowerConfig(
		port="/dev/ttyACM0",
		id="armzilla",
		use_degrees=True,
	)
	robot = SO101Follower(config)
	robot.connect()

	kin = RobotKinematics(
		urdf_path=urdf_path,
		target_frame_name=target_frame,
	)
	return robot, kin


def safe_shutdown(robot: SO101Follower):
	try:
		print("Shutting down robot...")
		go_home(robot, duration=2.0)
		time.sleep(1)
	except:
		pass
	finally:
		robot.disconnect()
		print("Robot disconnected.")

def test_ik_movement(robot: SO101Follower, kin: RobotKinematics):
	print("\n" + "=" * 60)
	print("IK MOVEMENT TEST")
	print("=" * 60)

	T_current = print_current_ee_pose(robot, kin)
	current_pos = T_current[:3, 3]
	print(f"\nCurrent position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")

	print("\nTest 1: Moving UP 2cm")
	target_z = current_pos[2] + 0.02
	go_to_xyz_with_ik(robot, kin, current_pos[0], current_pos[1], target_z, duration=2.0)

	T_new = print_current_ee_pose(robot, kin)
	print(f"Z change: {T_new[2, 3] - current_pos[2]:.3f}m (expected: +0.02m)")

	print("\nTest 2: Moving FORWARD 2cm")
	target_y = current_pos[1] + 0.02
	go_to_xyz_with_ik(robot, kin, current_pos[0], target_y, current_pos[2], duration=2.0)

	print("\nTest 3: Returning to start position")
	go_to_xyz_with_ik(robot, kin, current_pos[0], current_pos[1], current_pos[2], duration=2.0)

	print("\nTest complete!")


def main():
	print("URDF IK Fixed Module Test")
	robot, kin = None, None
	try:
		robot, kin = setup_robot_and_kinematics()

		test_ik_movement(robot, kin)

		print("\nTesting gripper...")
		open_gripper(robot, open_value=40.0)
		time.sleep(1)
		close_gripper(robot, close_value=12.0)
		time.sleep(1)

		go_home(robot, duration=2.0)

	except Exception as e:
		print(f"Error: {e}")
		traceback.print_exc()
	finally:
		if robot is not None:
			safe_shutdown(robot)


if __name__ == "__main__":
	main()