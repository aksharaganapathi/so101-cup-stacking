import os
import numpy as np

from utils.kinematics import RobotKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

URDF_PATH = os.path.expanduser("~/ros2_ws/src/dora-bambot/URDF/so101.urdf")

TARGET_FRAME = "gripperframe"

OBS_TO_KIN = {
	"shoulder_pan.pos": "1",
	"shoulder_lift.pos": "2",
	"elbow_flex.pos": "3",
	"wrist_flex.pos": "4",
	"wrist_roll.pos": "5",
	"gripper.pos": "6",
}


def make_robot(port: str, robot_id: str) -> SO101Follower:
	cfg = SO101FollowerConfig(port=port, id=robot_id, use_degrees=True)
	robot = SO101Follower(cfg)
	robot.connect(calibrate=False)
	return robot


def get_current_joint_vector(robot: SO101Follower, kin: RobotKinematics) -> np.ndarray:
	obs = robot.get_observation()
	q = np.zeros(len(kin.joint_names), dtype=float)

	for i, kin_name in enumerate(kin.joint_names):
		obs_keys = [k for k, v in OBS_TO_KIN.items() if v == kin_name]
		if not obs_keys:
			raise KeyError(f"No observation key found for joint '{kin_name}'")
		q[i] = obs[obs_keys[0]]

	return q


def print_current_ee_pose(robot: SO101Follower, kin) -> np.ndarray:
	q = get_current_joint_vector(robot, kin)
	T = kin.forward_kinematics(q)
	pos = T[:3, 3]
	print(f"Current EE position (URDF frame): x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
	return T


def print_calibration_constants(R_down: np.ndarray, fixed_wrist_roll: float) -> None:
	print("\n================ CALIBRATION RESULT ================")
	print("Paste these into your urdf_ik.py (near the top):\n")

	print("R_DOWN = np.array([")
	for row in R_down:
		row_str = ", ".join(f"{v:.8f}" for v in row)
		print(f"    [{row_str}],")
	print("])\n")

	print(f"FIXED_WRIST_ROLL = {fixed_wrist_roll:.6f}")
	print("===================================================\n")


def main():
	kin = RobotKinematics(
		urdf_path=URDF_PATH,
		target_frame_name=TARGET_FRAME,
	)

	WRIST_ROLL_KIN = OBS_TO_KIN["wrist_roll.pos"]
	WRIST_ROLL_INDEX = kin.joint_names.index(WRIST_ROLL_KIN)

	port = "/dev/ttyACM0"
	robot_id = "armzilla"
	robot = make_robot(port, robot_id)

	print(f"Connected to SO-101 follower on {port} (id={robot_id}).")
	print("\nUse this script like this:")
	print("  1) Press 't' to toggle torque OFF and manually pose the arm.")
	print("  2) Press 't' again to turn torque ON and lock that pose.")
	print("  3) Press 'c' to capture current pose and print R_DOWN + FIXED_WRIST_ROLL.")
	print("  4) Press 'q' to quit.\n")

	torque_on = True
	print("Torque is currently ON (robot holding its pose).")

	try:
		while True:
			cmd = input("[t=toggle torque, c=capture, q=quit] > ").strip().lower()

			if cmd == "t":
				if torque_on:
					print("Disabling torque on all motors (arm will go limp) ...")
					robot.bus.disable_torque()
					torque_on = False
					print("Torque OFF – you can manually move the arm now.")
				else:
					print("Enabling torque on all motors (arm will hold pose) ...")
					robot.bus.enable_torque()
					torque_on = True
					print("Torque ON – pose is now locked.")

			elif cmd == "c":
				if not torque_on:
					print("Torque is OFF; enabling torque first so the arm holds this pose.")
					robot.bus.enable_torque()
					torque_on = True

				q = get_current_joint_vector(robot, kin)
				T = kin.forward_kinematics(q)
				R = T[:3, :3]
				fixed_wrist_roll = float(q[WRIST_ROLL_INDEX])
				print_calibration_constants(R, fixed_wrist_roll)
				print("=== Current EE pose ===")
				print_current_ee_pose(robot, kin)
				print("Observation:", robot.get_observation())

			elif cmd == "q":
				print("Quitting without further changes.")
				break

			else:
				print("Unknown command. Use 't', 'c', or 'q'.")

	finally:
		print("Disconnecting robot...")
		robot.disconnect()
		print("Done.")


if __name__ == "__main__":
	main()