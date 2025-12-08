import json
import time
from pathlib import Path
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID = "armzilla"
WPS_FILE = Path("waypoints.json")

ERROR_DEG = 2.5
CMD_HZ = 50.0
MAX_SPEED = 20.0


def read_waypoints():
    if not WPS_FILE.exists():
        raise FileNotFoundError(f"Waypoints file not found: {WPS_FILE}")
    return json.loads(WPS_FILE.read_text())


def check_error(cur_pos, target_pos, tol=ERROR_DEG):
    for joint, target_val in target_pos.items():
        if joint not in cur_pos:
            continue
        if abs(target_val - cur_pos[joint]) > tol:
            return True
    return False


def get_joint_pose(obs: dict):
    return {k: v for k, v in obs.items() if k.endswith(".pos")}


def move_smooth(robot, target_pos, label="pose",
                max_speed_deg_per_s=MAX_SPEED, hz=CMD_HZ):
    obs = robot.get_observation()
    cur_pos = get_joint_pose(obs)
    cur_subset = {k: cur_pos[k] for k in target_pos.keys()}

    max_delta = max(abs(target_pos[j] - cur_subset[j]) for j in target_pos)
    if max_delta < 1e-3:
        print(f"{label}: already at target (delta < 0.001°)")
        return

    duration = max_delta / max_speed_deg_per_s
    period = 1.0 / hz
    steps = max(1, int(duration * hz))

    print(f"{label}: moving smoothly over ~{duration:.2f}s ({steps} steps)")

    start_time = time.perf_counter()
    for i in range(1, steps + 1):
        alpha = i / steps
        cmd = {
            j: (1 - alpha) * cur_subset[j] + alpha * target_pos[j]
            for j in target_pos
        }
        robot.send_action(cmd)

        next_t = start_time + i * period
        sleep_t = next_t - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)

    robot.send_action(target_pos)

    obs_final = robot.get_observation()
    cur_final = get_joint_pose(obs_final)
    if check_error(cur_final, target_pos):
        print(f"Warning: {label} not exactly within {ERROR_DEG}° on all joints.")
    else:
        print(f"{label}: target reached within tolerance.")


def main():
    wps = read_waypoints()
    REQUIRED = ["APPROACH", "TOUCH"]
    for r in REQUIRED:
        if r not in wps:
            raise KeyError(f"Missing waypoint '{r}' in {WPS_FILE}. "
                           f"Use capture script to add it with name '{r}'.")

    APPROACH = wps["APPROACH"]
    TOUCH = wps["TOUCH"]

    robot = SO101Follower(SO101FollowerConfig(port=ROBOT_PORT, id=ROBOT_ID))
    print(f"Connecting to '{ROBOT_ID}' on '{ROBOT_PORT}'...")
    robot.connect()
    print("Connected.\n")

    try:
        move_smooth(robot, APPROACH, "APPROACH")
        move_smooth(robot, TOUCH, "TOUCH")
        move_smooth(robot, APPROACH, "RETRACT to APPROACH")
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()