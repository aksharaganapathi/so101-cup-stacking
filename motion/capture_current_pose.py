import json
from pathlib import Path
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID   = "armzilla"
OUT_FILE   = Path("waypoints.json")

def format_pose(obs, nd=2):
    return {k: round(v, nd) for k, v in obs.items() if k.endswith(".pos")}

def main():
    robot = SO101Follower(SO101FollowerConfig(port=ROBOT_PORT, id=ROBOT_ID))
    print(f"Connecting to '{ROBOT_ID}' on '{ROBOT_PORT}'...")
    robot.connect()
    print("Connected.")

    print("Disabling torque so you can move the arm by hand (support it!)")
    robot.bus.disable_torque()

    waypoints = {}
    if OUT_FILE.exists():
        try:
            waypoints = json.loads(OUT_FILE.read_text())
        except Exception:
            pass

    print("""
Controls:
  [Enter]  capture current joint positions
  [t]      toggle torque (off/on)
  [q]      quit
""")
    torque_on = False
    try:
        while True:
            cmd = input("(Enter=cap, t=torque toggle, q=quit): ").strip().lower()
            if cmd == "q":
                break
            elif cmd == "t":
                if torque_on:
                    robot.bus.disable_torque()
                    print("Torque DISABLED \u2014 you can hand-pose now.")
                    torque_on = False
                else:
                    robot.bus.enable_torque()
                    print("Torque ENABLED \u2014 joints will hold position.")
                    torque_on = True
            else:
                obs = robot.get_observation()
                pose = format_pose(obs)
                print("Current pose:", pose)
                name = input("Name this waypoint (e.g., APPROACH, TOUCH, REPOS): ").strip() or "wp"
                waypoints[name] = pose
                OUT_FILE.write_text(json.dumps(waypoints, indent=2))
                print(f"Saved '{name}' \u2192 {OUT_FILE}\n")
    finally:
        robot.bus.disable_torque()
        robot.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()