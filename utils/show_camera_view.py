import cv2
import time
from utils.urdf_ik import setup_robot_and_kinematics, go_vision_pose, safe_shutdown
from vision_system import VisionSystem

def show_camera_feed(robot, kin):
    print("\nMoving to Vision Pose...")
    go_vision_pose(robot, duration=2.0)
    time.sleep(1)

    vision = VisionSystem(robot, kin, resolution='1080p')
    if not vision.setup_camera():
        print("Failed to setup camera")
        return

    print("\nShowing camera feed. Press 'q' to quit.")
    while True:
        ret, frame = vision.camera.read()
        if not ret:
            break
        
        display = frame.copy()
        
        pickup_rect_color = (0, 255, 0)
        cv2.rectangle(display,  
                      (vision.pickup_region_x_min, vision.pickup_region_y_min),
                      (vision.pickup_region_x_max - 1, vision.pickup_region_y_max - 1),
                      pickup_rect_color, 3)
        cv2.putText(display, "PICKUP ZONE (100%)",  
                    (vision.pickup_region_x_min + 10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, pickup_rect_color, 2)
        
        cv2.putText(display, "Camera Feed - Press 'q' to quit",  
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display, "PICKUP ZONE: All objects will be picked",  
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('Camera Feed', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    print("Exiting camera feed...")

def main():
    print("\n" + "="*60)
    print("CAMERA VIEW TOOL")
    print("="*60)
    
    print("\nConnecting to robot for camera feed...")
    robot, kin = setup_robot_and_kinematics()
    try:
        show_camera_feed(robot, kin)
    finally:
        safe_shutdown(robot)

if __name__ == "__main__":
    main()
