import cv2
import numpy as np
import time
import json
from utils.urdf_ik import setup_robot_and_kinematics, go_vision_pose, go_home, safe_shutdown
from vision_system import VisionSystem

def measure_pixels_per_meter_with_validation(robot=None, kin=None):
    print("\n" + "="*60)
    print("PIXELS PER METER CALIBRATION WITH VALIDATION")
    print("="*60)
    
    vision = VisionSystem(robot, kin, resolution='1080p')
    
    if not vision.setup_camera():
        print("Failed to setup camera")
        return None
    
    if robot:
        print("\nMoving to vision pose...")
        go_vision_pose(robot, duration=3.0)
        time.sleep(1)
    
    print("\nPlace a ruler or object with known dimensions in view.")
    print("We'll capture an image and let you mark two points.")
    
    captured_image = vision.capture_stable_image(stabilization_time=2.0)
    if captured_image is None:
        print("No image captured")
        return None
    
    print("\n" + "="*60)
    print("MARK MEASUREMENT POINTS")
    print("="*60)
    print("\nClick on TWO points with a known distance between them.")
    print("First click: Start point")
    print("Second click: End point")
    print("Then press ENTER")
    
    points = []
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 2:
                points.append((x, y))
                print(f"Point {len(points)}: ({x}, {y})")
                
                img_copy = param[0].copy()
                cv2.circle(img_copy, (x, y), 10, (0, 255, 0), -1)
                
                if len(points) == 1:
                    cv2.putText(img_copy, "START", (x, y-15), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(img_copy, "END", (x, y-15), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.line(img_copy, points[0], points[1], (0, 255, 0), 3)
                
                cv2.imshow('Mark Two Points', img_copy)
    
    image_copy = captured_image.copy()
    cv2.imshow('Mark Two Points', image_copy)
    cv2.setMouseCallback('Mark Two Points', mouse_callback, [image_copy])
    
    print("\nClick two points, then press ENTER...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    if len(points) != 2:
        print("Need exactly 2 points")
        return None
    
    x1, y1 = points[0]
    x2, y2 = points[1]
    pixel_distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    print(f"\nPixel distance: {pixel_distance:.1f} pixels")
    print("\nEnter the REAL distance between these points:")
    print("Example: For 10cm, enter '10'")
    
    real_distance_cm = float(input("Real distance in centimeters: "))
    real_distance_m = real_distance_cm / 100.0
    
    pixels_per_meter = pixel_distance / real_distance_m
    
    print(f"\n" + "="*60)
    print("CALIBRATION RESULT")
    print("="*60)
    print(f"Real distance: {real_distance_m:.3f} m ({real_distance_cm} cm)")
    print(f"Pixel distance: {pixel_distance:.1f} pixels")
    print(f"PIXELS_PER_METER = {pixels_per_meter:.1f}")
    
    result_img = captured_image.copy()
    cv2.line(result_img, points[0], points[1], (0, 255, 0), 3)
    cv2.circle(result_img, points[0], 10, (0, 255, 0), -1)
    cv2.circle(result_img, points[1], 10, (0, 255, 0), -1)
    
    cv2.putText(result_img, f"Distance: {pixel_distance:.1f} px", (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(result_img, f"Real: {real_distance_cm} cm", (50, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(result_img, f"{pixels_per_meter:.1f} px/m", (50, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('Calibration Result', result_img)
    cv2.waitKey(3000)
    cv2.destroyAllWindows()
    
    calibration = {
        'pixels_per_meter': float(pixels_per_meter),
        'real_distance_m': float(real_distance_m),
        'pixel_distance': float(pixel_distance),
        'image_resolution': [vision.img_width, vision.img_height],
        'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
        'note': 'Calibrated with validation'
    }
    
    with open('pixel_calibration.json', 'w') as f:
        json.dump(calibration, f, indent=2)
    
    print(f"\nCalibration saved to pixel_calibration.json")
    
    if robot:
        go_home(robot, duration=2.0)
    
    return pixels_per_meter

def main():
    print("\n" + "="*60)
    print("CAMERA CALIBRATION TOOL")
    print("="*60)
    
    print("\nSetting up robot for calibration...")
    robot, kin = setup_robot_and_kinematics()
    try:
        measure_pixels_per_meter_with_validation(robot, kin)
    finally:
        safe_shutdown(robot)

if __name__ == "__main__":
    main()
