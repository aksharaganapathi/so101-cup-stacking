import cv2
import numpy as np
import time
import json
import os
from urdf_ik_fixed import (
    setup_robot_and_kinematics, go_to_xyz_with_ik,
    go_home, go_vision_pose, open_gripper, close_gripper,
    get_current_joint_vector, safe_shutdown, go_to_joint_pose
)

class VisionSystem:
    def __init__(self, robot=None, kin=None, resolution='1080p'):
        self.robot = robot
        self.kin = kin
        
        if resolution == '1080p':
            self.img_width = 1920
            self.img_height = 1080
        else:
            self.img_width = 1280
            self.img_height = 720
            
        self.center_x = self.img_width // 2
        self.center_y = self.img_height // 2
        self.camera_forward_offset = 0.07
        self.camera_left_offset = 0.01
        
        self.pickup_region_x_min = 0
        self.pickup_region_x_max = self.img_width
        self.pickup_region_y_min = 0
        self.pickup_region_y_max = self.img_height

        self.place_positions = [
            {'x': -0.02, 'y': -0.3, 'z': 0.15},
            {'x': 0.02, 'y': -0.27, 'z': 0.15},
            {'x': -0.007, 'y': -0.28, 'z': 0.21},
            {'x': 0.1, 'y': -0.25, 'z': 0.1331},
        ]
        self.current_place_index = 0
        
        self.pixels_per_meter = None
        self.orientation = None
        self.load_calibration()
        
        self.camera = None
        
    def setup_camera(self):
        """Setup camera with 1080p resolution."""
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Failed to open camera")
            return False
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera resolution: {actual_width}x{actual_height}")
        
        if actual_width != self.img_width or actual_height != self.img_height:
            print(f"Warning: Could not set {self.img_width}x{self.img_height}")
            print(f"      Using {actual_width}x{actual_height} instead")
            self.img_width = actual_width
            self.img_height = actual_height
            self.center_x = self.img_width // 2
            self.center_y = self.img_height // 2
            self.update_regions()
            
        return True
    
    def update_regions(self):
        """Update region definitions based on current image size (now just one pickup zone)."""
        self.pickup_region_x_min = 0
        self.pickup_region_x_max = self.img_width
        self.pickup_region_y_min = 0
        self.pickup_region_y_max = self.img_height
    
    def load_calibration(self):
        """Load pixel calibration and orientation."""
        if os.path.exists('pixel_calibration.json'):
            with open('pixel_calibration.json', 'r') as f:
                data = json.load(f)
            self.pixels_per_meter = data['pixels_per_meter']
            print(f"✓ Pixel calibration: {self.pixels_per_meter:.1f} px/m")
        else:
            print("⚠️ Pixel calibration not found. Using default 1000 px/m")
            self.pixels_per_meter = 1000.0
        
        if os.path.exists('camera_orientation.json'):
            with open('camera_orientation.json', 'r') as f:
                self.orientation = json.load(f)
            print(f"✓ Camera orientation loaded")
        else:
            print("⚠️ Camera orientation not found. Using DEFAULT FOR YOUR ROBOT.")
            self.orientation = {
                'image_x_to_robot': 'y_negative',
                'image_y_to_robot': 'x_positive',
                'note': 'Based on direct mapping tests'
            }
    
    def pixel_to_robot_offset(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates to robot offset from camera center.
        """
        offset_u = pixel_x - self.center_x
        offset_v = pixel_y - self.center_y
        
        delta_u_m = offset_u / self.pixels_per_meter
        delta_v_m = offset_v / self.pixels_per_meter
        
        perspective_factor = 1.0 + (offset_u / self.center_x) * 0.4
        
        robot_delta_x = delta_v_m * perspective_factor    
        robot_delta_y = delta_u_m    
        
        print(f"\nVision offset calculation:")
        print(f"  Pixel offset: u={offset_u:+.1f}, v={offset_v:+.1f}")
        print(f"  Perspective factor: {perspective_factor:.3f}")
        print(f"  Robot offset: X={robot_delta_x:+.3f}m, Y={robot_delta_y:+.3f}m")
        
        return robot_delta_x, robot_delta_y
    
    def check_object_location(self, pixel_x, pixel_y):
        """
        Check if object is in the pickup spot.
        Returns: 'pickup' or 'other'
        """
        if (self.pickup_region_x_min <= pixel_x <= self.pickup_region_x_max and
            self.pickup_region_y_min <= pixel_y <= self.pickup_region_y_max):
            return 'pickup'
        else:
            return 'other'
    
    def get_robot_target_position(self, pixel_x, pixel_y):
        """
        Calculate robot target position from vision pose and pixel detection.
        Assumes robot is at vision pose.
        """
        if self.robot is None or self.kin is None:
            print("ERROR: Robot not connected")
            return None, None
        
        q = get_current_joint_vector(self.robot, self.kin)
        T = self.kin.forward_kinematics(q)
        robot_x, robot_y, robot_z = T[:3, 3]
        
        delta_x, delta_y = self.pixel_to_robot_offset(pixel_x, pixel_y)
        
        target_x = robot_x + delta_x + self.camera_forward_offset
        target_y = robot_y + delta_y + self.camera_left_offset
        
        print(f"\nTarget Calculation:")
        print(f"  Vision pose: ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
        print(f"  Vision offset: ({delta_x:+.3f}, {delta_y:+.3f})")
        print(f"  Target position: ({target_x:.3f}, {target_y:.3f}, {robot_z:.3f})")
        
        return target_x, target_y
    
    def get_next_place_position(self):
        """Get the next available place position (replaces get_next_stack_position)."""
        if self.current_place_index >= len(self.place_positions):
            print("⚠️ All place positions are full!")
            return None
        
        pos = self.place_positions[self.current_place_index]
        print(f"\nNext place position #{self.current_place_index + 1}: "
              f"({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
        
        return pos
    
    def mark_place_position_used(self):
        """Mark current place position as used and move to next (replaces mark_stack_position_used)."""
        if self.current_place_index < len(self.place_positions):
            self.current_place_index += 1
            print(f"✓ Place position #{self.current_place_index} marked as used")
    
    def reset_place_positions(self):
        """Reset place positions for new session (replaces reset_stack_positions)."""
        self.current_place_index = 0
        print("✓ Place positions reset")
    
    def detect_red_objects(self, frame, show_detection=True):
        """
        Detect all red objects in the image.
        Returns list of (center_x, center_y, contour, area, location_type) for each object.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return []
        
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            MIN_AREA = 500
            if area < MIN_AREA:
                continue
            
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            location = self.check_object_location(cX, cY)
            
            detected_objects.append((cX, cY, contour, area, location))
        
        detected_objects.sort(key=lambda x: x[3], reverse=True)
        
        if show_detection and detected_objects:
            display = frame.copy()
            
            pickup_rect_color = (0, 255, 0)
            cv2.rectangle(display,  
                          (self.pickup_region_x_min, self.pickup_region_y_min),
                          (self.pickup_region_x_max - 1, self.pickup_region_y_max - 1),
                          pickup_rect_color, 3)
            cv2.putText(display, "PICKUP ZONE (100%)",  
                        (self.pickup_region_x_min + 10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, pickup_rect_color, 2)
            
            for i, (cX, cY, contour, area, location) in enumerate(detected_objects):
                cv2.drawContours(display, [contour], -1, (0, 255, 255), 2)
                
                cv2.circle(display, (cX, cY), 10, (0, 0, 255), -1)
                cv2.putText(display, str(i+1), (cX-5, cY+5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                location_text = f"#{i+1}: PICKUP"
                location_color = (0, 255, 0)
                cv2.putText(display, location_text,  
                            (cX + 15, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, location_color, 2)
            
            pickup_count = sum(1 for obj in detected_objects if obj[4] == 'pickup')
            cv2.putText(display, f"Pickup Zone: {pickup_count} objects",  
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Object Detection', display)
            cv2.waitKey(1000)
        
        return detected_objects
    
    def capture_stable_image(self, stabilization_time=2.0):
        """
        Capture image with camera stabilization.
        """
        if self.camera is None:
            if not self.setup_camera():
                return None
        
        print(f"\nStabilizing camera for {stabilization_time:.1f} seconds...")
        
        for i in range(5):
            self.camera.read()
        
        start_time = time.time()
        while time.time() - start_time < stabilization_time:
            ret, frame = self.camera.read()
            if not ret:
                return None
            time.sleep(0.1)
        
        ret, frame = self.camera.read()
        if not ret:
            return None
        
        print("✓ Camera stabilized")
        return frame
    
    def find_best_pickup_object(self, frame):
        """
        Find the best object to pick (now, any detected object).
        Returns (center_x, center_y, contour, area) or None.
        """
        detected_objects = self.detect_red_objects(frame, show_detection=True)
        
        pickup_objects = [obj for obj in detected_objects if obj[4] == 'pickup']
        
        if not pickup_objects:
            print("✗ No objects found in pickup zone")
            return None
        
        best_object = pickup_objects[0]
        cX, cY, contour, area, location = best_object
        
        print(f"\n✓ Selected object #{1} for picking:")
        print(f"  Position: ({cX}, {cY})")
        print(f"  Area: {area:.0f} px")
        print(f"  Location: {location.upper()}")
        
        return (cX, cY, contour, area)

def automated_pick_and_place():
    """
    Automated system that continuously picks items from the entire feed 
    and places them in a fixed sequence of place positions.
    (This is the new function, renamed from automated_pick_and_stack)
    """
    robot, kin = None, None
    
    try:
        print("\n" + "="*60)
        print("AUTOMATED PICK AND PLACE SYSTEM")
        print("="*60)
        
        print("\nConnecting to robot...")
        robot, kin = setup_robot_and_kinematics()
        
        vision = VisionSystem(robot, kin, resolution='1080p')
        
        if not vision.setup_camera():
            print("Failed to setup camera")
            return
        
        TABLE_Z = 0.1331
        APPROACH_HEIGHT = TABLE_Z + 0.2
        GRASP_HEIGHT = TABLE_Z + 0.02
        
        PLACE_APPROACH_HEIGHT = TABLE_Z + 0.25
        PLACE_PLACE_HEIGHT = TABLE_Z + 0.05
        
        print("\n" + "="*60)
        print("SYSTEM READY")
        print("="*60)
        print("\nThe system will automatically:")
        print("1. Check for objects in the entire PICKUP ZONE (100% of feed)")
        print("2. Pick the largest object found")
        print("3. Place it in a fixed sequence of PLACE positions")
        print("4. Repeat until no objects are found or place positions run out")
        print("\nPress Ctrl+C at any time to stop.")
        
        input("\nPress Enter to start automated picking and placing...")
        
        item_count = 0
        
        while True:
            print(f"\n" + "="*60)
            print(f"ATTEMPT #{item_count + 1}")
            print("="*60)
            
            print("\n1. Moving to vision pose...")
            go_vision_pose(robot, duration=1.0)
            time.sleep(1)
            
            print("\n2. Capturing image...")
            frame = vision.capture_stable_image(stabilization_time=2.0)
            if frame is None:
                print("✗ Failed to capture image")
                break
            
            print("\n3. Looking for objects in pickup zone...")
            object_data = vision.find_best_pickup_object(frame)
            
            if object_data is None:
                print("\n" + "="*60)
                print("NO MORE OBJECTS FOUND IN PICKUP ZONE")
                print("="*60)
                print(f"\nTotal items placed: {item_count}")
                break
            
            cX, cY, contour, area = object_data
            
            print("\n4. Calculating pick position...")
            pick_x, pick_y = vision.get_robot_target_position(cX, cY)
            
            place_pos = vision.get_next_place_position()
            if place_pos is None:
                print("\n✗ No more place positions available")
                break
            
            print(f"\n" + "="*60)
            print(f"PLAN FOR ITEM #{item_count + 1}")
            print("="*60)
            print(f"Pick from:  ({pick_x:.3f}, {pick_y:.3f})")
            print(f"Place at:   ({place_pos['x']:.3f}, {place_pos['y']:.3f})")
            
            print(f"\n5. Executing pick and place #{item_count + 1}...")
            
            try:
                print("    a. Moving to home...")
                go_home(robot, duration=1.0)
                
                print(f"    b. Approaching pick position...")
                go_to_xyz_with_ik(robot, kin, pick_x, pick_y, APPROACH_HEIGHT, duration=1.0)
                
                print("    c. Opening gripper...")
                open_gripper(robot, 40.0)
                time.sleep(0.5)
                
                print("    d. Moving to grasp height...")
                go_to_xyz_with_ik(robot, kin, pick_x, pick_y, GRASP_HEIGHT, duration=1.0)
                
                print("    e. Closing gripper...")
                close_gripper(robot, 25.0)
                time.sleep(0.5)
                
                print("    f. Lifting item...")
                go_to_xyz_with_ik(robot, kin, pick_x, pick_y, APPROACH_HEIGHT, duration=1.0)
                
                print(" Going home...")
                go_home(robot, duration=1.0)
                
                print(f"    g. Moving to place position...")
                go_to_xyz_with_ik(robot, kin, place_pos['x'], place_pos['y'], 
                                 PLACE_APPROACH_HEIGHT, duration=2.0)
                
                print("    h. Moving to place height...")
                go_to_xyz_with_ik(robot, kin, place_pos['x'], place_pos['y'], 
                                 place_pos['z'], duration=1.5)
                
                print("    i. Opening gripper to release...")
                open_gripper(robot, 29.0)
                time.sleep(0.5)
                
                print("    j. Lifting from place spot...")
                go_to_xyz_with_ik(robot, kin, place_pos['x'], place_pos['y'], 
                                 PLACE_APPROACH_HEIGHT, duration=1.0)
                
                vision.mark_place_position_used()
                
                item_count += 1
                
                print(f"\n✓ Successfully placed item #{item_count}")
                
                print("\nPausing for 2 seconds before next item...")
                time.sleep(2)
                
            except Exception as e:
                print(f"\n✗ Error during pick and place: {e}")
                import traceback
                traceback.print_exc()
                
                try:
                    go_home(robot, duration=2.0)
                except:
                    pass
                
                response = input("\nContinue with next item? (y/n): ").strip().lower()
                if response != 'y':
                    break
        
        print(f"\n" + "="*60)
        print("AUTOMATED PICK AND PLACE COMPLETE")
        print("="*60)
        print(f"Total items placed: {item_count}")
        
        print("\nReturning to home position...")
        go_home(robot, duration=2.0)
    
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("PROGRAM INTERRUPTED BY USER")
        print("="*60)
        print(f"\nTotal items placed before interruption: {item_count}")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nCleaning up...")
        cv2.destroyAllWindows()
        if robot:
            try:
                go_home(robot, duration=2.0)
            except:
                pass
            safe_shutdown(robot)
        print("Done.")

def measure_pixels_per_meter_with_validation(robot=None, kin=None):
    """Interactive tool to measure pixels per meter WITH VALIDATION."""
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

def manual_pick_and_place_with_validation():
    """Manual pick and place with image validation."""
    
    robot, kin = None, None
    
    try:
        print("\n" + "="*60)
        print("MANUAL PICK AND PLACE WITH VALIDATION")
        print("="*60)
        
        print("\nConnecting to robot...")
        robot, kin = setup_robot_and_kinematics()
        
        vision = VisionSystem(robot, kin, resolution='1080p')
        
        if not vision.setup_camera():
            print("Failed to setup camera")
            return
        
        TABLE_Z = 0.1331
        APPROACH_HEIGHT = TABLE_Z + 0.2
        GRASP_HEIGHT = TABLE_Z + 0.02
        
        while True:
            print("\n" + "="*60)
            print("MANUAL CONTROL MENU")
            print("="*60)
            print("1. Single pick and place")
            print("2. Show camera feed")
            print("3. Calibrate pixels per meter")
            print("4. Go to vision pose")
            print("5. Go home")
            print("6. Run automated pick and place") 
            print("7. Exit")
            
            choice = input("\nChoice (1-7): ").strip()
            
            if choice == '1':
                execute_single_pick(robot, kin, vision, TABLE_Z, APPROACH_HEIGHT, GRASP_HEIGHT)
            
            elif choice == '2':
                show_camera_feed(vision)
            
            elif choice == '3':
                measure_pixels_per_meter_with_validation(robot, kin)
                vision.load_calibration()
            
            elif choice == '4':
                print("\nMoving to vision pose...")
                go_vision_pose(robot, duration=3.0)
            
            elif choice == '5':
                print("\nMoving home...")
                go_home(robot, duration=2.0)
            
            elif choice == '6':
                automated_pick_and_place() 
                vision.reset_place_positions()
            
            elif choice == '7':
                print("\nExiting...")
                break
            
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nCleaning up...")
        cv2.destroyAllWindows()
        if robot:
            try:
                go_home(robot, duration=2.0)
            except:
                pass
            safe_shutdown(robot)
        print("Done.")

def show_camera_feed(vision):
    """Show camera feed with zones (now only a single pickup zone)."""
    if vision.camera is None:
        vision.setup_camera()
    
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

def execute_single_pick(robot, kin, vision, TABLE_Z, APPROACH_HEIGHT, GRASP_HEIGHT):
    """Execute single manual pick and place."""
    print("Moving to vision pose...")
    go_vision_pose(robot, duration=1.0)
    time.sleep(1)
    
    print("\nCapturing image with stabilization...")
    frame = vision.capture_stable_image(stabilization_time=2.0)
    if frame is None:
        print("Failed to capture image")
        return
    
    print("\nDetecting objects...")
    object_data = vision.find_best_pickup_object(frame)
    
    if object_data is None:
        print("No suitable object found")
        return
    
    cX, cY, contour, area = object_data
    
    target_x, target_y = vision.get_robot_target_position(cX, cY)
    
    print(f"\nCalculated pick position: ({target_x:.3f}, {target_y:.3f})")
    print("\nEnter place position:")
    place_x = float(input(f"  Place X [0.1]: ") or 0.1)
    place_y = float(input(f"  Place Y [-0.05]: ") or -0.05)
    
    try:
        print("\n1. Moving to home...")
        go_home(robot, duration=2.0)
        
        print(f"2. Approaching object...")
        go_to_xyz_with_ik(robot, kin, target_x, target_y, APPROACH_HEIGHT, duration=1.0)
        
        print("3. Opening gripper...")
        open_gripper(robot, 60.0)
        time.sleep(0.5)
        
        print("4. Moving to grasp height...")
        go_to_xyz_with_ik(robot, kin, target_x, target_y, GRASP_HEIGHT, duration=1.0)
        
        print("5. Closing gripper...")
        close_gripper(robot, 20.0)
        time.sleep(0.5)
        
        print("6. Lifting...")
        go_to_xyz_with_ik(robot, kin, target_x, target_y, APPROACH_HEIGHT, duration=1.0)
        
        print(f"7. Moving to place position...")
        go_to_xyz_with_ik(robot, kin, place_x, place_y, APPROACH_HEIGHT, duration=1.0)
        
        print("8. Moving to place height...")
        go_to_xyz_with_ik(robot, kin, place_x, place_y, GRASP_HEIGHT, duration=1.0)
        
        print("9. Opening gripper...")
        open_gripper(robot, 40.0)
        time.sleep(0.5)
        
        print("10. Lifting...")
        go_to_xyz_with_ik(robot, kin, place_x, place_y, APPROACH_HEIGHT, duration=1.0)
        
        print("11. Returning to home...")
        go_home(robot, duration=1.0)
        
        print("\n✓ Pick and place complete!")
        
    except Exception as e:
        print(f"\n✗ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        go_home(robot, duration=2.0)

def main():
    print("\n" + "="*60)
    print("AUTOMATED VISION PICK AND PLACE SYSTEM") 
    print("="*60)
    print("Features:")
    print("• Automated picking from PICKUP ZONE (entire vision area)") 
    print("• Automated placing in a fixed sequence of points") 
    print("• Camera stabilization before capture")
    print("• Returns to vision pose for accurate reference")
    print("• Stops when no more objects in pickup zone or place positions run out")
    
    print("\n1. Run automated pick and place system") 
    print("2. Manual control (for testing)")
    print("3. Calibrate pixels per meter")
    print("4. Exit")
    
    choice = input("\nChoice (1-4): ").strip()
    
    if choice == '1':
        automated_pick_and_place() 
    elif choice == '2':
        manual_pick_and_place_with_validation()
    elif choice == '3':
        print("\nSetting up robot for calibration...")
        robot, kin = setup_robot_and_kinematics()
        try:
            measure_pixels_per_meter_with_validation(robot, kin)
        finally:
            safe_shutdown(robot)
    elif choice == '4':
        print("Exiting...")
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()