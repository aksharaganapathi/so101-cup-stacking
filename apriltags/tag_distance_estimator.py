#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
import math

class TagDistanceEstimator(Node):
    def __init__(self):
        super().__init__('tag_distance_estimator')
        self.tag_size_m = 0.1  # match your AprilTag size parameter
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribe to camera info to get intrinsics
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Subscribe to tag detections
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)

    def camera_info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]  # fx in pixels
            self.fy = msg.k[4]  # fy in pixels
            self.cx = msg.k[2]  # principal point x
            self.cy = msg.k[5]  # principal point y
            self.get_logger().info(f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def detections_callback(self, msg: AprilTagDetectionArray):
        if self.fx is None:
            return  # wait until camera info arrives

        for det in msg.detections:
            corners = det.corners
            if len(corners) != 4:
                continue

            # Compute average width in pixels (top and bottom edges)
            width_px_top = math.hypot(corners[1].x - corners[0].x, corners[1].y - corners[0].y)
            width_px_bottom = math.hypot(corners[2].x - corners[3].x, corners[2].y - corners[3].y)
            width_px = (width_px_top + width_px_bottom) / 2.0

            # Compute average height in pixels (left and right edges)
            height_px_left = math.hypot(corners[3].x - corners[0].x, corners[3].y - corners[0].y)
            height_px_right = math.hypot(corners[2].x - corners[1].x, corners[2].y - corners[1].y)
            height_px = (height_px_left + height_px_right) / 2.0

            # Use average of width/height
            size_px = (width_px + height_px) / 2.0

            # Compute Z in meters and inches
            Z_m = self.fx * self.tag_size_m / size_px
            Z_in = Z_m * 39.3701

            # Compute X/Y relative to camera
            center_u = sum(c.x for c in corners) / 4.0
            center_v = sum(c.y for c in corners) / 4.0
            X_m = (center_u - self.cx) * Z_m / self.fx
            Y_m = (center_v - self.cy) * Z_m / self.fy

            X_in = X_m * 39.3701
            Y_in = Y_m * 39.3701

            self.get_logger().info(
                f"Tag ID {det.id} | Z={Z_m:.3f} m ({Z_in:.1f} in) | X={X_m:.3f} m ({X_in:.1f} in) | Y={Y_m:.3f} m ({Y_in:.1f} in)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TagDistanceEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
