import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

class MJPEGCameraNode(Node):
    def __init__(self):
        super().__init__('mjpeg_camera_node')

        self.declare_parameter('stream_url', 'http://192.168.1.100:8090/video_feed')
        self.declare_parameter('camera_info_url', '/home/akshara/ros2_ws/src/my_camera.yaml')

        stream_url = self.get_parameter('stream_url').get_parameter_value().string_value
        camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value

        if not os.path.exists(camera_info_url):
            self.get_logger().error(f"Calibration file not found: {camera_info_url}")
            exit(1)

        with open(camera_info_url, 'r') as f:
            calib = yaml.safe_load(f)

        self.camera_info = CameraInfo()
        self.camera_info.width = calib['image_width']
        self.camera_info.height = calib['image_height']
        self.camera_info.k = calib['camera_matrix']['data']
        self.camera_info.d = calib['distortion_coefficients']['data']
        self.camera_info.r = calib['rectification_matrix']['data']
        self.camera_info.p = calib['projection_matrix']['data']
        self.camera_info.distortion_model = calib.get('distortion_model', 'plumb_bob')

        self.pub_image = self.create_publisher(Image, '/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.cap = cv2.VideoCapture(stream_url)
        self.bridge = CvBridge()

        self.create_timer(0.2, self.publish_frame)

        self.get_logger().info("✅ MJPEG camera node started, publishing synchronized Image + CameraInfo at 5 Hz")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received from stream")
            return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_optical_frame'

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = 'camera_optical_frame'

        self.pub_image.publish(img_msg)
        self.pub_info.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = MJPEGCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
