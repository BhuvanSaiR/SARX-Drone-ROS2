import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # Bridge: OpenCV ↔ ROS
        self.bridge = CvBridge()

        # Publishers
        self.front_pub = self.create_publisher(Image, '/camera/front', 1)
        self.bottom_pub = self.create_publisher(Image, '/camera/bottom', 1)

        # Cameras (same as your code)
        self.cam_front = Picamera2(camera_num=1)
        self.cam_bottom = Picamera2(camera_num=0)
        self.get_logger().info("Front cam = 1, Bottom cam = 0 (VERIFY physically!)")

        self.cam_front.configure(self.cam_front.create_preview_configuration())
        self.cam_bottom.configure(self.cam_bottom.create_preview_configuration())

        self.cam_front.start()
        self.cam_bottom.start()

        # Timer = ROS2 loop (VERY IMPORTANT)
        self.timer = self.create_timer(0.1, self.capture_callback)  # 20 Hz

        self.get_logger().info("Camera node started")

    def capture_callback(self):

        try:
            frame_front = self.cam_front.capture_array()
            frame_bottom = self.cam_bottom.capture_array()
        except Exception as e:
            self.get_logger().error(f"Camera capture failed: {e}")
            return

        # Convert to ROS message
        msg_front = self.bridge.cv2_to_imgmsg(frame_front, encoding='bgr8')
        msg_bottom = self.bridge.cv2_to_imgmsg(frame_bottom, encoding='bgr8')

        #Time stamps
        msg_front.header.stamp = self.get_clock().now().to_msg()
        msg_bottom.header.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self.front_pub.publish(msg_front)
        self.bottom_pub.publish(msg_bottom)


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()