import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # Subscribe to detection output
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/detection/front',
            self.detection_callback,
            10
        )

        # Publish refined perception
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/perception/target',
            10
        )

        # ---- YOUR THRESHOLDS ----
        self.AREA_THRESHOLD_FRONT = 0.15

        self.get_logger().info("Perception node started")

    def detection_callback(self, msg):

        found, area, cx, cy = msg.data

        target_detected = False
        approach = False

        # ---- YOUR LOGIC MOVED HERE ----
        if found:
            target_detected = True

            if area > self.AREA_THRESHOLD_FRONT:
                approach = True

        # ---- OUTPUT MESSAGE ----
        out = Float32MultiArray()

        # Format:
        # [detected, approach_flag, cx, cy, area]
        out.data = [
            float(target_detected),
            float(approach),
            float(cx),
            float(cy),
            float(area)
        ]

        self.pub.publish(out)