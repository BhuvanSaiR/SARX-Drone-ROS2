import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        self.timer = self.create_timer(0.1, self.process)

        # Subscribe to detection output
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/detection/front',
            self.front_callback,
            10
        )

        # Publish refined perception
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/perception/target',
            10
        )

        self.sub_bottom = self.create_subscription(
            Float32MultiArray,
            '/detection/bottom',
            self.bottom_callback,
            10
        )

        self.bottom_data = None
        self.front_data = None

        self.cam_sub = self.create_subscription(
            Float32MultiArray,
            '/active_camera',
            self.camera_callback,
            10
        )
        
        # ---- YOUR THRESHOLDS ----
        self.AREA_THRESHOLD_FRONT = 0.15
        self.AREA_THRESHOLD_BOTTOM = 0.15
        self.get_logger().info("Perception node started")
        self.active_camera = "front"

    
    def front_callback(self, msg):
        self.front_data = msg.data

    def bottom_callback(self, msg):
        self.bottom_data = msg.data

    def process(self):

        # Select correct camera
        if self.active_camera == "front":
            data = self.front_data
        else:
            data = self.bottom_data

        if data is None:
            return

        found, area, cx, cy = data

        target_detected = found > 0.5

        # Use correct threshold
        if self.active_camera == "front":
            approach = area > self.AREA_THRESHOLD_FRONT
        else:
            approach = area > self.AREA_THRESHOLD_BOTTOM

        self.get_logger().info(
            f"[{self.active_camera}] Detected: {target_detected}, Area: {area:.2f}"
        )

        msg = Float32MultiArray()
        msg.data = [
            float(target_detected),
            float(approach),
            float(cx),
            float(cy),
            float(area)
        ]

        self.pub.publish(msg)

    def camera_callback(self, msg):

        if msg.data[0] == 0.0:
            self.active_camera = "front"
        else:
            self.active_camera = "bottom"

        self.get_logger().info(f"[Perception] Using {self.active_camera} camera")