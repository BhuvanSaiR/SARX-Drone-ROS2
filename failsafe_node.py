import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class FailsafeNode(Node):

    def __init__(self):
        super().__init__('failsafe_node')

        # ---- INPUTS ----
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/command_raw',
            self.cmd_callback,
            1
        )

        self.gps_sub = self.create_subscription(
            Float32MultiArray,
            '/gps',
            self.gps_callback,
            1
        )

        self.perception_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/target',
            self.perception_callback,
            1
        )

        self.kill_sub = self.create_subscription(
            Float32MultiArray,
            '/kill_switch',
            self.kill_callback,
            1
        )

        # ---- OUTPUT ----
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/command',
            1
        )

        # ---- STATE ----
        self.last_cmd_time = time.time()
        self.last_gps_time = time.time()
        self.last_detection_time = time.time()

        self.kill = False

        self.timer = self.create_timer(0.1, self.monitor)

        self.get_logger().info("Failsafe node started")

    # ============================
    # CALLBACKS
    # ============================

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        self.latest_cmd = msg

    def gps_callback(self, msg):
        self.last_gps_time = time.time()

    def perception_callback(self, msg):
        detected = msg.data[0]
        if detected > 0.5:
            self.last_detection_time = time.time()

    def kill_callback(self, msg):
        if msg.data[0] == 1.0:
            self.kill = True

    # ============================
    # MAIN MONITOR LOOP
    # ============================

    def monitor(self):

        now = time.time()

        # 🚨 KILL SWITCH (highest priority)
        if self.kill:
            self.publish_zero("KILL")
            return

        # 🚨 COMMAND LOSS
        if now - self.last_cmd_time > 0.5:
            self.publish_zero("COMMAND LOST")
            return

        # 🚨 GPS LOSS
        if now - self.last_gps_time > 1.0:
            self.publish_zero("GPS LOST")
            return

        # 🚨 DETECTION LOSS (optional behavior)
        if now - self.last_detection_time > 5.0:
            # You can choose hover or continue
            self.get_logger().warn("Detection lost → continuing mission")

        # ✅ NORMAL OPERATION
        if hasattr(self, 'latest_cmd'):
            self.cmd_pub.publish(self.latest_cmd)

    # ============================
    # HELPERS
    # ============================

    def publish_zero(self, reason):

        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]

        self.cmd_pub.publish(msg)
        self.get_logger().warn(f"FAILSAFE: {reason}")


def main():
    rclpy.init()
    node = FailsafeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()