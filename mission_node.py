import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ---- STATE ----
        self.state = "SEARCHING"
        self.last_detection_time = time.time()

        # ---- SUBSCRIBE (Perception) ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/perception/target',
            self.perception_callback,
            10
        )

        # ---- PUBLISH (Commands) ----
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/command',
            10
        )

        self.drop_pub = self.create_publisher(
            Float32MultiArray,
            '/drop',
            10
        )

        self.get_logger().info("Mission node started")

    # ============================
    # MAIN STATE MACHINE
    # ============================

    def perception_callback(self, msg):

        detected, approach, cx, cy, area = msg.data

        # -------- SEARCHING --------
        if self.state == "SEARCHING":

            if detected:
                self.get_logger().info("Target detected → APPROACHING")
                self.save_checkpoint()
                self.state = "APPROACHING"

        # -------- APPROACHING --------
        elif self.state == "APPROACHING":

            if not detected:
                self.get_logger().info("Lost target → CENTERING")
                self.state = "CENTERING"
                return

            # Move forward + yaw alignment
            self.publish_movement(1.0, cx, 0.0)

        # -------- CENTERING --------
        elif self.state == "CENTERING":

            # Use cx, cy to center (later bottom cam)
            if abs(cx) < 0.05 and abs(cy) < 0.05:
                self.get_logger().info("Centered → DESCENDING")
                self.state = "DESCENDING"
                return

            self.publish_movement(0.0, cx, cy)

        # -------- DESCENDING --------
        elif self.state == "DESCENDING":

            self.get_logger().info("Descending...")
            self.publish_movement(0.0, 0.0, -0.5)

            time.sleep(2)  # simple placeholder

            self.state = "DROPPING"

        # -------- DROPPING --------
        elif self.state == "DROPPING":

            self.get_logger().info("Dropping payload")

            drop_msg = Float32MultiArray()
            drop_msg.data = [1.0]

            self.drop_pub.publish(drop_msg)

            time.sleep(1)

            self.state = "RETURNING"

        # -------- RETURNING --------
        elif self.state == "RETURNING":

            self.get_logger().info("Returning to checkpoint")

            self.publish_movement(-1.0, 0.0, 0.0)

            time.sleep(2)

            self.state = "SEARCHING"

    # ============================
    # HELPERS
    # ============================

    def publish_movement(self, forward, cx, cy):

        cmd = Float32MultiArray()

        # format: [forward, yaw_correction, vertical]
        cmd.data = [
            float(forward),
            float(cx),
            float(cy)
        ]

        self.cmd_pub.publish(cmd)

    def save_checkpoint(self):
        self.get_logger().info("Checkpoint saved (placeholder)")


def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()