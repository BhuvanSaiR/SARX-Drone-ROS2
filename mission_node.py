import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ============================
        # STATE MACHINE
        # ============================
        self.state = "SEARCHING"

        # ============================
        # WAYPOINT SYSTEM
        # ============================
        self.waypoints = []
        self.current_wp = 0
        self.wp_tolerance = 1.5  # meters

        # ============================
        # CHECKPOINT SYSTEM
        # ============================
        self.checkpoint = None
        self.returning_from_drop = False

        # ============================
        # SUBSCRIBERS
        # ============================
        self.perception_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/target',
            self.perception_callback,
            10
        )

        self.wp_sub = self.create_subscription(
            Float32MultiArray,
            '/mission_waypoints',
            self.waypoint_callback,
            10
        )

        # ============================
        # PUBLISHERS
        # ============================
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

        self.gps_pub = self.create_publisher(
            Float32MultiArray,
            '/goto_gps',
            10
        )

        # ============================
        # TIMERS
        # ============================
        self.timer = self.create_timer(0.1, self.control_loop)

        # ============================
        # CURRENT STATE DATA
        # ============================
        self.detected = False
        self.approach = False
        self.cx = 0.0
        self.cy = 0.0
        self.area = 0.0

        self.get_logger().info("Mission node fully initialized")

    # ======================================
    # CALLBACKS
    # ======================================

    def perception_callback(self, msg):
        self.detected, self.approach, self.cx, self.cy, self.area = msg.data

    def waypoint_callback(self, msg):

        data = msg.data
        self.waypoints = []

        for i in range(0, len(data), 2):
            lat = data[i]
            lon = data[i + 1]
            self.waypoints.append((lat, lon))

        self.current_wp = 0

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

    # ======================================
    # MAIN CONTROL LOOP
    # ======================================

    def control_loop(self):

        # -------- SEARCHING (Waypoint Following) --------
        if self.state == "SEARCHING":

            if self.detected:
                self.get_logger().info("Human detected → APPROACHING")

                self.save_checkpoint()
                self.state = "APPROACHING"
                return

            self.follow_waypoints()

        # -------- APPROACHING --------
        elif self.state == "APPROACHING":

            if not self.detected:
                self.get_logger().info("Lost target → CENTERING")
                self.state = "CENTERING"
                return

            # Move forward + yaw correction
            self.publish_velocity(1.0, self.cx, 0.0)

        # -------- CENTERING --------
        elif self.state == "CENTERING":

            if abs(self.cx) < 0.05 and abs(self.cy) < 0.05:
                self.get_logger().info("Centered → DESCENDING")
                self.state = "DESCENDING"
                return

            self.publish_velocity(0.0, self.cx, self.cy)

        # -------- DESCENDING --------
        elif self.state == "DESCENDING":

            self.get_logger().info("Descending...")
            self.publish_velocity(0.0, 0.0, -0.5)

            time.sleep(2)

            self.state = "DROPPING"

        # -------- DROPPING --------
        elif self.state == "DROPPING":

            self.get_logger().info("Dropping payload")

            msg = Float32MultiArray()
            msg.data = [1.0]
            self.drop_pub.publish(msg)

            time.sleep(1)

            self.returning_from_drop = True
            self.state = "RETURNING"

        # -------- RETURNING --------
        elif self.state == "RETURNING":

            if self.checkpoint is None:
                self.get_logger().warn("No checkpoint → SEARCHING")
                self.state = "SEARCHING"
                return

            lat, lon = self.checkpoint
            self.publish_gps_target(lat, lon)

            if self.reached_checkpoint():
                self.get_logger().info("Reached checkpoint → SEARCHING")

                self.returning_from_drop = False
                self.state = "SEARCHING"

    # ======================================
    # WAYPOINT FOLLOWING
    # ======================================

    def follow_waypoints(self):

        if len(self.waypoints) == 0:
            return

        lat, lon = self.waypoints[self.current_wp]

        self.publish_gps_target(lat, lon)

        if self.reached_waypoint(lat, lon):
            self.get_logger().info(f"Reached WP {self.current_wp}")

            self.current_wp += 1

            if self.current_wp >= len(self.waypoints):
                self.get_logger().info("Mission complete")
                self.current_wp = 0

    # ======================================
    # HELPERS
    # ======================================

    def publish_velocity(self, forward, yaw, vertical):

        msg = Float32MultiArray()
        msg.data = [float(forward), float(yaw), float(vertical)]
        self.cmd_pub.publish(msg)

    def publish_gps_target(self, lat, lon):

        msg = Float32MultiArray()
        msg.data = [float(lat), float(lon)]
        self.gps_pub.publish(msg)

    def save_checkpoint(self):
        # Placeholder (replace with real GPS reading later)
        if self.current_wp < len(self.waypoints):
            self.checkpoint = self.waypoints[self.current_wp]
            self.get_logger().info(f"Checkpoint saved: {self.checkpoint}")

    def reached_waypoint(self, lat, lon):
        # Placeholder logic
        return True

    def reached_checkpoint(self):
        # Placeholder logic
        return True


def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()