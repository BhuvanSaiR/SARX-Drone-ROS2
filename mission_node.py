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

        # ---- GPS State ----
        self.current_lat = None
        self.current_lon = None
        self.gps_sub = self.create_subscription(
            Float32MultiArray,
            '/gps',
            self.gps_callback,
            10
        )
        # ============================
        # PID PARAMETERS
        # ============================

        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.2

        # X-axis (left-right / yaw)
        self.prev_error_x = 0.0
        self.integral_x = 0.0

        # Y-axis (up-down)
        self.prev_error_y = 0.0
        self.integral_y = 0.0

        self.prev_time = time.time()


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

            now = time.time()
            dt = now - self.prev_time
            self.prev_time = now

            error_x = self.cx
            error_y = self.cy

            # PID for X (yaw)
            output_x, self.integral_x = self.pid_control(
                error_x,
                self.prev_error_x,
                self.integral_x,
                dt
            )

            # PID for Y (vertical)
            output_y, self.integral_y = self.pid_control(
                error_y,
                self.prev_error_y,
                self.integral_y,
                dt
            )

            self.prev_error_x = error_x
            self.prev_error_y = error_y

            # Clamp outputs (VERY IMPORTANT)
            output_x = max(min(output_x, 1.0), -1.0)
            output_y = max(min(output_y, 1.0), -1.0)

            self.get_logger().info(
                f"PID -> X: {output_x:.2f}, Y: {output_y:.2f}"
            )

            # Check centered condition
            if abs(error_x) < 0.03 and abs(error_y) < 0.03:
                self.get_logger().info("Centered → DESCENDING")

                # Reset PID integrals for next phase
                self.integral_x = 0.0
                self.integral_y = 0.0

                self.state = "DESCENDING"
                return

            # Apply smooth control
            self.publish_velocity(0.0, output_x, output_y)

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
        forward = max(min(forward, 1.0), -1.0)
        yaw = max(min(yaw, 1.0), -1.0)
        vertical = max(min(vertical, 1.0), -1.0)

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

        if self.current_lat is None:
            return False

        dist = self.distance_meters(
            self.current_lat,
            self.current_lon,
            lat,
            lon
        )

        self.get_logger().info(f"Distance to WP: {dist:.2f} m")

        return dist < self.wp_tolerance

    def reached_checkpoint(self):

        if self.checkpoint is None:
            return False

        if self.current_lat is None:
            return False

        lat, lon = self.checkpoint

        dist = self.distance_meters(
            self.current_lat,
            self.current_lon,
            lat,
            lon
        )

        self.get_logger().info(f"Distance to checkpoint: {dist:.2f} m")

        return dist < self.wp_tolerance
    
    def gps_callback(self, msg):
        self.current_lat = msg.data[0]
        self.current_lon = msg.data[1] 

    def distance_meters(self, lat1, lon1, lat2, lon2):

        R = 6371000  # meters

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c 
    def pid_control(self, error, prev_error, integral, dt):

        integral += error * dt
        derivative = (error - prev_error) / dt if dt > 0 else 0.0

        output = (
            self.kp * error +
            self.ki * integral +
            self.kd * derivative
        )

        return output, integral
    
def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()