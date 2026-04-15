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
            1
        )

        self.wp_sub = self.create_subscription(
            Float32MultiArray,
            '/mission_waypoints',
            self.waypoint_callback,
            1
        )

        # ============================
        # PUBLISHERS
        # ============================
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/command',
            1
        )

        self.drop_pub = self.create_publisher(
            Float32MultiArray,
            '/drop',
            1
        )

        self.gps_pub = self.create_publisher(
            Float32MultiArray,
            '/goto_gps',
            1
        )

        self.cam_pub = self.create_publisher(
            Float32MultiArray,
            '/active_camera',
            1
        )

        # ============================
        # TIMERS
        # ============================
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_detection_time = time.time()

        # ============================
        # CURRENT STATE DATA
        # ============================
        self.detected = False
        self.approach = False
        self.cx = 0.0
        self.cy = 0.0
        self.area = 0.0

        self.get_logger().debug("Mission node fully initialized")

        # ---- GPS State ----
        self.current_lat = None
        self.current_lon = None
        self.gps_sub = self.create_subscription(
            Float32MultiArray,
            '/gps',
            self.gps_callback,
            1
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
        data = msg.data

        if len(data) != 5:
            self.get_logger().warn("Invalid perception data")
            return

        detected, approach, cx, cy, area = data

        self.detected = detected > 0.5
        self.approach = approach > 0.5
        self.cx = cx
        self.cy = cy
        self.area = area

    def waypoint_callback(self, msg):

        data = msg.data
        self.waypoints = []

        for i in range(0, len(data), 2):
            lat = data[i]
            lon = data[i + 1]
            self.waypoints.append((lat, lon))

        self.current_wp = 0

        self.get_logger().debug(f"Loaded {len(self.waypoints)} waypoints")

    # ======================================
    # MAIN CONTROL LOOP
    # ======================================

    def control_loop(self):
        self.get_logger().debug(f"STATE: {self.state}")
        # -------- SEARCHING (Waypoint Following) --------
        if self.state == "SEARCHING":
            self.set_camera("front")
            if self.detected:
                self.get_logger().debug("Human detected → APPROACHING")

                self.save_checkpoint()
                self.state = "APPROACHING"
                self.set_camera("front")
                return

            self.follow_waypoints()

        # -------- APPROACHING --------
        elif self.state == "APPROACHING":

            if self.detected:
                self.last_detection_time = time.time()
            else:
                if time.time() - self.last_detection_time > 1.0:
                    self.get_logger().warn("Detection timeout → SEARCHING")
                    self.state = "SEARCHING"
                    return

                self.get_logger().debug("Lost target → CENTERING")
                self.state = "CENTERING"
                self.set_camera("bottom")
                return

            # Move forward + yaw correction
            forward_speed = max(0.2, min(0.8, 1.0 - self.area))  # Change to trapezium curve later
            yaw_cmd = max(min(self.cx, 0.5), -0.5)
            self.publish_velocity(forward_speed, yaw_cmd, 0.0)

        # -------- CENTERING --------
        elif self.state == "CENTERING":
            self.stop_motion()
            now = time.time()
            dt = now - self.prev_time
            dt = min(dt, 0.1)
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

            self.get_logger().debug(
                f"PID -> X: {output_x:.2f}, Y: {output_y:.2f}"
            )

            # Check centered condition
            if abs(error_x) < 0.03 and abs(error_y) < 0.03:
                self.get_logger().debug("Centered → DESCENDING")

                # Reset PID integrals for next phase
                self.integral_x = 0.0
                self.integral_y = 0.0

                self.state = "DESCENDING"
                return

            # Apply smooth control
            self.publish_velocity(0.0, output_x, output_y)

        # -------- DESCENDING --------
        elif self.state == "DESCENDING":
            self.stop_motion()

            self.get_logger().debug("Descending...")
            if abs(self.cx) < 0.05 and abs(self.cy) < 0.05:
                self.publish_velocity(0.0, 0.0, -0.5)
                self.descend_start = time.time()
                self.state = "WAIT_AFTER_DESCEND"
            else:
                self.get_logger().warn("Not centered → abort descent")
                self.state = "CENTERING"

        # -------- WAIT AFTER DESCEND --------
        elif self.state == "WAIT_AFTER_DESCEND":
            if time.time() - self.descend_start >= 2.0:
                self.state = "DROPPING"

        # -------- DROPPING --------
        elif self.state == "DROPPING":

            self.get_logger().debug("Dropping payload")

            msg = Float32MultiArray()
            msg.data = [1.0]
            self.drop_pub.publish(msg)

            self.drop_time = time.time()
            self.state = "WAIT_AFTER_DROP"
        # -------- WAIT AFTER DROP --------
        elif self.state == "WAIT_AFTER_DROP":
            if time.time() - self.drop_time >= 1.0:
                self.returning_from_drop = True
                self.stop_motion()
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
                self.get_logger().debug("Reached checkpoint → SEARCHING")

                self.returning_from_drop = False
                self.state = "SEARCHING"

    # ======================================
    # WAYPOINT FOLLOWING
    # ======================================

    def follow_waypoints(self):

        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints loaded")
            return

        if self.current_wp >= len(self.waypoints):
            return
        lat, lon = self.waypoints[self.current_wp]
        
        self.publish_gps_target(lat, lon)

        if self.reached_waypoint(lat, lon):
            self.get_logger().debug(f"Reached WP {self.current_wp}")

            self.current_wp += 1

            if self.current_wp >= len(self.waypoints):
                self.get_logger().debug("Mission complete")
                self.current_wp = 0
        
    # ======================================
    # HELPERS
    # ======================================

    def publish_velocity(self, forward, yaw, vertical):

        forward = max(min(forward, 1.0), -1.0)
        yaw = max(min(yaw, 1.0), -1.0)
        vertical = max(min(vertical, 1.0), -1.0)

        msg = Float32MultiArray()
        msg.data = [float(forward), float(yaw), float(vertical)]

        self.cmd_pub.publish(msg)

    def publish_gps_target(self, lat, lon):
        if self.current_lat is None:
            self.get_logger().warn("No GPS → holding position")
            return
        msg = Float32MultiArray()
        msg.data = [float(lat), float(lon)]
        self.gps_pub.publish(msg)

    def save_checkpoint(self):
        # Placeholder (replace with real GPS reading later)
        if self.current_wp < len(self.waypoints):
            if self.current_lat is not None:
                self.checkpoint = (self.current_lat, self.current_lon)
                self.get_logger().debug(f"Checkpoint saved: {self.checkpoint}")

    def reached_waypoint(self, lat, lon):

        if self.current_lat is None:
            return False

        dist = self.distance_meters(
            self.current_lat,
            self.current_lon,
            lat,
            lon
        )

        self.get_logger().debug(f"Distance to WP: {dist:.2f} m")

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

        self.get_logger().debug(f"Distance to checkpoint: {dist:.2f} m")

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

        # Integral Clamping to prevent windup
        integral = max(min(integral, 1.0), -1.0)

        derivative = (error - prev_error) / dt if dt > 0 else 0.0

        output = (
            self.kp * error +
            self.ki * integral +
            self.kd * derivative
        )

        return output, integral

    def set_camera(self, cam):

        if not hasattr(self, 'current_camera'):
            self.current_camera = None

        if self.current_camera == cam:
            return

        self.current_camera = cam

        msg = Float32MultiArray()

        if cam == "front":
            msg.data = [0.0]
        else:
            msg.data = [1.0]

        self.cam_pub.publish(msg)
        self.get_logger().debug(f"Switched to {cam} camera")

    def stop_motion(self):
        self.publish_velocity(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()