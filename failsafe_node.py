import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math

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

        self.wp_sub = self.create_subscription(
            Float32MultiArray,
            '/mission_waypoints',
            self.waypoint_callback,
            1
        )

        self.waypoints = []

        self.rtl_pub = self.create_publisher(
            Float32MultiArray,
            '/goto_gps',
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
        self.current_lat = None
        self.current_lon = None
        self.kill = False
        self.geofence_ready = False
        self.timer = self.create_timer(0.1, self.monitor)
        self.rtl_triggered = False
        self.geofence_margin = 0.00015   
        self.last_failsafe_reason = None
        self.get_logger().debug("Failsafe node started")

    # ============================
    # CALLBACKS
    # ============================

    def gps_callback(self, msg):
        self.last_gps_time = time.time()

        self.current_lat = msg.data[0]
        self.current_lon = msg.data[1]

    def perception_callback(self, msg):
        detected = msg.data[0]
        if detected > 0.5:
            self.last_detection_time = time.time()

    def kill_callback(self, msg):
        if msg.data[0] == 1.0:
            self.kill = True

    def waypoint_callback(self, msg):

        data = msg.data
        self.waypoints = []

        for i in range(0, len(data), 2):
            lat = data[i]
            lon = data[i + 1]
            self.waypoints.append((lat, lon))

        if len(self.waypoints) > 0:
            self.compute_geofence()
            self.geofence_ready = True
            
        if len(self.waypoints) == 0:
            self.geofence_ready = False

        self.get_logger().debug(f"Loaded {len(self.waypoints)} waypoints")
    
    def nearest_waypoint(self): # need to be optimised

        if not self.waypoints or self.current_lat is None:
            return None

        min_dist = float('inf')
        nearest = None

        for wp in self.waypoints:
            d = self.distance(
                self.current_lat,
                self.current_lon,
                wp[0],
                wp[1]
            )

            if d < min_dist:
                min_dist = d
                nearest = wp

        return nearest

    # ============================
    # HELPERS
    # ============================

    def publish_goto(self, wp, reason):

        lat, lon = wp

        msg = Float32MultiArray()
        msg.data = [lat, lon]

        self.rtl_pub.publish(msg)

        self.get_logger().warn(f"{reason}: {wp}")

    def compute_geofence(self):

        lats = [wp[0] for wp in self.waypoints]
        lons = [wp[1] for wp in self.waypoints]

        self.min_lat = min(lats) - self.geofence_margin
        self.max_lat = max(lats) + self.geofence_margin
        self.min_lon = min(lons) - self.geofence_margin
        self.max_lon = max(lons) + self.geofence_margin

    def outside_geofence(self):

        if self.current_lat is None:
            return False

        margin = self.geofence_margin * 1.5  # hysteresis

        return not (
            (self.min_lat + margin) <= self.current_lat <= (self.max_lat - margin) and
            (self.min_lon + margin) <= self.current_lon <= (self.max_lon - margin)
    )
    
    def distance(self, lat1, lon1, lat2, lon2):

        R = 6371000

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2)**2 + \
            math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2

        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    # ============================
    # MAIN MONITOR LOOP
    # ============================

    def monitor(self):

        if self.current_lat is None:
            self.publish_zero("WAITING FOR GPS")
            return
        
        now = time.time()

        # KILL
        if self.kill:
            self.publish_zero("KILL")
            return

        # GPS LOSS
        if now - self.last_gps_time > 1.0:
            self.publish_zero("GPS LOST")
            return

        # COMMAND LOSS (FIXED)
        if now - self.last_cmd_time > 0.5:
            self.publish_zero("COMMAND LOST")

            if now - self.last_cmd_time > 2.0 and not self.rtl_triggered:
                if self.current_lat is not None and self.current_lon is not None:
                    nearest = self.nearest_waypoint()
                if nearest:
                    self.publish_goto(nearest, "COMMAND LOST → RTL")
                    self.rtl_triggered = True
                    self.get_logger().warn("FAILSAFE TRIGGER → COMMAND LOST")
                else:
                    self.publish_zero("NO WAYPOINT → HOVER")

            return

        # GEOFENCE
        if self.geofence_ready and self.outside_geofence():

            if not self.rtl_triggered:
                if self.current_lat is not None and self.current_lon is not None:
                    nearest = self.nearest_waypoint()
                if nearest:
                    self.publish_goto(nearest, "GEOFENCE → NEAREST WP")
                    self.rtl_triggered = True
                    self.get_logger().warn("FAILSAFE TRIGGER → GEOFENCE")
                else:
                    self.publish_zero("NO WAYPOINT → HOVER")
                return
            
        # Reset RTL if back inside geofence
        if self.geofence_ready and not self.outside_geofence():
            self.rtl_triggered = False

        # DETECTION LOSS (throttled)
        if now - self.last_detection_time > 5.0:
            if int(time.time()) % 2 == 0:
                self.get_logger().warn("Detection lost → continuing mission")

        self.last_failsafe_reason = None
        # NORMAL
        if not hasattr(self, 'latest_cmd'):
            self.publish_zero("WAITING FOR COMMAND")
            return
        self.cmd_pub.publish(self.latest_cmd)

    # ============================
    # HELPERS
    # ============================

    def publish_zero(self, reason):

        if self.last_failsafe_reason != reason:
            self.get_logger().warn(f"FAILSAFE: {reason}")
            self.last_failsafe_reason = reason

        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]

        self.cmd_pub.publish(msg)

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        self.latest_cmd = msg
        self.rtl_triggered = False

def main():
    rclpy.init()
    node = FailsafeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()