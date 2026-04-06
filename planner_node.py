import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import custom_survey as cs
import numpy as np


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        # Publisher
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/mission_waypoints',
            10
        )

        # Run once after startup
        self.timer = self.create_timer(2.0, self.generate_plan)

        self.generated = False

    def generate_plan(self):

        if self.generated:
            return

        self.get_logger().info("Generating survey waypoints...")

        PLAN_FILE = "/home/drone/Desktop/kml/mission.plan"

        # ---- LOAD POLYGON ----
        poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = \
            cs.load_polygon_from_plan_in_meters(PLAN_FILE)

        # ---- TAKEOFF POINT ----
        tx, ty, _ = cs.longest_side_midpoint(poly_m)
        takeoff_xy = (tx, ty)

        # ---- SPLIT ----
        poly1, poly2, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)

        separation_m = 15.0

        # ---- PATHS ----
        _, path1, _, _, _ = cs.find_best_angle_for_region(
            poly1, separation_m, takeoff_xy
        )

        _, path2, _, _, _ = cs.find_best_angle_for_region(
            poly2, separation_m, takeoff_xy
        )

        # ---- COMBINE ----
        full_path = list(path1.coords) + list(path2.coords)

        waypoints = []

        for x, y in full_path:

            lat = lat0 + (y / m_per_deg_lat)
            lon = lon0 + (x / m_per_deg_lon)

            waypoints.append((lat, lon))

        # ---- PUBLISH ----
        msg = Float32MultiArray()

        # flatten [(lat,lon),...] → [lat1,lon1,lat2,lon2,...]
        flat = []
        for lat, lon in waypoints:
            flat.extend([lat, lon])

        msg.data = flat

        self.pub.publish(msg)

        self.get_logger().info(f"Published {len(waypoints)} waypoints")

        self.generated = True


def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()