import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import asyncio
import threading

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # ---- Subscribe to commands ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/command',
            self.command_callback,
            10
        )

        # ---- Drone setup ----
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Control node started")

    # ============================
    # ASYNC LOOP (from your code)
    # ============================

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect())

    async def connect(self):
        self.get_logger().info("Connecting to drone...")

        await self.drone.connect(system_address="serial:///dev/ttyACM0:115200")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

        await self.drone.action.arm()
        await self.drone.action.takeoff()

        await asyncio.sleep(5)

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        await self.drone.offboard.start()

        self.get_logger().info("Offboard mode started")

    # ============================
    # COMMAND HANDLER
    # ============================

    def command_callback(self, msg):

        forward, yaw, vertical = msg.data

        # Convert to MAVSDK command
        asyncio.run_coroutine_threadsafe(
            self.send_velocity(forward, yaw, vertical),
            self.loop
        )

    async def send_velocity(self, forward, yaw, vertical):

        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward,      # forward/back
                    0.0,          # right/left (not used yet)
                    vertical,     # up/down
                    yaw * 30.0    # yaw scaling
                )
            )
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()