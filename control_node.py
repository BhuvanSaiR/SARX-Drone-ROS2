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
            1
        )
        # ---- GPS Publisher ----
        self.gps_pub = self.create_publisher(
            Float32MultiArray,
            '/gps',
            1
        )
        # ---- Drone setup ----
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()
        self.yaw_scale = 30.0
        self.get_logger().info("Control node started")
        self.last_cmd_time = self.loop.time()
        self.received_first_cmd = False
        self.offboard_started = False

    # ============================
    # ASYNC LOOP (from your code)
    # ============================

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect())

    async def connect(self):

        self.get_logger().info("Connecting to drone...")

        try:
            await asyncio.wait_for(
                self.drone.connect(system_address="serial:///dev/ttyACM0:115200"),
                timeout=10
            )
        except asyncio.TimeoutError:
            self.get_logger().error("Connection timeout")
            return

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                break

        await self.drone.action.arm()
        await self.drone.action.takeoff()

        await asyncio.sleep(5)

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            self.get_logger().info("Offboard control enabled")
        except Exception as e:
            self.get_logger().error(f"Offboard failed: {e}")
            await self.drone.action.land()
            return

        asyncio.create_task(self.publish_gps())
        asyncio.create_task(self.command_watchdog())
        self.get_logger().info("Command watchdog started")

        self.get_logger().info("Offboard mode started")
        
    # ============================
    # COMMAND HANDLER
    # ============================

    def command_callback(self, msg):

        if len(msg.data) != 3:
            self.get_logger().warn("Invalid command received")
            return

        forward, yaw, vertical = msg.data

        self.loop.call_soon_threadsafe(
            asyncio.create_task,
            self.send_velocity(forward, yaw, vertical)
        )   
        self.received_first_cmd = True
        self.last_cmd_time = self.loop.time()

    async def send_velocity(self, forward, yaw, vertical):
        if not self.offboard_started:
            return
        try:
            forward = max(min(forward, 0.8), -0.5)
            vertical = max(min(vertical, 1.0), -1.0)
            yaw = max(min(yaw, 1.0), -1.0)

            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward,      # forward/back
                    0.0,          # right/left (not used yet)
                    vertical,     # up/down
                    yaw * self.yaw_scale  # yaw scaling
                )
            )

        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")

    async def publish_gps(self):
        async for position in self.drone.telemetry.position():

            lat = position.latitude_deg
            lon = position.longitude_deg

            msg = Float32MultiArray()
            msg.data = [lat, lon]

            self.gps_pub.publish(msg)
            await asyncio.sleep(0.1)

    def destroy_node(self):

        future = asyncio.run_coroutine_threadsafe(
            self.safe_shutdown(),
            self.loop
        )
        future.result(timeout=5)

        super().destroy_node()

    async def safe_shutdown(self):
        try:
            if self.offboard_started:
                await self.drone.offboard.stop()
            await self.drone.action.land()
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")
            
    async def command_watchdog(self):

        while True:
            await asyncio.sleep(0.1)
            if not self.offboard_started:
                continue

            if not self.received_first_cmd:
                continue

            if self.loop.time() - self.last_cmd_time > 0.5:
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                self.get_logger().warn("Command timeout → stopping drone")

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()