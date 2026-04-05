import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil
import time


class ActuatorNode(Node):

    def __init__(self):
        super().__init__('actuator_node')

        # ---- Subscribe to drop command ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/drop',
            self.drop_callback,
            10
        )

        # ---- Connect to Pixhawk ----
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.master.wait_heartbeat()

        self.get_logger().info("Actuator connected to Pixhawk")

        # ---- Servo setup ----
        self.SERVO_NUMBERS = [9, 10, 11, 12, 13]
        self.SERVO_OPEN = 1400
        self.SERVO_CLOSE = 900

        self.current_servo_index = 0

        # Initialize all servos to closed
        for servo in self.SERVO_NUMBERS:
            self.set_servo(servo, self.SERVO_CLOSE)
            time.sleep(0.2)

        self.get_logger().info("Servos initialized")

    # ============================
    # DROP HANDLER
    # ============================

    def drop_callback(self, msg):

        trigger = msg.data[0]

        if trigger != 1.0:
            return

        if self.current_servo_index >= len(self.SERVO_NUMBERS):
            self.get_logger().warn("All payloads already dropped")
            return

        servo = self.SERVO_NUMBERS[self.current_servo_index]

        self.get_logger().info(f"Dropping payload from servo {servo}")

        # Open servo
        self.set_servo(servo, self.SERVO_OPEN)

        time.sleep(1)

        # (optional) keep open or close again
        # self.set_servo(servo, self.SERVO_CLOSE)

        self.current_servo_index += 1

    # ============================
    # SERVO COMMAND
    # ============================

    def set_servo(self, servo_number, pwm):

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm,
            0, 0, 0, 0, 0
        )


def main():
    rclpy.init()
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()