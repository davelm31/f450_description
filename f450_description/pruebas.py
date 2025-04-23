import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
import numpy as np
from f450_description.function import *


class MotorSpeedPublisher(Node):
    def __init__(self):
        super().__init__('motor_speed_publisher')
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

    def publish_motor_speeds(self):

        # Estos valores puedes luego recibirlos desde otro nodo o modificarlos
        u1 = 9000
        p = 3000
        q = 5000
        r = 2000

        omega= effort2velocityw(u1, p, q, r)

        # Publicar velocidades
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

def main(args=None):
    rclpy.init(args=args)
    motor_speed_publisher = MotorSpeedPublisher()
    rclpy.spin(motor_speed_publisher)
    motor_speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
