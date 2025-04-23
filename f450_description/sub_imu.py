import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/f450/imu',  # Asegúrate que este sea el nombre correcto del tópico
            self.imu_callback,
            10
        )
        self.subscription  # evitar advertencia de variable no utilizada

    def imu_callback(self, msg):
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        self.get_logger().info(f'Velocidad angular -> x: {wx:.4f}, y: {wy:.4f}, z: {wz:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
