import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Imu, '/f450/imu', self.imu_callback, 10)

    def publish_motor_speeds(self):
        msg = Actuators()
        msg.velocity = [690, 690, 690, 690]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando velocidades: {msg.velocity}')

    def imu_callback(self, msg: Imu):
        # Mostrar la velocidad angular leída del IMU
        av = msg.angular_velocity
        self.get_logger().info(f'IMU -> Vel. angular: x={av.x:.2f}, y={av.y:.2f}, z={av.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
