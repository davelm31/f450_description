import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from f450_description.function import *



# Nodo de ROS2 para el controlador del dron
class DroneOrientation(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Inicialización de las variables de roll, pitch y yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Imu, '/f450/imu', self.imu_callback, 10)

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

    def imu_callback(self, msg: Imu):
        # Extraer los valores de p, q, r desde el mensaje del IMU
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        # Llamar a var2vasi para obtener las derivadas de roll, pitch, yaw
        euler_dot = var2vasi(self.roll, self.pitch, self.yaw, p, q, r)

        # Integrar las derivadas para obtener los nuevos valores de roll, pitch, yaw
        dt = 0.1  
        self.roll += euler_dot[0][0] * dt
        self.pitch += euler_dot[1][0] * dt
        self.yaw += euler_dot[2][0] * dt

        # Mostrar los valores de roll, pitch, yaw actualizados
        self.get_logger().info(f'Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}')

    def publish_motor_speeds(self):
        msg = Actuators()
        msg.velocity = [670, 670, 670, 670]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando velocidades: {msg.velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneOrientation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
