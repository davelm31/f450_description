import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators

class MotorSpeedPublisher(Node):
    def __init__(self):
        super().__init__('motor_speed_publisher')
        
        # Crear el publicador en el tópico /gazebo/command/motor_speed
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        
        # Publicar las velocidades de los motores
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)  # Publicar cada 1 segundo

    def publish_motor_speeds(self):
        msg = Actuators()
        
        # Asumimos que estamos enviando las velocidades de 4 motores
        # Puedes modificar los valores para que se ajusten a tu configuración
        msg.velocity = [690, 690, 690, 690]  # Velocidades de los 4 motores

        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

def main(args=None):
    rclpy.init(args=args)
    motor_speed_publisher = MotorSpeedPublisher()
    rclpy.spin(motor_speed_publisher)

    # Cerrar el nodo de manera segura
    motor_speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
