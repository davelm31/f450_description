import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from f450_description.function import *

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Imu, '/f450/imu', self.imu_callback, 10)

        # Estado actual de p, q, r
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        

    def publish_motor_speeds(self):
        # Parámetros del dron (inercia)
        Ix, Iy, Iz = 2.547*10**7, 2.516*10**7, 4.892*10**7

        # Velocidades angulares deseadas (aceleraciones)
        kp = 0.1
        pc,qc,rc = 0,0,5

        pdot_des = kp*(pc - self.p)
        qdot_des = kp*(qc - self.p)
        rdot_des = kp*(rc - self.p)

        # Calcular torques deseados
        ux, uy, uz = p_torque(Ix, Iy, Iz, pdot_des, qdot_des, rdot_des, self.p, self.q, self.r)
        self.get_logger().info(f"Torques deseados: ux={ux}, uy={uy}, uz={uz}")

        # Simulación de empuje total (u1) y paso a velocidades
        u1 = 10 # Empuje vertical deseado

        # Convertir torques + u1 en velocidades angulares de motores
        omega = effort2velocityw(u1, ux, uy, uz)

        # Publicar velocidades
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

    def imu_callback(self, msg: Imu):
        # Actualizar p, q, r con valores del IMU
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

        self.get_logger().info(f'IMU -> Vel. angular: x={self.p:.2f}, y={self.q:.2f}, z={self.r:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()