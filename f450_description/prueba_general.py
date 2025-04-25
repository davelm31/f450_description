import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from f450_description.function import *

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 50)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Imu, '/f450/imu', self.imu_callback, 10)

        # Inicializar aceleraciones actuales
        self.ddx = 0.0
        self.ddy = 0.0
        self.ddz = 0.0
        # Inicializar aceleraciones actuales
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        # Inicializar aceleraciones actuales
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Estado actual de p, q, r
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # Inicialización de las variables de roll, pitch y yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def publish_motor_speeds(self):

        R = I_R_D(self.roll,self.pitch,self.yaw)
        # Llamar a var2vasi para obtener las derivadas de roll, pitch, yaw
        euler_dot = var2vasi(self.roll, self.pitch, self.yaw, self.p, self.q, self.r)

        # Integrar las derivadas para obtener los nuevos valores de roll, pitch, yaw
        dt = 0.1  
        self.roll += euler_dot[0][0] * dt
        self.pitch += euler_dot[1][0] * dt
        self.yaw += euler_dot[2][0] * dt

        # Aceleración del cuerpo → mundo
        acc_body = np.array([self.ddx, self.ddy, self.ddz])
        acc_world = R @ acc_body

        # Integrar para obtener velocidad
        self.dx += acc_world[0] * dt
        self.dy += acc_world[1] * dt
        self.dz += acc_world[2] * dt

        # Integrar para obtener posición
        self.x += self.dx * dt
        self.y += self.dy * dt
        self.z += self.dz * dt

        kp, kd, = 3, 0.01
        zd, dzd, ddzd = 0.5, 0, 0

        ddz = kp*(zd - self.z) + kd *(dzd - self.dz) + ddzd
        u1 = (ddz + 9.81)/R[2,2]
        print(self.z)

        u1 = (u1*2.013127)/8.54858e-06
        ux = 0
        uy = 0
        uz = 0

        omega= effort2velocityw(u1, ux, uy, uz)

        # Publicar velocidades
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

    def imu_callback(self, msg: Imu):

        # Leer aceleración lineal y velocida angular
        self.ddx = msg.linear_acceleration.x
        self.ddy = msg.linear_acceleration.y
        self.ddz = msg.linear_acceleration.z

        # Actualizar p, q, r con valores del IMU
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

        # Mostrar aceleraciones
        #self.get_logger().info(
         #   f'IMU -> Aceleración lineal: x={self.ddx:.2f}, y={self.ddy:.2f}, z={self.ddz:.2f}'
        #)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
