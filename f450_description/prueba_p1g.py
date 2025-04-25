import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from f450_description.function import *

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

from threading import Event

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(0.1, self.publish_motor_speeds)
        self.subscription = self.create_subscription(Imu, '/f450/imu', self.imu_callback, 10)

        self.p = self.q = self.r = 0.0
        self.pc, self.qc, self.rc = 0.1, 0.1, 0.2

        self.times = deque(maxlen=100)
        self.pdot_list = deque(maxlen=100)
        self.qdot_list = deque(maxlen=100)
        self.rdot_list = deque(maxlen=100)
        self.ux_list = deque(maxlen=100)
        self.uy_list = deque(maxlen=100)
        self.uz_list = deque(maxlen=100)
        self.p_list = deque(maxlen=100)
        self.q_list = deque(maxlen=100)
        self.r_list = deque(maxlen=100)
        self.t0 = self.get_clock().now().seconds_nanoseconds()[0]

    def publish_motor_speeds(self):
        Ix, Iy, Iz = 2.547e7, 2.516e7, 4.892e7
        kp = 0.1

        pdot_des = kp * (self.pc - self.p)
        qdot_des = kp * (self.qc - self.q)
        rdot_des = kp * (self.rc - self.r)
        ux, uy, uz = p_torque(Ix, Iy, Iz, pdot_des, qdot_des, rdot_des, self.p, self.q, self.r)

        t = self.get_clock().now().seconds_nanoseconds()[0] - self.t0
        self.times.append(t)
        self.pdot_list.append(pdot_des)
        self.qdot_list.append(qdot_des)
        self.rdot_list.append(rdot_des)
        self.ux_list.append(ux)
        self.uy_list.append(uy)
        self.uz_list.append(uz)
        self.p_list.append(self.p)
        self.q_list.append(self.q)
        self.r_list.append(self.r)

        u1 = 200
        omega = effort2velocityw(u1, ux, uy, uz)

        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)

    def imu_callback(self, msg: Imu):
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

def start_visualization(node: DroneController, stop_event: Event):
    plt.style.use('seaborn-darkgrid')
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle('Drone Dynamics')

    def update(frame):
        axs[0].clear()
        axs[1].clear()
        axs[2].clear()

        axs[0].plot(node.times, node.pdot_list, label='pdot_des')
        axs[0].plot(node.times, node.qdot_list, label='qdot_des')
        axs[0].plot(node.times, node.rdot_list, label='rdot_des')
        axs[0].legend()
        axs[0].set_ylabel("Angular Accelerations")

        axs[1].plot(node.times, node.ux_list, label='ux')
        axs[1].plot(node.times, node.uy_list, label='uy')
        axs[1].plot(node.times, node.uz_list, label='uz')
        axs[1].legend()
        axs[1].set_ylabel("Torques")

        axs[2].plot(node.times, node.p_list, label='p')
        axs[2].plot(node.times, node.q_list, label='q')
        axs[2].plot(node.times, node.r_list, label='r')
        if node.times:
            axs[2].hlines(node.pc, node.times[0], node.times[-1], colors='r', linestyles='dashed', label='pc')
            axs[2].hlines(node.qc, node.times[0], node.times[-1], colors='g', linestyles='dashed', label='qc')
            axs[2].hlines(node.rc, node.times[0], node.times[-1], colors='b', linestyles='dashed', label='rc')
        axs[2].legend()
        axs[2].set_ylabel("p, q, r")
        axs[2].set_xlabel("Time (s)")

    ani = FuncAnimation(fig, update, interval=500)

    def on_close(event):
        stop_event.set()

    fig.canvas.mpl_connect('close_event', on_close)
    plt.tight_layout()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()

    stop_event = Event()

    def ros_spin():
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    try:
        start_visualization(node, stop_event)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
