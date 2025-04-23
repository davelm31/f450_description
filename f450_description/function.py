import numpy as np
from copy import copy
cos=np.cos; sin=np.sin; pi=np.pi

#rotaiones
def rotx(theta):
    Rx = np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])
    return Rx
def roty(theta):
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])
    return Ry
def rotz(theta):
    Rz = np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
    return Rz


#Dron respecto al sistema inercial
#R = RxRyRz
def I_R_D(roll,pitch,yaw):
    R = rotx(roll) @ roty(pitch) @ rotz(yaw)
    return R


#fuerzas y torques vs velocidad angular

def effort2velocityw(u1, p, q, r):
    """
    Calcula las velocidades angulares omega_i a partir de los comandos
    generales de control: empuje total (u1), y momentos (p, q, r).

    Retorna:
    - omega: array de 4 elementos con las velocidades angulares de los motores
    """
    M = 0.25 * np.array([
        [1, 1, 1, 1],
        [1, 1, -1, -1],
        [1, -1, -1, 1],
        [1, -1, 1, -1],
    ])

    u_vec = np.array([u1, p, q, r])
    omega_squared = M @ u_vec

    # Asegurar que no haya valores negativos antes de la raíz cuadrada
    omega_squared = np.clip(omega_squared, 0, None)

    omega = np.sqrt(omega_squared)
    return omega


# relacion velocidad angualar del robot y velocidad angular en el sistema inercial

def var2vasi(roll,pitch,yaw,p,q,r):
    """
    Recordar que pdot, qdot, rdot son bar ux,uy,z
    es lo que se quiere controlar
    """

    E_0 = np.array([
                    [1, 0, -sin(pitch)],  
                    [0, cos(roll), sin(roll)*cos(pitch)],
                    [0, -sin(roll), cos(roll)*cos(pitch)]
    ])

    # Inversa de la matriz de rotación
    E_inv = np.linalg.inv(E_0)
    R = I_R_D(roll,pitch,yaw)

    vel = np.array([
                    [p],  # p (velocidad angular alrededor del eje X)
                    [q],  # q (velocidad angular alrededor del eje Y)
                    [r]   # r (velocidad angular alrededor del eje Z)
    ])

    euler_dot = E_inv @ R @ vel
    return euler_dot


#planta 1 - movimiento rotacional - control/torque
def p_torque(Ix, Iy, Iz, pdot, qdot, rdot,p,q,r):
    """
    Recordar que pdot, qdot, rdot son bar ux,uy,z
    es lo que se quiere controlar
    """

    ux = Ix * pdot + (Iz - Iy) * q * r
    uy = Iy * qdot + (Ix - Iy) * p * r
    uz = Iz * rdot + (Iy - Ix) * p * q
    return ux

#plnata 2 - derivada de la matriz de rotacion - c
def p_velocity(R, dot_bc_x, dot_bc_y):
    """
    Calcula p_c y q_c y lo que contrla es la orientacion
    dot_bc_i es la variable de control
    """
    r11, r12, r21, r22, r33 = R[0, 0], R[0, 1], R[1, 0], R[1, 1], R[2, 2]
    
    if r33 == 0:
        raise ValueError("r33 no puede ser cero para evitar división por cero.")
    
    J = (1 / r33) * np.array([
        [r21, -r11],
        [r22, -r12]
    ])
    
    b_dot = np.array([dot_bc_x, dot_bc_y])
    pq = J @ b_dot
    return pq

#planta 1 - movimiento translacional
def p_posicion(ddot_x_c, ddot_y_c, u1):
    """
    Calcula b_c^x y b_c^y a partir de las aceleraciones del centro de masa
    en x e y, y del empuje total u1.
    Donde ddotx y ddot y y u son variables controladas

    Parámetros:
    - ddot_x_c: float — Aceleración del centro de masa en x
    - ddot_y_c: float — Aceleración del centro de masa en y
    - u1: float — Empuje total
    """
    if u1 == 0:
        raise ValueError("u1 no puede ser cero para evitar división por cero.")

    b_c_x = ddot_x_c / u1
    b_c_y = ddot_y_c / u1

    return b_c_x, b_c_y
