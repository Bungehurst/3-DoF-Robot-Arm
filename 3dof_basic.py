import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from sympy import *

def DH_2_Trans(dh_params):
        alpha, a, d, theta = dh_params
        return Matrix([ [cos(theta), -sin(theta), 0., a],
                        [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                        [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),  cos(alpha) * d],
                        [0., 0., 0., 1.]])

if __name__ == '__main__':
    theta1,theta2,theta3,delta_theta = symbols('theta1 theta2 theta3 delta_theta')
    h1,h2,h3,h4 = symbols('h1 h2 h3 h4')
    x,y,z = symbols('x y z')
    h1,h2,h3,h4 = 10.,19.2,2.8,21.
    delta_theta = atan2(h3,h2)

    theta1,theta2,theta3 = pi/3,pi/2,-delta_theta
    # verify the result with FK model
    DH_Table = [[0,0,h1,theta1+pi/2],
                [pi/2,0,0,theta2-delta_theta],
                [0,sqrt(h2**2+h3**2),0,pi/2+theta3+delta_theta],
                [pi/2,0,h4,0]]

    T_01 = DH_2_Trans(DH_Table[0])
    T_02 = T_01*DH_2_Trans(DH_Table[1])
    T_03 = T_02*DH_2_Trans(DH_Table[2])
    T_04 = T_03*DH_2_Trans(DH_Table[3])

    T_01 = np.array(T_01).astype(np.float64)
    T_02 = np.array(T_02).astype(np.float64)
    T_03 = np.array(T_03).astype(np.float64)
    T_04 = np.array(T_04).astype(np.float64)
    
    soa = np.array([[0, 0, 0], 
                    np.transpose(T_01)[3][:3], 
                    np.transpose(T_02)[3][:3], 
                    np.transpose(T_03)[3][:3],
                    np.transpose(T_04)[3][:3]])

    X, Y, Z = zip(*soa)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(left=-30,right=30)
    ax.set_ylim(bottom=-30,top=30)
    ax.plot(X, Y, Z)
    ax.scatter(X[:-1], Y[:-1], Z[:-1], c='black', s=20)
    ax.scatter(X[-1], Y[-1], Z[-1], c='red', s=20)
    plt.show()

    
    