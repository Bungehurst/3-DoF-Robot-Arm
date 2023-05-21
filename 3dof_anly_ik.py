import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from sympy import *
import math
def DH_2_Trans(dh_params):
        alpha, a, d, theta = dh_params
        return Matrix([ [cos(theta), -sin(theta), 0., a],
                        [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                        [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),  cos(alpha) * d],
                        [0., 0., 0., 1.]])

if __name__ == '__main__':
    h1,h2,h3,h4 = 10.,19.2,2.8,21.
    delta_theta = np.arctan2(h3,h2)
    data = {'x':[],'y':[],'z':[],'xx':[],'yy':[],'zz':[],'so':[]}

    joint_space = {'theta1':[],'theta2':[],'theta3':[]}

    # generate trajectory in task space then convert to joint space using analitical IK model
    for t in np.linspace(0,4,100):
        x = 5+3*math.cos(math.pi/2*t)
        y = 10
        z = 5+3*math.sin(math.pi/2*t)

        data['x'].append(x)
        data['y'].append(y)
        data['z'].append(z)

        theta1 = math.atan(-x/y)
        A,B = -x * math.sin(theta1) + y * math.cos(theta1), z - h1
        theta3 = -math.fabs(math.acos((A ** 2 + B ** 2 - h2 ** 2 - h3 ** 2 - h4 ** 2) / (2 * math.sqrt(h2**2+h3**2) * h4))) - delta_theta
        theta2 = math.atan((2*A*(h2**2*math.sin(delta_theta)**2 + h3**2*math.sin(delta_theta)**2 + h4**2*math.sin(theta3)**2 - 2*h4*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta)*math.sin(theta3))*\
                    (-2*h2**2*h4*math.sin(theta3) + h2**2*h4*math.sin(2*delta_theta + theta3) + h2**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - 2*h3**2*h4*math.sin(theta3) + \
                    h3**2*h4*math.sin(2*delta_theta + theta3) + h3**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - h4**3*math.sin(theta3) + 2*h4**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - \
                    h4**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta + 2*theta3)) + B*(h2**2*math.sin(2*delta_theta) + h3**2*math.sin(2*delta_theta) - h4**2*math.sin(2*theta3) + 2*h4*math.sqrt(h2**2 + \
                    h3**2)*math.sin(delta_theta - theta3))*(2*h2**2*h4*math.sin(delta_theta)*math.cos(delta_theta + theta3) - h2**2*h4*math.sin(theta3) + h2**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) + \
                    2*h3**2*h4*math.sin(delta_theta)*math.cos(delta_theta + theta3) - h3**2*h4*math.sin(theta3) + h3**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - h4**3*math.sin(theta3) + h4**2*math.sqrt(h2**2 + \
                    h3**2)*math.sin(delta_theta) - 2*h4**2*math.sqrt(h2**2 + h3**2)*math.sin(theta3)*math.cos(delta_theta + theta3)))*(h2**2 + h3**2 + h4**2 + 2*h4*math.sqrt(h2**2 + h3**2)*math.cos(delta_theta + theta3))/(2*(A*(h4*math.cos(theta3) + \
                    math.sqrt(h2**2 + h3**2)*math.cos(delta_theta)) + B*(h4*math.sin(theta3) - math.sqrt(h2**2 + h3**2)*math.sin(delta_theta)))*(-2*h2**2*h4*math.sin(theta3) + h2**2*h4*math.sin(2*delta_theta + theta3) + \
                    h2**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - 2*h3**2*h4*math.sin(theta3) + h3**2*h4*math.sin(2*delta_theta + theta3) + h3**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - \
                    h4**3*math.sin(theta3) + 2*h4**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - h4**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta + 2*theta3))*(2*h2**2*h4*math.sin(delta_theta)*math.cos(delta_theta + \
                    theta3) - h2**2*h4*math.sin(theta3) + h2**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) + 2*h3**2*h4*math.sin(delta_theta)*math.cos(delta_theta + theta3) - h3**2*h4*math.sin(theta3) + \
                    h3**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - h4**3*math.sin(theta3) + h4**2*math.sqrt(h2**2 + h3**2)*math.sin(delta_theta) - 2*h4**2*math.sqrt(h2**2 + h3**2)*math.sin(theta3)*math.cos(delta_theta + theta3))))
        joint_space['theta1'].append(theta1)
        joint_space['theta2'].append(theta2)
        joint_space['theta3'].append(theta3)

    for t in range(len(joint_space['theta1'])):
        theta1 = joint_space['theta1'][t]
        theta2 = joint_space['theta2'][t]
        theta3 = joint_space['theta3'][t]

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

        # store datas
        data['so'].append(soa)
        tmp = np.transpose(T_04)[3][:3]
        data['xx'].append(tmp[0])
        data['yy'].append(tmp[1])
        data['zz'].append(tmp[2])

    # np.savetxt("theta1_anl.txt",np.array(joint_space['theta1']))
    # np.savetxt("theta2_anl.txt",np.array(joint_space['theta2']))
    # np.savetxt("d3_anl.txt",np.array(joint_space['d3']))
    # show realtime moving 
    t = 0
    plt.ioff()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("end-effector trajectory")
    with plt.ion():
        fig.gca()
        for da in data['so']:
            ax.cla()
            X, Y, Z = zip(*da)
            plt.xlim(-30, 30)
            plt.ylim(-30, 30)
            ax.plot(X, Y, Z)
            ax.scatter(X, Y, Z, c='red', s=3)
            X, Y, Z = data['xx'][:t],data['yy'][:t],data['zz'][:t]
            X1, Y1, Z1 = data['x'][:t],data['y'][:t],data['z'][:t]
            t += 1
            ax.plot(X, Y, Z, color='red')
            ax.plot(X1, Y1, Z1, color='blue')
            plt.draw()
            plt.pause(0.01)
    plt.show()
    
    # show position error
    fig1 = plt.figure()
    bx = fig1.add_subplot(111)
    bx.set_title("position error")
    bx.set_xlabel('t')
    bx.set_ylabel('error')
    error = []
    for t in range(len(data['x'])):
        e = sqrt((data['x'][t]-data['xx'][t])**2\
                    +(data['y'][t]-data['yy'][t])**2\
                    +(data['z'][t]-data['zz'][t])**2)
        error.append(e.evalf())
    bx.plot(error)
    np.savetxt("pos_error_anl.txt",np.array(error))
    plt.show()

    # fig2 = plt.figure()
    # cx = fig2.add_subplot(111)
    # cx.set_title("q in joint space")
    # cx.set_xlabel('t')
    # cx.plot(joint_space['theta1'],color='red',label='theta1')
    # cx.plot(joint_space['theta2'],color='green',label='theta2')
    # cx.plot(joint_space['d3'],color='blue',label='d3')
    # fig2.legend()
    
    