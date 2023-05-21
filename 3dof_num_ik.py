import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from sympy import *
import IPython.display as disp
def DH_2_Trans(dh_params):
        alpha, a, d, theta = dh_params
        return Matrix([ [cos(theta), -sin(theta), 0., a],
                        [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                        [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),  cos(alpha) * d],
                        [0., 0., 0., 1.]])

if __name__ == '__main__':
    # theta1,theta2,theta3,delta_theta = symbols('theta1 theta2 theta3 delta_theta')
    # h1,h2,h3,h4 = symbols('h1 h2 h3 h4')
    # x,y,z = symbols('x y z')
    h1,h2,h3,h4 = 10.,19.2,2.8,21.
    delta_theta = atan2(h3,h2)

    data = {'x':[],'y':[],'z':[],'xx':[],'yy':[],'zz':[],'so':[]}

    joint_space = {'theta1':[],'theta2':[],'theta3':[]}

    # generate trajectory in task space then convert to joint space using numerical IK model
    q_last = Matrix([0,0,0])
    q = q_last
    Kp = Matrix.diag(6, 6, 6)
    # Kp = Matrix.diag(40, 40, 40)
    dt = 0.01
    xc_dot = Matrix([0,0,0])
    xc_dot_last = xc_dot
    t = 0
    x = 5+3*cos(pi/2*t)
    y = 10
    z = 5+3*sin(pi/2*t)
    xr = Matrix([x,y,z])
    xr_last = xr
    tmp_x = xr
    
    for t in np.linspace(0,4,400):
        q_last = q
        xr_last = xr
        xc_dot_last = xc_dot
        x = 5+3*cos(pi*t)
        y = 10
        z = 5+3*sin(pi*t)
        xr = Matrix([x,y,z])
        
        data['x'].append(x)
        data['y'].append(y)
        data['z'].append(z)
        
        xr_dot = (xr - xr_last)/dt
        xc_dot = xr_dot + Kp*(xr - tmp_x)
        
        J = Matrix([[-1.0*(h4*cos(q[1] + q[2]) + sqrt(h2**2 + h3**2)*cos(delta_theta - q[1]))*cos(q[0]), (h4*sin(q[1] + q[2]) - 1.0*sqrt(h2**2 + h3**2)*sin(delta_theta - q[1]))*sin(q[0]), h4*sin(q[0])*sin(q[1] + q[2])], \
                    [-1.0*(h4*cos(q[1] + q[2]) + sqrt(h2**2 + h3**2)*cos(delta_theta - q[1]))*sin(q[0]), 1.0*(-h4*sin(q[1] + q[2]) + sqrt(h2**2 + h3**2)*sin(delta_theta - q[1]))*cos(q[0]), -h4*sin(q[1] + q[2])*cos(q[0])], \
                    [0, h4*cos(q[1] + q[2]) + 1.0*sqrt(h2**2 + h3**2)*cos(delta_theta - q[1]), h4*cos(q[1] + q[2])]]).evalf()

        q = q_last+dt*Kp*J.inv()*xc_dot_last

        joint_space['theta1'].append(q[0].evalf())
        joint_space['theta2'].append(q[1].evalf())
        joint_space['theta3'].append(q[2].evalf())

        DH_Table = [[0,0,h1,q[0].evalf()+pi/2],
                    [pi/2,0,0,q[1].evalf()-delta_theta],
                    [0,sqrt(h2**2+h3**2),0,pi/2+q[2].evalf()+delta_theta],
                    [pi/2,0,h4,0]]

        T_01 = DH_2_Trans(DH_Table[0]).evalf()
        T_02 = T_01*DH_2_Trans(DH_Table[1]).evalf()
        T_03 = T_02*DH_2_Trans(DH_Table[2]).evalf()
        T_04 = T_03*DH_2_Trans(DH_Table[3]).evalf()
        
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
        tmp_x = Matrix([tmp[0],tmp[1],tmp[2]])
        data['xx'].append(tmp[0])
        data['yy'].append(tmp[1])
        data['zz'].append(tmp[2])
        
    # show realtime moving 
    t = 0
    plt.ioff()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
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
            ax.plot(X, Y, Z,color='red')
            ax.plot(X1, Y1, Z1,color='blue')
            plt.draw()
            plt.pause(0.01)
    plt.show()
    
    # show position error
    np.savetxt("theta1.txt",np.array(joint_space['theta1']))
    np.savetxt("theta2.txt",np.array(joint_space['theta2']))
    np.savetxt("theta3.txt",np.array(joint_space['theta3']))

    np.savetxt("xx.txt",np.array(data['xx']))
    np.savetxt("yy.txt",np.array(data['yy']))
    np.savetxt("zz.txt",np.array(data['zz']))
    
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
    np.savetxt("pos_error.txt",np.array(error))
    plt.show()

    # fig2 = plt.figure()
    # cx = fig2.add_subplot(111)
    # cx.set_title("q in joint space")
    # cx.set_xlabel('t')
    # cx.plot(joint_space['theta1'],color='red',label='theta1')
    # cx.plot(joint_space['theta2'],color='green',label='theta2')
    # cx.plot(joint_space['theta3'],color='blue',label='theta3')

    # fig2.legend()
    # plt.show()