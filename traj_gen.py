import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from sympy import *
import IPython.display as disp
na = np.array

def t_vec(t, c=4, derivation=False):
    return na([0. if not cc else cc * t ** (cc - 1) for cc in range(c)]) if derivation else na([t ** cc for cc in range(c)])

def plot(time, frequency, function):
    t = np.linspace(time[0], time[1], num=frequency * (time[1] - time[0]))
    y = np.zeros((t.shape[0], 3))
    for i in range(100):
        y[i] = function(t[i]).flatten()[:3]
    return y

class Trajectory:
    # start and endpoint have to be in position first, then acceleration
    def __init__(self, start, end, time, ftype="cubic", *args):
        self.start = start
        self.end = end
        self.time = time
        self.type = ftype

        if ftype == "cubic":
            self.sol = self.cubic()
            self.function = self.cubic_fun

        elif ftype == "quintic":
            self.im = args[0]
            self.sol = self.quintic()
            self.function = self.quintic_fun

        elif ftype == "parabolic":
            self.tb = args[0]
            self.sol = self.parabolic()
            self.function = self.parabolic_fun

        else:
            print(f"type {ftype} is not valid!    destructing...")
            self.__del__()

    def cubic(self):
        ts, te = self.time
        vals = na([self.start[0], self.end[0], self.start[1], self.end[1]])
        M = na([t_vec(ts, 4), t_vec(te, 4), t_vec(ts, 4, True), t_vec(te, 4, True)])
        return np.dot(np.linalg.inv(M), vals)

    def quintic(self):
        ts, ti, te = self.time
        vals = na([self.start[0], self.im[0], self.end[0], self.start[1], self.im[1], self.end[1]])
        M = na([t_vec(ts, 6), t_vec(ti, 6), t_vec(te, 6), t_vec(ts, 6, 1), t_vec(ti, 6, 1), t_vec(te, 6, 1)])
        return np.dot(np.linalg.inv(M), vals)

    def cubic_fun(self, t):
        M = na([[1., t, t ** 2, t ** 3], [0, 1., 2 * t, 3 * t ** 2]])
        return np.dot(M, self.sol)

    def quintic_fun(self, t):
        M = na([t_vec(t, 6), t_vec(t, 6, True)])
        return np.dot(M, self.sol)

    def parabolic(self):
        return (self.start[0] - self.end[0]) / (self.tb ** 2 - self.tb * self.time[-1])

    def parabolic_fun(self, t):
        acc, tb, ps, pe = self.sol, self.tb, self.start[0], self.end[0]
        if t < self.tb:
            return acc * (t ** 2) / 2. + ps
        elif t > (self.time[-1] - self.tb):
            return - acc * ((t - self.time[-1]) ** 2) / 2. + pe
        else:
            return acc * tb * (t - tb) + acc * (tb ** 2) / 2. + ps


fig = plt.figure()
ax = fig.add_subplot(111)
# plt.xlim(-30, 30)
# plt.ylim(-30, 30)
time = [0,1]
frequency = 100
t = np.linspace(time[0], time[1], num=frequency * (time[1] - time[0]))

start = na([[0, 1, 0], [0, 0, 0]])
end = na([[-2, 3, 4], [0, 0, 0]])
tra_c = Trajectory(start, end, [0, 1], "cubic")
print("plotting the cubic trajectory")
y = plot([0, 1], 100, tra_c.function)
ax.plot(t,y,label="cubic")

im = na([[-1, 2, 2], [3, 3, 3]])
print("plotting the quintic trajectory")
tra_q = Trajectory(start, end, [0, 0.5, 1], "quintic", im)
y = plot([0, 1], 100, tra_q.function)
ax.plot(t,y,label="quintic")

print("plotting the parabolic trajectory")
tra_p = Trajectory(start, end, [0, 1], "parabolic", 0.15)
y = plot([0, 1], 100, tra_p.function)
ax.plot(t,y,label="parabolic")

plt.legend()
plt.show()