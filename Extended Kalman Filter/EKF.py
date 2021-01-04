import sympy
import math
import numpy as np
from numpy.random import randn
from filterpy.stats import plot_covariance_ellipse
import matplotlib.pyplot as plt

# observation: z = [dist, th]
# state: x = [x, y th]
# control input: u = [v, w]
class EKF:
    def __init__(self, dt=1, wheel_base=0.5):
        self.sigma_v = 0.001
        self.sigma_w = np.radians(1)
        self.sigma_range = 0.3
        self.sigma_bearing = 0.1

        self.R = np.diag([self.sigma_range, self.sigma_bearing])

        self.x = np.array([[2, 6, .3]]).T
        self.P = np.diag([.1, .1, .1])
        
        a, x, y, v, w, theta, time = sympy.symbols('z, x, y, v, w, theta, t')
        d = v * time
        beta = (d / w) * sympy.tan(a)
        R = w / sympy.tan(a)

        self.dt = dt
        self.wheel_base = wheel_base

        # robot motion model
        self.fxu = sympy.Matrix([[x-R*sympy.sin(theta)+R*sympy.sin(theta+beta)],
                    [y+R*sympy.cos(theta)-R*sympy.cos(theta+beta)],
                    [theta+beta]])


        # linearize robot motion model wrt state space variables
        self.Fj = self.fxu.jacobian(sympy.Matrix([x, y, theta]))

        # linearize robot motion model wrt control input variables
        self.Vj = self.fxu.jacobian(sympy.Matrix([v, a]))

        self.subs = {x:0, y:0, v:0, a:0, time:dt, w:wheel_base, theta:0}
        self.x_x, self.x_y, self.v, self.a, self.theta = x, y, v, a, theta

    def predict(self, u=0):
        self.x_pred = self.move(self.x, u, self.dt)

        self.subs[self.v] = u[0, 0]
        self.subs[self.a] = u[1, 0]
        self.subs[self.theta] = self.x[2, 0]

        # use linearized motion model to compute covariance
        F = np.array(self.Fj.evalf(subs=self.subs)).astype(float)
        V = np.array(self.Vj.evalf(subs=self.subs)).astype(float)

        M = np.array([
            [self.sigma_v*u[0, 0]**2, 0], 
            [0, self.sigma_w**2]])

        # covaraince matrix
        self.p_pred = np.dot(F, np.dot(self.P, F.T)) + np.dot(V, np.dot(M, V.T))
    
    def update(self, z, landmark_pos):
        H = self.observationModel(self.x_pred, landmark_pos)
        hx = self.hxp(self.x_pred, landmark_pos)
        y = self.residual(z, hx)
        S = (self.R + np.dot(H, np.dot(self.p_pred, H.T)))
        #print(S)
        K = np.dot(self.p_pred, np.dot(H.T, np.linalg.inv(S)))
        self.x = self.x_pred + np.dot(K, y)
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.p_pred)
        return (self.x, self.P)

    def residual(self, a, b):
        """ compute residual between two measurement containing [range, bearing]. Bearing
        is normalized to [0, 360)"""
        y = a - b
        if y[1] > np.pi:
            y[1] -= 2*np.pi
        if y[1] < -np.pi:
            y[1] += 2*np.pi
        return y
    
    def move(self, x, u, dt):
        v = u[0, 0] # velocity
        alpha = u[1, 0] # steering angle
        th = x[2, 0] # state orientation
        d = v * dt # distance traveled
        r = self.wheel_base / math.tan(alpha) # radius of circle created by motion of the back wheel
        b = (d / self.wheel_base) * math.tan(alpha) # angle of rotation of back wheel from posn to posn+1

        x = x + np.array([
            [-r*math.sin(th) + r*math.sin(th + b)],
            [r*math.cos(th) - r*math.cos(th + b)],
            [b]])

        return x

    # convert state variable to the corresponding measurement (range, bearing to landmark)
    def hxp(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        d = math.sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
        Hx = np.array([[d], [math.atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        return Hx

    # compute jacobian of H matrix
    def observationModel(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        r = (px - x[0, 0])**2 + (py - x[1, 0])**2
        d = math.sqrt(r)

        H = np.array(
            [[-(px - x[0, 0]) / d, -(py - x[1, 0]) / d, 0],
            [ (py - x[1, 0]) / r,  -(px - x[0, 0]) / r, -1]])
        return H

if __name__ == "__main__":
    dt = 1
    ekf = EKF()
    sim_pos = ekf.x.copy()
    u = np.array([[1.1], [0.1]])
    sigma_range = 0.3
    sigma_bearing = 0.1
    landmarks = np.array([[5, 10], [10, 5], [15, 15], [20, 5]])
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)
    for i in range(200):
        sim_pos = ekf.move(sim_pos, u, dt/10.) # simulate robot
        plt.plot(sim_pos[0], sim_pos[1], ',', color='g')
        if i % 10 == 0:
            ekf.predict(u=u)
            plot_covariance_ellipse((ekf.x[0,0], ekf.x[1,0]), ekf.P[0:2, 0:2], std=6, facecolor='b', alpha=0.08)
                
            x, y = sim_pos[0, 0], sim_pos[1, 0]
            for lmark in landmarks:
                # recieve measurement z = (range, bearing) from sensor
                d = np.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)  
                a = math.atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
                z = np.array([[d + randn()*sigma_range], [a + randn()*sigma_bearing]])
                ekf.update(z, lmark)
                    
            plot_covariance_ellipse((ekf.x[0,0], ekf.x[1,0]), ekf.P[0:2, 0:2], std=6, facecolor='g', alpha=0.4)
    plt.axis('equal')
    plt.show()