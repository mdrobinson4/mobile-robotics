import time
import math
import numpy as np
import matplotlib.pyplot as plt

# control vector (u = [v, w]) [gyro]
# observationn vector (z = [x, y]) [gps]
# state vector (x = [x, y, theta])
class KalmanFilter:
    def __init__(self, dt=0.1):
        n = 3 # state vector
        m = 2 # observation vector
        k = 2 # m control vector

        # state estimation (robot model)
        self.X = np.zeros((n, 1))

        # state covariance matrix 
        # represents an estimate of the accuracy of the state estimate (X)
        self.P = 0.1 * np.eye(n)

        # state transition model
        # describes how the state of the system changes with time when no control is executed
        self.F = np.eye(n)

        # state model noise covariance matrix
        # represents how much the actual motioin deviates from your assumed state space model
        self.Q = np.eye(n)

        # measurement noise covariance
        self.R = np.eye(k)

        # measurement matrix
        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

    def filter(self, u, z, dt):
        (x_pred, p_pred) = self.predict(u, dt)
        (self.X, self.P) = self.update(z, x_pred, p_pred)
        return (self.X, self.P)
    
    def predict(self, u, dt):
        x_pred, p_pred = self.motionModel(u, dt)
        return (x_pred, p_pred)

    def update(self, z, x_pred, p_pred):
        V = z - np.dot(self.H, x_pred) # the innovation or measurement residual
        S = np.dot(self.H, np.dot(p_pred, self.H.T)) + self.R # the measurement prediction covarance 
        K = np.dot(p_pred, np.dot(self.H.T, np.linalg.inv(S))) # the kalman gain matrix
        X = x_pred + np.dot(K, V)
        P = p_pred - np.dot(K, np.dot(S, K.T)) 
        return (X, P)

    def motionModel(self, u, dt):
        theta = self.X[2, 0]
        B = np.array([
            [dt * math.cos(theta), 0],
            [dt * math.sin(theta), 0],
            [0, dt]])
        x_pred = np.dot(self.F, self.X) + np.dot(B, u)
        p_pred = np.dot(F, np.dot(self.P, F.T)) + self.Q
        return x_pred, p_pred
    
    def observationModel(self):
        z = np.dot(self.H, self.X)
        return z

def plotCovariance(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


if __name__ == "__main__":
    n = 3
    m = 2
    k = 2

    Q_sim = np.diag([0.5, 0.5])**2 # state space model error
    R_sim = np.diag([1.0, np.deg2rad(30.0)])**2 # 

    u = np.zeros((m, 1))
    z = np.zeros((k, 1))

    x_true = X_TRUE = np.zeros((n, 1))
    x_est = X_EST = np.zeros((n, 1))

    v = 10.5
    w = 70.4

    kf = KalmanFilter()
    curr_time = time.time()
    total_time = 0
    end_time = 10000
    while total_time < end_time:
        dt = 1 #time.time() - curr_time
        F = np.eye(n)
        theta = x_true[2]
        B = np.array([
            [dt * math.cos(theta), 0],
            [dt * math.sin(theta), 0],
            [0, dt]])
        x_true = np.dot(F, x_true) + np.dot(B, u)

        z[0] = x_true[0, 0] + np.random.randn() * Q_sim[0, 0]
        z[1] = x_true[1, 0] + np.random.randn() * Q_sim[1, 1]

        u[0] = v + np.random.randn() * R_sim[0, 0]
        u[1] = w + np.random.randn() * R_sim[1, 1]

        x_est, p_est = kf.filter(u, z, dt)
        
        X_TRUE = np.hstack((X_TRUE, x_true))
        X_EST = np.hstack((X_EST, x_est))
        

        plt.cla()
        plt.plot(X_TRUE[0, :].flatten(), X_TRUE[1, :].flatten(), "-b")
        plt.plot(X_EST[0, :].flatten(), X_EST[1, :].flatten(), "-r")
        plotCovariance(x_est, p_est)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

        total_time += dt
        curr_time = time.time()
        #print(total_time)
    plt.show()