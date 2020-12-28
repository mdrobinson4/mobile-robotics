import numpy as np

class KalmanFilter:
    def __init__(self, Q=None, R=None, A=None, B=None, H=None):
        if Q == None:
            self.defaultParams()
        else:
            self.Q = Q
            self.R = R
            self.A = A
            self.B = B
            self.H = H

        # kalman gain
        self.K = 0

    def defaultParams(self):
        # process noise covariance matrix
        self.Q = np.zeros((6, 6))
        self.Q[0:3,0:3] = (1/4) * (np.eye(3)*math.pow(dt, 4))
        self.Q[4:, 4:] = np.eye(3) * math.pow(dt, 2)

        # measurement noise covariance
        self.R = None

        # state transition matrix
        self.A = np.zeros((6, 6))
        self.A[0:3, 0:3] = np.eye(3)
        self.A[4:, 4:] = np.eye(3)
        self.A[0:3, 4:] = np.eye(3) * dt

        # control-input matrix
        self.B = np.zeros((6, 3))
        self.B[0:3, 0:] = (1/2) * np.eye(3) * math.pow(dt, 2)
        self.B[4:, 0:] = np.eye(3) * dt

        # measurement matrix
        self.H = np.eye(6)

    def filter(self, x, u, z):
        (x_pred, p_pred) = self.predict(x, u)
        (self.X, self.P) = self.update(z, x_pred, p_pred)
        return (self.X, self.P)
    
    def predict(self, x, u):
        x_pred = np.dot(self.A, self.X) + np.dot(self.B, u)
        p_pred = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q
        return (x_pred, p_pred)
    
    def update(self, z, x_pred, p_pred):
        y = z - np.dot(self.H, x_pred)
        a = np.dot(p_pred, self.H.T)
        b = np.inv(self.R + np.dot(self.H, np.dot(p_pred, self.H.T)))
        k = np.dot(a, b)
        X = x_pred + np.dot(k, y)
        P = np.dot((np.eye(z.shape[0]) - np.dot(k, self.H)), p_pred)
        return (X, P)

if __name__ == "__main__":
    N = 20
    dt = 0.1
    shape = (6, 6)
    kf = KalmanFilter(dt)
    self.X = 
    self.P = 
