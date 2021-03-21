import numpy as np
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        Hj = Jacobian(self.x)
        # 2. Calculate S = H_j * P' * H_j^T + R
        S = np.dot(np.dot(Hj, self.P ), Hj.T) + self.R
        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        K = np.dot(np.dot(self.P, Hj.T), np.linalg.inv(S))
        # 4. Estimate y = z - h(x')
        sqrt_x = sqrt(self.x[0]*self.x[0] + self.x[1]*self.x[1])
        tan_1 = atan2(self.x[1],self.x[0])
        sqrt_x_2 = (self.x[0]*self.x[2]+ self.x[1]*self.x[3])/sqrt_x
        
        hx = np.array([sqrt_x,tan_1,sqrt_x_2])

        y = z - hx
        # 5. Normalize phi so that it is between -PI and +PI 
        # y[1] = y[1] / (math.pi*2)
        if y[1] > 3.14 or y[1] < -3.14:
            y[1] = y[1] % 3.14
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P
        self.x = self.x + np.dot(K,y)
        IK = np.eye(4) - np.dot(K,Hj)
        self.P = np.dot(IK,self.P)
        

        
        
        
        
        
