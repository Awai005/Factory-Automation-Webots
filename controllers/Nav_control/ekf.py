import numpy as np

class EKF:
    def __init__(self, dt):
        self.dt = dt
        # x = [x, y, theta, v]
        self.x = np.zeros(4)
        self.P = np.eye(4) * 0.01
        self.Q = np.diag([0.002, 0.002, np.deg2rad(1)**2, 0.01])
        self.R_gps = np.diag([0.09, 0.09])      # (0.3 m)^2

    #--------------- prediction ---------------
    def predict(self, omega):
        x, y, theta, v = self.x
        dt = self.dt

        # motion model
        self.x = np.array([
            x + v*np.cos(theta)*dt,
            y + v*np.sin(theta)*dt,
            theta + omega*dt,
            v
        ])

        F = np.eye(4)
        F[0,2] = -v*np.sin(theta)*dt
        F[0,3] =  np.cos(theta)*dt
        F[1,2] =  v*np.cos(theta)*dt
        F[1,3] =  np.sin(theta)*dt

        self.P = F @ self.P @ F.T + self.Q

    #--------------- update (GPS) -------------
    def update_gps(self, z):
        H = np.array([[1,0,0,0],
                      [0,1,0,0]])
        y  = z - H @ self.x
        S  = H @ self.P @ H.T + self.R_gps
        K  = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P  = (np.eye(4) - K @ H) @ self.P
