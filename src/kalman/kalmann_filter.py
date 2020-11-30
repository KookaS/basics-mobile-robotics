import numpy as np
import math


class Kalman:

    def __init__(self, qx, qy, qt, k_delta_sr, k_delta_sl):
        self.Ts = 0.1
        self.state_est = [np.array([[0], [0], [0]])]
        self.cov_est = [0.01 * np.ones([3, 3])]
        self.b = 0.095
        self.H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.stripe_width = 50
        self.tune_values(qx, qy, qt, k_delta_sr, k_delta_sl)

    def tune_values(self, qx, qy, qt, k_delta_sr, k_delta_sl):
        self.qx = qx
        self.qy = qy
        self.qt = qt
        self.k_delta_sr = k_delta_sr
        self.k_delta_sl = k_delta_sl
        self.Q = np.array([[self.qx, 0, 0], [0, self.qy, 0], [0, 0, self.qt]])
        self.R = np.array([[self.k_delta_sr, 0], [0, self.k_delta_sl]])

    def jacobianF_x(self, theta, delta_s, delta_theta):
        return np.array(
            [[1, 0, -delta_s * np.sin(theta + delta_theta / 2)], [0, 1, delta_s * np.cos(theta + delta_theta / 2)],
             [0, 0, 1]])

    def jacobianF_u(self, theta, delta_s, delta_theta):
        return np.array(
            [[1 / 2 * np.cos(theta + delta_theta / 2) - delta_s / (2 * self.b) * np.sin(theta + delta_theta / 2),
              1 / 2 * np.cos(theta + delta_theta / 2) + delta_s / (2 * self.b) * np.sin(theta + delta_theta / 2)],
             [1 / 2 * np.sin(theta + delta_theta / 2) + delta_s / (2 * self.b) * np.cos(theta + delta_theta / 2),
              1 / 2 * np.sin(theta + delta_theta / 2) - delta_s / (2 * self.b) * np.cos(theta + delta_theta / 2)],
             [1 / self.b, -1 / self.b]])

    def prediction_only(self, state_est_prev, cov_est_prev, delta_sr, delta_sl):
        """
        Estimates the current state using only the previous state

        param delta_sr: travelled distance for the right wheel (in meters)
        param delta_sl: travelled distance for the left wheel (in meters)
        param state_est_prev: previous state a posteriori estimation
        param cov_est_prev: previous state a posteriori covariance

        return state_est_a_priori: new a priori state estimation
        return cov_est: new a priori state covariance
        """

        theta = state_est_prev[2, 0]
        delta_s = (delta_sr + delta_sl) / 2
        delta_theta = (delta_sr - delta_sl) / self.b

        Fx = self.jacobianF_x(theta, delta_s, delta_theta)
        Fu = self.jacobianF_u(theta, delta_s, delta_theta)

        # Prediction step
        # estimated mean of the state
        state_est_a_priori = state_est_prev + np.array(
            [[delta_s * np.cos(theta + delta_theta / 2)], [delta_s * np.sin(theta + delta_theta / 2)], [delta_theta]])

        # Estimated covariance of the state
        cov_est_a_priori = np.dot(Fx, np.dot(cov_est_prev, Fx.T)) + np.dot(Fu, np.dot(self.R, Fu.T))
        return state_est_a_priori, cov_est_a_priori

    def plot_covariance_ellipse(self, state_est, cov_est):
        Pxy = cov_est[0:2, 0:2]
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
        self.R = np.array([[math.cos(angle), math.sin(angle)],
                           [-math.sin(angle), math.cos(angle)]])
        fx = self.R.dot(np.array([[x, y]]))
        px = np.array(fx[0, :] + state_est[0, 0]).flatten()
        py = np.array(fx[1, :] + state_est[1, 0]).flatten()

        return px, py

    def kalman_filter(self, z, state_est_prev, cov_est_prev, delta_sr, delta_sl, measurement):
        """
        Estimates the current state using input sensor data and the previous state
        Everything is in meter and seconds here!

        param z: array representing the measurement (x,y,theta) (coming from the vision sensor)
        param delta_sr: travelled distance for the right wheel (in meters)
        param delta_sl: travelled distance for the left wheel (in meters)
        param state_est_prev: previous state a posteriori estimation
        param cov_est_prev: previous state a posteriori covariance

        return state_est: new a posteriori state estimation
        return cov_est: new a posteriori state covariance
        """
        if z[0] == -1 or z[1] == -1:  # if no camera, just odometry
            measurement = False

        theta = state_est_prev[2]
        delta_s = (delta_sr + delta_sl) / 2
        delta_theta = (delta_sr - delta_sl) / self.b

        Fx = self.jacobianF_x(theta, delta_s, delta_theta)
        Fu = self.jacobianF_u(theta, delta_s, delta_theta)

        # print("z: ", z)
        # print("state_est_prev: ", state_est_prev)
        # print("cov_est_prev: ", cov_est_prev)
        # print("delta_sr: ", delta_sr)
        # print("delta_sl: ", delta_sl)
        # Prediction step
        # estimated mean of the state
        z = np.array([[z[0]], [z[1]], [z[2]]])
        state_est_prev = np.array([[state_est_prev[0]], [state_est_prev[1]], [state_est_prev[2]]])
        # print("z: ", z)
        # print("state_est_prev: ", state_est_prev)

        state_est_a_priori = state_est_prev + np.array(
            [[delta_s * np.cos(theta + delta_theta / 2)], [delta_s * np.sin(theta + delta_theta / 2)], [delta_theta]])

        # Estimated covariance of the state
        cov_est_a_priori = np.dot(Fx, np.dot(cov_est_prev, Fx.T)) + np.dot(Fu, np.dot(self.R, Fu.T))

        # If we have camera's measurements
        if measurement:  # odometry et measurements
            # Update step
            # innovation / measurement residual
            i = z - state_est_a_priori

            # Kalman gain (tells how much the predictions should be corrected based on the measurements)
            K = np.dot(cov_est_a_priori, np.linalg.inv(cov_est_a_priori + self.Q))

            # a posteriori estimate
            state_est = state_est_a_priori + np.dot(K, i)
            cov_est = cov_est_a_priori - np.dot(K, cov_est_a_priori)

        else:  # odometry
            state_est = state_est_a_priori
            cov_est = cov_est_a_priori

        # print("cov_est: ", cov_est)
        # print("state_est: ", state_est)
        return state_est.flatten().tolist(), cov_est
