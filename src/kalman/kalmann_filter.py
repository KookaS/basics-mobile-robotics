import numpy as np
import math

Ts = 0.1
# tune model params
qx = 0.2
qy = 0.2
qt = 0.4
k_delta_sr = 0.8
k_delta_sl = 0.8
state_est = [np.array([[0], [0], [0]])]
cov_est = [0.01 * np.ones([3, 3])]
# k_delta_sr = 0.01
# k_delta_sl = 0.01

b = 0.095
H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
stripe_width = 50
Q = np.array([[qx, 0, 0], [0, qy, 0], [0, 0, qt]])
R = np.array([[k_delta_sr, 0], [0, k_delta_sl]])


def jacobianF_x(theta, delta_s, delta_theta):
    Fx = np.array(
        [[1, 0, -delta_s * np.sin(theta + delta_theta / 2)], [0, 1, delta_s * np.cos(theta + delta_theta / 2)],
         [0, 0, 1]])
    return Fx


def jacobianF_u(theta, delta_s, delta_theta):
    Fu = np.array([[1 / 2 * np.cos(theta + delta_theta / 2) - delta_s / (2 * b) * np.sin(theta + delta_theta / 2),
                    1 / 2 * np.cos(theta + delta_theta / 2) + delta_s / (2 * b) * np.sin(theta + delta_theta / 2)],
                   [1 / 2 * np.sin(theta + delta_theta / 2) + delta_s / (2 * b) * np.cos(theta + delta_theta / 2),
                    1 / 2 * np.sin(theta + delta_theta / 2) - delta_s / (2 * b) * np.cos(theta + delta_theta / 2)],
                   [1 / b, -1 / b]])
    return Fu


def prediction_only(state_est_prev, cov_est_prev, delta_sr, delta_sl):
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
    delta_theta = (delta_sr - delta_sl) / b

    Fx = jacobianF_x(theta, delta_s, delta_theta)
    Fu = jacobianF_u(theta, delta_s, delta_theta)

    # Prediction step
    # estimated mean of the state
    state_est_a_priori = state_est_prev + np.array(
        [[delta_s * np.cos(theta + delta_theta / 2)], [delta_s * np.sin(theta + delta_theta / 2)], [delta_theta]])

    # Estimated covariance of the state
    cov_est_a_priori = np.dot(Fx, np.dot(cov_est_prev, Fx.T)) + np.dot(Fu, np.dot(R, Fu.T))

    return state_est_a_priori, cov_est_a_priori


def plot_covariance_ellipse(state_est, cov_est):
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
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + state_est[0, 0]).flatten()
    py = np.array(fx[1, :] + state_est[1, 0]).flatten()

    return px, py


def kalman_filter(camera, state_est_prev, cov_est_prev, delta_sr, delta_sl):
    """
    Estimates the current state using input sensor data and the previous state

    param camera: array representing the measurement (x,y,theta) (coming from the vision sensor)
    param delta_sr: travelled distance for the right wheel (in meters)
    param delta_sl: travelled distance for the left wheel (in meters)
    param state_est_prev: previous state a posteriori estimation
    param cov_est_prev: previous state a posteriori covariance

    return state_est: new a posteriori state estimation
    return cov_est: new a posteriori state covariance
    """
    new_pos = np.array([[state_est_prev[0], state_est_prev[1], state_est_prev[2]]])
    new_cam = np.array([[camera[0], camera[1], camera[2]]]).T
    print(new_pos)

    if camera[0] != -1 and camera[1] != -1:
        condition = True
    else:
        condition = False

    theta = state_est_prev[2]
    delta_s = (delta_sr + delta_sl) / 2
    delta_theta = (delta_sr - delta_sl) / b

    Fx = jacobianF_x(theta, delta_s, delta_theta)
    Fu = jacobianF_u(theta, delta_s, delta_theta)

    # Prediction step
    # estimated mean of the state
    temp = np.array(
        [[delta_s * np.cos(theta + delta_theta / 2), delta_s * np.sin(theta + delta_theta / 2), delta_theta]])
    print("temp ", temp)
    state_est_a_priori = (state_est_prev + temp).T
    print("state_est_a_priori ", state_est_a_priori)

    # Estimated covariance of the state
    cov_est_a_priori = np.dot(Fx, np.dot(cov_est_prev, Fx.T)) + np.dot(Fu, np.dot(R, Fu.T))

    # If we have camera's measurements
    if condition:
        # Update step
        # innovation / measurement residual
        i = new_cam - state_est_a_priori
        print(i)

        # Kalman gain (tells how much the predictions should be corrected based on the measurements)
        K = np.dot(cov_est_a_priori, np.linalg.inv(cov_est_a_priori + Q))

        new = np.dot(K, i)
        print(new)
        # a posteriori estimate
        state_est = state_est_a_priori + new
        print(state_est)
        cov_est = cov_est_a_priori - np.dot(K, cov_est_a_priori)

    else:
        # TODO change stuff here
        state_est = state_est_a_priori
        print(state_est)
        cov_est = cov_est_a_priori

    return [state_est[0, 0], state_est[1, 0], state_est[2, 0]], cov_est
