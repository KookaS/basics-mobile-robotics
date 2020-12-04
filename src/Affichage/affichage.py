import os
import sys
import time
import serial
import numpy as np
import math
import matplotlib.pyplot as plt


Ts = 0.1

# tune model params
qx = 0.2  # size of the object on the robot in pixels
qy = 0.2  # small qx, qy, qt
qt = 0.4
k_delta_sr = 0.8 # std of the inputs 0.1 * order we have
k_delta_sl = 0.8
#k_delta_sr = 0.01
#k_delta_sl = 0.01

b = 0.095
H = np.array([[1, 0, 0], [0, 1, 0],[0, 0, 1]])
stripe_width = 50
Q = np.array([[qx, 0, 0], [0, qy, 0], [0, 0, qt]])
R = np.array([[k_delta_sr, 0], [0, k_delta_sl]])
def jacobianF_x(theta, delta_s, delta_theta):

    Fx = np.array([[1, 0, -delta_s*np.sin(theta + delta_theta/2)], [0, 1, delta_s*np.cos(theta + delta_theta/2)], [0, 0, 1]])
    return Fx

def jacobianF_u(theta, delta_s, delta_theta):

    Fu = np.array([[1/2*np.cos(theta + delta_theta/2) - delta_s/(2*b)*np.sin(theta + delta_theta/2), 1/2*np.cos(theta + delta_theta/2) + delta_s/(2*b)*np.sin(theta + delta_theta/2)], [1/2*np.sin(theta + delta_theta/2) + delta_s/(2*b)*np.cos(theta + delta_theta/2), 1/2*np.sin(theta + delta_theta/2) - delta_s/(2*b)*np.cos(theta + delta_theta/2)], [1/b , -1/b]])
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

    theta = state_est_prev[2,0]
    delta_s = (delta_sr + delta_sl)/2
    delta_theta = (delta_sr - delta_sl)/b

    Fx = jacobianF_x(theta, delta_s, delta_theta)
    Fu = jacobianF_u(theta, delta_s, delta_theta)

    ## Prediciton step
    # estimated mean of the state
    state_est_a_priori = state_est_prev + np.array([[delta_s*np.cos(theta + delta_theta/2)],[delta_s*np.sin(theta + delta_theta/2)],[delta_theta]])


    # Estimated covariance of the state
    cov_est_a_priori = np.dot(Fx, np.dot(cov_est_prev, Fx.T)) + np.dot(Fu, np.dot(R, Fu.T))

    return state_est_a_priori, cov_est_a_priori
state_pred = [np.array([[0], [0], [0]])]
cov_pred = [0.01 * np.ones([3,3])]


#delta_sl = [0.8, 0.8, 0.8, 0.8, 0.9, 0.9, 0.9, 0.009, 0.5, 0.5, 0.5, 0.2, 0.2, 0.2, 0.02, 0.01, 0.01, 0.3, 0.3, 0.3]
#delta_sr = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0, 0, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.3, 0.3, 0.3]

delta_sl = [0.8, 0.8, 0.8, 0.8, 0.83, 0.83, 0.83, 0.83]
delta_sr = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]

for i in range(len(delta_sr)):

    new_state_pred, new_cov_pred = prediction_only(state_pred[-1], cov_pred[-1], delta_sr[i], delta_sl[i])

    state_pred.append(new_state_pred)
    cov_pred.append(new_cov_pred)
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




plt.ion()
fig, ax = plt.subplots()

px, py = plot_covariance_ellipse(state_pred[0], cov_pred[0]/1000)
line_v = ax.axvline(x = state_pred[0][0], color = "k")
line_h = ax.axhline(y = state_pred[0][1], color = "k")
ellips, = ax.plot(px, py, "--r", label="covariance matrix")

#ellips.set_data(px, py)
#plt.style.use('ggplot')

for i in range(len(delta_sr)):
    plt.close()
    print(i)
    px, py = plot_covariance_ellipse(state_pred[i], cov_pred[i]/1000)

    line_v.set_xdata(state_pred[i][0])
    line_h.set_ydata(state_pred[i][1])

    ellips.set_xdata(px)
    ellips.set_ydata(py)
    ax.relim()
    ax.autoscale_view()


    fig.canvas.draw()

    fig.canvas.flush_events()

    plt.pause(1)
    time.sleep(1)

    #plt.pause(1)

    #ax.cla()

