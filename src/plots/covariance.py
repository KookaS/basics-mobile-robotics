import time
import numpy as np
import math
import matplotlib.pyplot as plt


def plot_covariance_ellipse(state_est, cov_est):
    """
    Computes the eigenvalues of the covariance of the position x and y

    :param state_est: lit of all the previous positions
    :param cov_est:  list of all the previous covariances
    :return:
    """
    Pxy = cov_est[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind]) * 10
    b = math.sqrt(eigval[smallind]) * 10
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]

    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + state_est[0, 0]).flatten()
    py = np.array(fx[1, :] + state_est[1, 0]).flatten()

    return px, py


def plot_cov(state_pred, cov_pred):
    """
    Plot the covariances

    :param state_pred: lit of all the previous positions
    :param cov_pred:  list of all the previous covariances
    :return:
    """
    state = []
    cov = []
    for i in range(len(state_pred)):
        x = np.array([[state_pred[i][0]], [state_pred[i][1]], [state_pred[i][2]]])
        state.append(x)
        x2 = np.array(cov_pred[i])
        cov.append(x2)

    plt.ion()
    fig, ax = plt.subplots()

    px, py = plot_covariance_ellipse(np.array(state[0]), np.array(cov[0] / 1000))
    line_v = ax.axvline(x=state[0][0], color="k")
    line_h = ax.axhline(y=state[0][1], color="k")
    ellips, = ax.plot(px, py, "--r", label="covariance matrix")

    l_max = len(state)
    list = [i for i in range(l_max)]
    list.insert(0, -1)

    for i in list:
        px, py = plot_covariance_ellipse(np.asarray(state[i]), np.array(cov[i] / 1000))

        line_v.set_xdata([state[i][0]])
        line_h.set_ydata([state[i][1]])

        ellips.set_xdata(px)
        ellips.set_ydata(py)
        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()

        fig.canvas.flush_events()
        plt.axis([0.8, 0, 0.725, 0])
        plt.show()
        if i == (l_max - 1):
            time.sleep(10)
