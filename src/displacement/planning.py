import time

import numpy as np

from src.displacement.movement import rotate, advance
from src.thymio.Thymio import Thymio


# this code do a sequence of  displacement corresponding to the entire global path planning
# i.e to go from the start to the goal

def update_path(path, x, y, theta):
    CONST_DISP = 2.5  # distance in cm between two squares
    if len(path[0]) and len(path[1]):
        target_x = path[0][0]
        target_y = path[1][0]
        delta_x = target_x - x
        delta_y = target_y - y
        # print("sign: ", np.sign(delta_x))
        # print("sign: ", np.sign(delta_x))

        # print("cases en x: ", int(np.sign(delta_x)))
        # print("cases en y: ", int(np.sign(delta_y)))
        delta_x = target_x - x
        delta_y = target_y - y
        delta_x_cm = delta_x * CONST_DISP
        delta_y_cm = delta_y * CONST_DISP
        delta_r = np.sqrt(delta_x_cm ** 2 + delta_y_cm ** 2)
        # print("x, y: ", x, y)

        # Relative rotation to target
        target_theta_rad = np.arctan2(delta_y_cm, delta_x_cm)
        target_theta_deg = np.rad2deg(target_theta_rad)
        target_theta_deg = np.abs(target_theta_deg) * np.sign(delta_x)
        delta_theta = target_theta_deg - theta
        delta_theta = (delta_theta + 180.0) % 360.0 - 180.0
        # delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi
        # print("theta: ", theta)
        # print("target_theta_deg: ", target_theta_deg)
        return delta_r, delta_theta
