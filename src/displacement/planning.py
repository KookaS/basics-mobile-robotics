import numpy as np


# this code do a sequence of  displacement corresponding to the entire global path planning
# i.e to go from the start to the goal

def update_path(path, x, y, theta, case_size):
    if len(path[0]) and len(path[1]):
        print("x,y", x, y)
        target_x = path[0][0] * case_size
        target_y = path[1][0] * case_size
        delta_x_cm = target_x - x
        delta_y_cm = target_y - y
        print("target_x [cm] ", target_x)
        print("target_y [cm] ", target_y)
        # print("x: ", x)
        # print("y: ", y)
        # print("delta_x: ", delta_x)
        # print("delta_y: ", delta_y)
        # print("sign: ", np.sign(delta_x))
        # print("sign: ", np.sign(delta_x))

        # print("cases en x: ", int(np.sign(delta_x)))
        # print("cases en y: ", int(np.sign(delta_y)))
        delta_r = np.sqrt(delta_x_cm ** 2 + delta_y_cm ** 2)
        # print("x, y: ", x, y)

        # Relative rotation to target
        target_theta_rad = np.arctan2(delta_y_cm, delta_x_cm)
        # print("target_theta_rad: ", target_theta_rad)
        target_theta_deg = np.rad2deg(target_theta_rad) + 90
        # target_theta_deg = np.abs(target_theta_deg) * np.sign(delta_x_cm)
        print("target_theta_deg: ", target_theta_deg)

        delta_theta = target_theta_deg - theta
        delta_theta = (delta_theta + 180.0) % 360.0 - 180.0
        # print("delta_theta: ", delta_theta)
        # delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi
        # print("theta: ", theta)
        # print("target_theta_deg: ", target_theta_deg)
        return delta_r, delta_theta
