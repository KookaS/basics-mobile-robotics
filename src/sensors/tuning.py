import string
import time
import numpy as np
import math
from scipy.interpolate import interp1d
from typing import List
from src.thymio.Thymio import Thymio
from src.sensors.state import RepeatedTimer

# Sensor measurements
sensor_distances = np.array([i for i in range(0, 21)])
sensor_measurements = np.array([5120, 4996, 4964, 4935, 4554, 4018, 3624, 3292, 2987,
                                2800, 2580, 2307, 2039, 1575, 1127, 833, 512, 358, 157, 52, 0])
# sensor_distances = np.array([0, 30])
# sensor_measurements = np.array([5120, 0])

# Thymio outline
center_offset = np.array([5.5, 5.5])
thymio_coords = np.array([[0, 0], [11, 0], [11, 8.5], [10.2, 9.3],
                          [8, 10.4], [5.5, 11], [3.1, 10.5],
                          [0.9, 9.4], [0, 8.5], [0, 0]]) - center_offset

# Sensor positions and orientations
sensor_pos_from_center = np.array(
    [[0.9, 9.4], [3.1, 10.5], [5.5, 11.0], [8.0, 10.4], [10.2, 9.3], [8.5, 0], [2.5, 0]]) - center_offset
sensor_angles = np.array([120, 105, 90, 75, 60, -90, -90]) * math.pi / 180


def sensor_val_to_cm_dist(val: int) -> int:
    """
    Interpolation from sensor values to distances in cm

    :param val: the sensor value that you want to convert to a distance
    :return:    corresponding distance in cm
    """
    if val == 0:
        return np.inf

    f = interp1d(sensor_measurements, sensor_distances)
    return np.asscalar(f(val))


def obstacles_pos_from_sensor_vals(sensor_vals):
    """
    Returns a list containing the position of the obstacles
    w.r.t the center of the Thymio robot.

    :param sensor_vals:     sensor values provided clockwise starting from the top left sensor.
    :return: numpy.array()  that contains the position of the different obstacles
    """
    dist_to_sensor = [sensor_val_to_cm_dist(x) for x in sensor_vals]
    dx_from_sensor = [d * math.cos(alpha) for (d, alpha) in zip(dist_to_sensor, sensor_angles)]
    dy_from_sensor = [d * math.sin(alpha) for (d, alpha) in zip(dist_to_sensor, sensor_angles)]
    obstacles_pos = [[x[0] + dx, x[1] + dy] for (x, dx, dy) in
                     zip(sensor_pos_from_center, dx_from_sensor, dy_from_sensor)]
    return np.array(obstacles_pos)


def print_sensor_values(thymio: Thymio, sensor_id: string, print_duration=3, delta_time=0.5):
    """
    While the end time has not been reached, print the sensor values every delta_time seconds

    :param thymio:          the class of Thymio to refer to the robot
    :param sensor_id:       sensor values provided clockwise starting from the top left sensor.
    :param print_duration:  sensor values provided clockwise starting from the top left sensor.
    :param delta_time:      sensor values provided clockwise starting from the top left sensor.
    :return: numpy.array()  that contains the position of the different obstacles

    Example:

        print_sensor_values('prox.ground.reflected')

        print_sensor_values('prox.horizontal', 3)
    """
    t_end = time.time() + print_duration

    while time.time() < t_end:
        time.sleep(delta_time)
        print(thymio[sensor_id])


class InitTuning(object):
    """
    Tune the robot to go in a straight line
    """

    def __init__(self, thymio: Thymio, ts: float = 0.1):
        self.CONST_STOP = 0
        self.CONST_FORWARD_TUNE = 1
        self.CONST_TURN_TUNE = 2
        self.CONST_ONE_TURN_TUNE = 8873
        self.CONST_SAMPLING_TUNE = 5
        self.state = 0
        self.time = 0
        self.thymio = thymio
        self.thymio.set_var("motor.left.target", 0)
        self.thymio.set_var("motor.right.target", 0)
        self.ts = ts

    def __forward(self):
        """
        Make the robot go forward
        """
        self.state = self.CONST_FORWARD_TUNE
        self.time = 0
        self.thymio.set_var("motor.left.target", 100)
        self.thymio.set_var("motor.right.target", 100)

    def __center(self):
        """
        Make the robot stop
        """
        self.state = self.CONST_FORWARD_TUNE
        self.time = 0
        self.thymio.set_var("motor.left.target", 0)
        self.thymio.set_var("motor.right.target", 0)

    def __tune_wheels(self):
        """
        Tune the wheels to have a straight motion
        """

        # manual mode
        if self.thymio["button.center"] == 1:
            print("center")
            self.__center()
            return
        elif self.thymio["button.forward"] == 1:
            print("forward")
            self.__forward()
            return

        self.time += 1
        print(self.time)
        if self.state == self.CONST_FORWARD_TUNE:
            if self.time > (self.CONST_ONE_TURN_TUNE / self.CONST_SAMPLING_TUNE / 2):
                "turn!"
                self.time = 0
                self.state = self.CONST_TURN_TUNE
                self.thymio.set_var("motor.left.target", 100)
                self.thymio.set_var("motor.right.target", -100)
                return

        elif self.state == self.CONST_TURN_TUNE:
            if self.time > (self.CONST_ONE_TURN_TUNE / self.CONST_SAMPLING_TUNE / 2):
                "straight!"
                self.time = 0
                self.state = self.CONST_FORWARD_TUNE
                self.thymio.set_var("motor.left.target", 100)
                self.thymio.set_var("motor.right.target", 99)
                return

    def tune(self):
        """
        Launches the tuning process
        """
        self.__forward()
        print("before timer")
        self.ts = 1
        rt = RepeatedTimer(self.ts, print("test"))
        print("after timer")
