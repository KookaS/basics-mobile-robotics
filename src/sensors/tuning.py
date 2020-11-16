import string
import threading
import time
import numpy as np
import math
from scipy.interpolate import interp1d

from src.displacement.movement import advance, rotate, stop
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from threading import Timer

load_dotenv()

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


def obstacles_pos_from_sensor_val(sensor_val):
    """
    Returns a list containing the position of the obstacles
    w.r.t the center of the Thymio robot.

    :param sensor_val:     sensor values provided clockwise starting from the top left sensor.
    :return: numpy.array()  that contains the position of the different obstacles
    """
    dist_to_sensor = [sensor_val_to_cm_dist(x) for x in sensor_val]
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


class InitTuning:
    """
    Tune the robot to go in a straight line.
    Be careful, make it run for more than one lien because apparently there are some error initially.

    Example:
    InitTuning(thymio=th, distance=15.0, angle=180.0)
    """

    def __init__(self, thymio: Thymio, interval_check=0.1, interval_sleep=0.1, distance=15.0, angle=180.0):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.distance = distance
        self.angle = angle
        self.tune_thread = threading.Event()
        self.timer_advance = Timer(interval=interval_sleep, function=stop)
        self.timer_rotate = Timer(interval=interval_sleep, function=stop)
        self.timer_check = Timer(interval=self.interval_check, function=self.__check_handler)
        threading.Thread(target=self.__tune_thread_init).start()
        print("after starting all threads!")
        self.__check_thread_init()

    def __forward(self):
        """
        Make the robot go forward
        """
        print("Forward button pressed!")
        self.timer_check.start()
        self.tune_thread.set()  # run a thread, flag from false to true

    def __center(self):
        """
        Make the robot stop

        To remove a thread .set() & .join()
        To remove a timer .cancel() & .join()
        TODO: Still has problems if you press the button in the center
        """
        print("Center Button pressed!")
        self.timer_check.run()
        # print(threading.active_count())
        if self.timer_advance.is_alive():
            self.timer_advance.cancel()
            self.timer_advance.join()
        if self.timer_rotate.is_alive():
            self.timer_rotate.cancel()
            self.timer_advance.join()
        if self.tune_thread.is_set():
            self.tune_thread.clear()  # set the thread flag to false
            self.tune_thread.wait()  # if thread flag is false, then pause the thread

    def __check_thread_init(self):
        threading.Timer(self.interval_check, self.__check_handler).start()  # will check every time the conditions
        self.tune_thread.set()

    def __tune_thread_init(self):
        print("Waiting on thread")
        self.tune_thread.wait()
        print("thread is fired")
        self.__tune_handler()

    def __check_handler(self):
        if self.thymio["button.center"] == 1:
            self.__center()
        elif self.thymio["button.forward"] == 1:
            self.__forward()
        else:
            self.timer_check.run()

    def __tune_handler(self):
        # print(threading.active_count())
        if not self.timer_rotate.is_alive() and not self.timer_advance.is_alive():
            stop(self.thymio, verbose=True)
            self.timer_advance = advance(thymio=self.thymio, distance=self.distance, verbose=True,
                                         function=self.__rot_handler, args=[self.thymio])
        else:
            Timer(interval=self.interval_sleep, function=self.__tune_handler).start()

    def __rot_handler(self, thymio: Thymio):
        stop(self.thymio, verbose=True)
        self.timer_rotate = rotate(thymio=thymio, angle=self.angle, verbose=True)
        Timer(interval=self.interval_sleep, function=self.__tune_handler).start()
