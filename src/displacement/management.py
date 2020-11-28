import threading
import time

import numpy as np
from enum import Enum

from src.displacement.movement import stop
from src.displacement.planning import update_path
from src.kalman.kalmann_filter import kalman_filter
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.occupancy import create_grid, display_occupancy
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.vision.camera import record_project
import matplotlib.pyplot as plt


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    GLOBAL = 0
    LOCAL = 1
    STOP = 2
    KALMAN = 3
    CAMERA = 4


class EventHandler:
    """
    Example:
            EventHandler(thymio=th, interval_check=5)
    """

    def __init__(self, thymio: Thymio, interval_check=0.2, interval_sleep=0.01, obstacle_threshold=2000,
                 stop_threshold=3500):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.sensor_handler = SensorHandler(self.thymio)
        self.covariance = [0.01 * np.ones([3, 3])]
        self.thymio_speed_to_mms = 0.4347
        self.kalman_time = time.time()
        self.delta_sr = 0
        self.delta_sl = 0
        self.running = []
        for _ in EventEnum:
            self.running.append(False)
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        self.final_occupancy_grid = create_grid()
        self.camera_measure = record_project()
        self.position = self.camera_measure
        print("init position ", self.position)

        threading.Timer(self.interval_check, self.__check_handler).start()

        self.state = EventEnum.KALMAN.value
        self.running[EventEnum.KALMAN.value] = True
        threading.Timer(self.interval_sleep, self.__kalman_handler).start()
        """
        self.state = EventEnum.CAMERA.value
        self.running[EventEnum.CAMERA.value] = True
        threading.Timer(self.interval_sleep, self.__camera_handler).start()
        """
        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __check_handler(self):
        """
        Handles the different scenarios in which the robot will be.
        This function is called on it's own thread every interval_check seconds.
        """
        # print(threading.active_count())
        sensor_values = self.sensor_handler.sensor_raw()
        # print(sensor_values)

        sensor = np.amax(sensor_values["sensor"]).astype(int)  # RANDOM CONDITION FOR TESTING
        if self.state != EventEnum.GLOBAL.value and sensor <= self.obstacle_threshold:  # CHECK HERE FOR THE GLOBAL CONDITION
            print("changing to GLOBAL!!")
            self.running[self.state] = False
            self.running[EventEnum.GLOBAL.value] = True
            self.state = EventEnum.GLOBAL.value
            threading.Timer(self.interval_sleep, self.__global_handler).start()

        elif self.state != EventEnum.LOCAL.value and sensor > self.obstacle_threshold:  # CHECK HERE FOR THE LOCAL CONDITION
            print("changing to LOCAL!!")
            self.running[self.state] = False
            self.running[EventEnum.LOCAL.value] = True
            self.state = EventEnum.LOCAL.value
            threading.Timer(self.interval_sleep, self.__local_handler).start()
            return  # stop the code so that __check_handler is called after avoiding objects

        elif self.state != EventEnum.STOP.value and sensor >= self.stop_threshold:  # CHECK HERE FOR THE STOP CONDITION
            print("changing to STOP!!")
            self.running[self.state] = False
            self.running[EventEnum.STOP.value] = True
            self.state = EventEnum.STOP.value
            threading.Timer(self.interval_sleep, self.__stop_handler).start()

        time.sleep(self.interval_sleep)
        self.__check_handler()
        return

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # print("inside __global_handler")
        goal = (15, 15)
        path = display_occupancy(self.final_occupancy_grid, (int(self.position[0]), int(self.position[1])), goal)
        update_path(self.thymio, path, self.position[0], self.position[1], self.position[2])

        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        ObstacleAvoidance(self.thymio, self.final_occupancy_grid, self.position)
        threading.Timer(self.interval_check, self.__check_handler).start()  # restart checking the correct state

        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __stop_handler(self):
        """
        Manages the thread for the STOP scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # print("inside __stop_handler")
        stop(self.thymio, verbose=True)

    def __kalman_handler(self):
        # print("inside __kalman_handler")
        speed = self.sensor_handler.speed()
        now = time.time()
        ts = now - self.kalman_time
        print("ts", ts)
        self.kalman_time = now
        self.delta_sl = speed['left_speed'] * ts / self.thymio_speed_to_mms / 1000
        self.delta_sr = speed['right_speed'] * ts / self.thymio_speed_to_mms / 1000

        self.position, self.covariance = kalman_filter(self.camera_measure, self.position, self.covariance,
                                                       self.delta_sr, self.delta_sl)
        print("kalman position ", self.position)

        if self.running[EventEnum.KALMAN.value]:
            time.sleep(self.interval_sleep)
            # self.__kalman_handler()
            self.__camera_handler()

    def __camera_handler(self):
        # print("inside __camera_handler")
        self.camera_measure = record_project()
        print("camera ", self.camera_measure)
        # TODO sleep until kalman_ts

        """if self.running[EventEnum.CAMERA.value]:
            time.sleep(self.interval_sleep)
            self.__camera_handler()"""
        if self.running[EventEnum.KALMAN.value]:
            time.sleep(self.interval_sleep)
            self.__kalman_handler()
