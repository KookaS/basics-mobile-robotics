import threading
import time

import numpy as np
from enum import Enum

from src.displacement.movement import stop, rotate
from src.displacement.planning import update_path
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.occupancy import display_occupancy, create_grid
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.vision.kalman import detect_object


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    GLOBAL = 0
    LOCAL = 1
    STOP = 2
    KALMAN = 3


class EventHandler:
    """
    Example:
            EventHandler(thymio=th, interval_check=5)
    """

    def __init__(self, thymio: Thymio, interval_check=0.2, interval_sleep=0.1, obstacle_threshold=2000,
                 stop_threshold=3500):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.sensor_handler = SensorHandler(self.thymio)
        self.running = []
        for _ in EventEnum:
            self.running.append(False)
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        self.final_occupancy_grid = create_grid()
        threading.Timer(self.interval_check, self.__check_handler).start()
        """
        self.state = EventEnum.KALMAN.value
        self.running[EventEnum.KALMAN.value] = True
        threading.Timer(self.interval_sleep, self.__kalman_handler).start()
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
        # TODO: give actual position
        self.position = (2, 2, -30.0)
        # path = display_occupancy(self.final_occupancy_grid, self.position)
        # update_path(self.thymio, path)

        # self.running[EventEnum.KALMAN.value] = False    # stop kalman when we reached goal
        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # TODO: give actual position
        self.position = (2, 2, -30.0)
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
        print("inside __kalman_handler")
        detect_object()

        if self.running[EventEnum.KALMAN.value]:
            time.sleep(self.interval_sleep)
            self.__kalman_handler()
