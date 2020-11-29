import threading
import time

import numpy as np
from enum import Enum

from src.displacement.movement import stop, rotate, advance_time, move
from src.displacement.planning import update_path
from src.kalman.kalmann_filter import kalman_filter
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.localization import Localization
from src.path_planning.occupancy import display_occupancy
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.vision.camera import record_project


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    RECORD = 0


class EventHandler:
    """
    Example:
            EventHandler(thymio=th, interval_check=5)
    """

    def __init__(self, thymio: Thymio, interval_check=0.2, interval_sleep=0.05, obstacle_threshold=2000,
                 stop_threshold=3500, goal=(20, 15)):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.sensor_handler = SensorHandler(self.thymio)
        self.covariance = 1 * np.ones([3, 3])
        self.thymio_speed_to_mms = 0.4347
        self.kalman_time = time.time()
        self.ts = 0
        self.delta_sr = 0
        self.delta_sl = 0
        self.running = []
        self.record_left = [0]
        self.record_right = [0]
        self.goal = goal
        for _ in EventEnum:
            self.running.append(False)
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        self.localize = Localization()
        self.final_occupancy_grid, self.goal = self.localize.localize()
        self.__camera_handler()
        self.position = [0, 0, 0]
        self.position, self.covariance = kalman_filter(self.camera_measure, self.position,
                                                       self.covariance, 0, 0, True)
        print("initial positions: ", self.position)
        self.__global_init()

    def __global_init(self):
        self.goal = (15, 15)
        self.path = display_occupancy(self.final_occupancy_grid, (int(self.position[0]), int(self.position[1])),
                                      self.goal)
        self.__global_handler()

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # print("inside __global_handler")
        print("path_x: ", self.path[0])
        print("path_y: ", self.path[1])
        delta_r, delta_theta = update_path(self.path, self.position[0], self.position[1], self.position[2])

        # Apply rotation
        self.kalman_time = time.time()
        rotate(self.thymio, delta_theta, verbose=True)
        self.__kalman_handler(False)

        # apply displacement
        self.kalman_time = time.time()
        l_speed, r_speed, distance_time = advance_time(delta_r)
        start_advancing = time.time()
        move(self.thymio, l_speed, r_speed)
        self.running[EventEnum.RECORD.value] = True
        threading.Thread(target=self.__record_handler).start()

        while start_advancing - time.time() < distance_time:
            # check if local avoidance needed
            sensor_values = self.sensor_handler.sensor_raw()
            if np.amax(sensor_values["sensor"]).astype(int) < self.obstacle_threshold:
                stop(self.thymio)
                self.__local_handler()
                self.__global_init()
            time.sleep(self.interval_sleep)

        # threading.Timer(self.interval_check, self.__record_handler).start()
        self.record_right = filter(lambda number: number < 30, self.record_right)  # removes the speeds below 30
        self.record_left = filter(lambda number: number < 30, self.record_left)
        print("right records: ", self.record_right)
        print("left records: ", self.record_left)

        self.__kalman_handler(True)
        self.path = np.delete(self.path, 0, 1)  # removes the step done from the non-concatenated lists

        if self.path.shape[0]:
            time.sleep(self.interval_sleep)
            self.__global_handler()

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        print("inside __local_handler")
        ObstacleAvoidance(self.thymio, self.final_occupancy_grid, self.position)
        self.__global_handler()

    def __stop_handler(self):
        stop(self.thymio, verbose=True)

    def __kalman_handler(self, measurement):
        # print("inside __kalman_handler")
        now = time.time()
        ts = now - self.kalman_time
        print("kalman ts", ts)
        speed_left = np.mean(self.record_left)
        speed_right = np.mean(self.record_right)
        self.record_right = [0]
        self.record_left = [0]
        # TODO check self.thymio_speed_to_mms
        self.delta_sl = speed_left * ts / self.thymio_speed_to_mms / 1000
        self.delta_sr = speed_right * ts / self.thymio_speed_to_mms / 1000

        if measurement:
            self.__camera_handler()

        self.position, self.covariance = kalman_filter(self.camera_measure, self.position,
                                                       self.covariance,
                                                       self.delta_sr, self.delta_sl, measurement)
        print("kalman position ", self.position)

    def __camera_handler(self):
        # print("inside __camera_handler")
        self.camera_measure = record_project()
        print("camera position ", self.camera_measure)

    def __record_handler(self):
        speed = self.sensor_handler.speed()
        self.record_right.append(speed['right_speed'])
        self.record_left.append(speed['left_speed'])

        if self.running[EventEnum.RECORD.value]:
            time.sleep(self.interval_sleep)
            self.__record_handler()
