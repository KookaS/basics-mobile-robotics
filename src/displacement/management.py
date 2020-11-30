import os
import threading
import time

import numpy as np
from enum import Enum

from src.displacement.movement import stop, rotate, advance_time, move, rotate_time, rotate_thread, advance_thread
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

    def __init__(self, thymio: Thymio, interval_check=0.1, interval_sleep=0.05, obstacle_threshold=2000,
                 stop_threshold=3500, goal_threshold=2, low_blue=np.array([98, 134, 106]), up_blue=np.array([109, 225, 174])):
        self.goal_threshold = goal_threshold  # nb of cubes around the goal
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.case_size_cm = 2.5
        self.sensor_handler = SensorHandler(self.thymio)
        self.covariance = 1 * np.ones([3, 3])
        self.thymio_speed_to_mm_s = float(os.getenv("SPEED_80_TO_MM_S"))
        self.kalman_time = time.time()
        self.ts = 0
        self.delta_sr = 0
        self.delta_sl = 0
        self.running = []
        self.record_left = [0]
        self.record_right = [0]
        self.goal = (15, 15)
        self.low_blue = low_blue
        self.up_blue = up_blue
        for _ in EventEnum:
            self.running.append(False)
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        # TODO add goal detection
        self.localize = Localization(self.low_blue, self.up_blue)
        self.final_occupancy_grid, self.goal = self.localize.localize()
        self.__camera_handler()
        self.position = [0, 0, 0]
        conv_cam = [self.camera_measure[0] * self.case_size_cm / 100, self.camera_measure[1] * self.case_size_cm / 100,
                    np.deg2rad(self.camera_measure[2])]
        temp, self.covariance = kalman_filter(conv_cam, self.position, self.covariance, 0, 0, True)
        self.position = [temp[0] * 100 / self.case_size_cm, temp[1] * 100 / self.case_size_cm, np.rad2deg(temp[2])]

        print("initial positions: ", self.position)
        self.__global_init()

    def __global_init(self):
        self.goal = (20, 10)
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
        delta_r, delta_theta = update_path(self.path, int(self.position[0]), int(self.position[1]), self.position[2])

        # Apply rotation
        self.kalman_time = time.time()
        self.right_dir = int(np.sign(delta_theta))
        self.left_dir = -int(np.sign(delta_theta))
        thread = rotate_thread(self.thymio, delta_theta, verbose=True)
        self.running[EventEnum.RECORD.value] = True
        threading.Thread(target=self.__record_handler).start()

        while thread.is_alive():
            # print("still not rotated at next point!")
            self.__kalman_handler(False)
            time.sleep(self.interval_sleep)  # TODO check interval here

        # apply displacement
        print("done rotating")
        self.kalman_time = time.time()
        self.right_dir = int(np.sign(delta_r))
        self.left_dir = int(np.sign(delta_r))
        thread = advance_thread(self.thymio, delta_r, verbose=True)
        while thread.is_alive():
            # print("still not moves at next point!")
            self.__kalman_handler(False)
            # check if local avoidance needed
            sensor_values = self.sensor_handler.sensor_raw()
            if np.amax(sensor_values["sensor"]).astype(int) >= self.obstacle_threshold:
                stop(self.thymio)
                self.running[EventEnum.RECORD.value] = False
                self.__record_reset()
                self.__local_handler()
                self.__global_init()
            time.sleep(self.interval_check)

        print("reaching next point!")
        stop(self.thymio)
        self.running[EventEnum.RECORD.value] = False
        # self.record_right = filter(lambda number: number < 30, self.record_right)  # removes the speeds below 30
        # self.record_left = filter(lambda number: number < 30, self.record_left)
        self.__kalman_handler(True)
        self.path = np.delete(self.path, 0, 1)  # removes the step done from the non-concatenated lists

        if len(self.path[0]):
            self.__global_handler()
        else:
            stop(self.thymio)
            self.running[EventEnum.RECORD.value] = False

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
        self.kalman_time = now
        # print("kalman ts", ts)
        speed_left = np.mean(self.record_left) * self.left_dir
        speed_right = np.mean(self.record_right) * self.right_dir
        # print("right records: ", self.record_right)
        # print("left records: ", self.record_left)
        # print("left average: ", speed_left)
        # print("right average: ", speed_right)
        self.__record_reset()
        self.delta_sl = speed_left * ts * self.thymio_speed_to_mm_s / 1000  # [m]
        self.delta_sr = speed_right * ts * self.thymio_speed_to_mm_s / 1000  # [m]
        # print("self.delta_sl [m]: ", self.delta_sl)
        # print("self.delta_sr [m]: ", self.delta_sr)

        if measurement:
            self.__camera_handler()

        conv_pos = [self.position[0] * self.case_size_cm / 100, self.position[1] * self.case_size_cm / 100,
                    np.deg2rad(self.position[2])]
        conv_cam = [self.camera_measure[0] * self.case_size_cm / 100, self.camera_measure[1] * self.case_size_cm / 100,
                    np.deg2rad(self.camera_measure[2])]
        temp, self.covariance = kalman_filter(conv_cam, conv_pos, self.covariance, self.delta_sr,
                                              self.delta_sl, measurement)
        angle = (np.rad2deg(temp[2]) + 180.0) % 360.0 - 180.0
        self.position = [temp[0] * 100 / self.case_size_cm, temp[1] * 100 / self.case_size_cm, angle]
        print("kalman position ", self.position)

    def __camera_handler(self):
        # print("inside __camera_handler")
        self.camera_measure = record_project()
        print("camera position ", self.camera_measure)

    def __record_handler(self):
        speed = self.sensor_handler.speed()
        right = speed['right_speed']
        if right > 110:
            right = 100
        left = speed['left_speed']
        if left > 110:
            left = 100
        self.record_right.append(right)
        self.record_left.append(left)

        if self.running[EventEnum.RECORD.value]:
            time.sleep(self.interval_sleep / 10)
            self.__record_handler()

    def __record_reset(self):
        self.record_right = []
        self.record_left = []
