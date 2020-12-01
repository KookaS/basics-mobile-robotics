import os
import threading
import time

import numpy as np
from enum import Enum

from src.displacement.movement import stop, rotate_thread, advance_thread, rotate_time, move
from src.displacement.planning import update_path
from src.kalman.kalmann_filter import Kalman, KalmanHandler
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.localization import Localization
from src.path_planning.occupancy import display_occupancy
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.vision.camera import Camera, Colors


class EventHandler:
    """
    """

    def __init__(self, thymio: Thymio, interval_camera=3, interval_odometry=0.05, obstacle_threshold=2000,
                 stop_threshold=3500, goal_threshold=2):
        self.goal_threshold = goal_threshold  # nb of cubes around the goal
        self.thymio: Thymio = thymio
        self.interval_camera = interval_camera
        self.interval_odometry = interval_odometry
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.case_size_cm = 2.5
        self.goal = (15, 15)
        self.final_occupancy_grid, self.goal = Localization().localize()
        self.kalman_handler = KalmanHandler(thymio=self.thymio)
        self.kalman_position = self.kalman_handler.get_camera()
        print("initial positions: ", self.kalman_position)
        self.path = display_occupancy(self.final_occupancy_grid, (self.kalman_position[0], self.kalman_position[1]),
                                      self.goal)
        self.__global_handler()

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_odometry seconds.
        """

        delta_r, delta_theta = update_path(self.path, self.kalman_position[0], self.kalman_position[1],
                                           self.kalman_position[2],
                                           self.case_size_cm)

        # Apply rotation
        left_dir, right_dir, turn_time = rotate_time(delta_theta)
        movement_time = time.time()
        move(self.thymio, left_dir, right_dir, verbose=True)
        self.kalman_handler.start_recording()

        while time.time() - movement_time < turn_time:
            print("still not rotated at next point!")
            self.kalman_handler.get_kalman(left_dir, right_dir, False)
            time.sleep(self.interval_odometry)

        # Apply displacement
        print("done rotating")
        right_dir = int(np.sign(delta_r))
        left_dir = int(np.sign(delta_r))
        thread = advance_thread(self.thymio, delta_r, verbose=True)
        while thread.is_alive():
            # print("still not moves at next point!")
            self.kalman_handler.get_kalman(left_dir, right_dir, False)
            # check if local avoidance needed
            sensor_values = self.kalman_handler.sensor_handler.sensor_raw()
            if np.amax(sensor_values["sensor"]).astype(int) >= self.obstacle_threshold:
                stop(self.thymio)
                self.kalman_handler.stop_recording()
                self.__local_handler()
                # TODO update the global path instead of __global_init()
            time.sleep(self.interval_camera)

        print("done advancing!")
        stop(self.thymio)
        self.kalman_handler.get_kalman(left_dir, right_dir, True)
        self.kalman_handler.stop_recording()
        self.path = np.delete(self.path, 0, 1)  # removes the step done from the non-concatenated lists
        print("kalman position ", self.kalman_position)

        if len(self.path[0]):
            self.__global_handler()
        else:
            stop(self.thymio)

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_odometry seconds.
        """
        print("inside __local_handler")
        ObstacleAvoidance(self.thymio, self.final_occupancy_grid, self.kalman_position)
        self.__global_handler()
