import time

import numpy as np

from src.displacement.movement import stop, rotate_time, move, advance_time
from src.displacement.planning import update_path
from src.kalman.kalmann_filter import KalmanHandler
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.localization import Localization
from src.path_planning.occupancy import display_occupancy
from src.thymio.Thymio import Thymio


class EventHandler:
    """
    """

    def __init__(self, thymio: Thymio, interval_camera=3, interval_odometry=0.05, interval_sleep=0.01,
                 obstacle_threshold=2000, epsilon_theta=5, epsilon_r=2.5):
        self.thymio: Thymio = thymio
        self.interval_camera = interval_camera
        self.interval_odometry = interval_odometry
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.case_size_cm = 2.5  # [cm]
        stop(self.thymio)

        self.final_occupancy_grid, self.goal = Localization().localize()
        self.kalman_handler = KalmanHandler(thymio=self.thymio)
        self.kalman_position = self.kalman_handler.get_camera()
        self.epsilon_theta = epsilon_theta  # [degrees]
        self.epsilon_r = epsilon_r  # [cm]
        print("initial positions: ", self.kalman_position)
        self.path, self.full_path = display_occupancy(self.final_occupancy_grid, (self.kalman_position[0], self.kalman_position[1]),
                                      self.goal)
        self.kalman_handler.start_recording()
        self.camera_timer = time.time()
        self.odometry_timer = time.time()
        self.__global_handler()

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_odometry seconds.
        """
        delta_r, delta_theta = update_path(self.path, self.kalman_position[0], self.kalman_position[1],
                                           self.kalman_position[2],
                                           self.case_size_cm)
        print("delta_r, delta_theta", delta_r, delta_theta)
        # TODO add scaling to slow down when close to goal
        # Apply rotation
        if abs(delta_theta) > self.epsilon_theta:
            left_dir, right_dir, turn_time = rotate_time(delta_theta)
            left_dir = left_dir * 0.5
            right_dir = right_dir * 0.5
            move(self.thymio, left_dir, right_dir, verbose=True)

        # Apply displacement
        elif abs(delta_r) > self.epsilon_r:
            print("done rotating")
            left_dir, right_dir, distance_time = advance_time(delta_r)
            left_dir = left_dir * 0.5
            right_dir = right_dir * 0.5
            move(self.thymio, left_dir, right_dir, verbose=True)

            # check if local avoidance needed
            sensor_values = self.kalman_handler.sensor_handler.sensor_raw()
            if np.amax(sensor_values["sensor"]).astype(int) >= self.obstacle_threshold:
                stop(self.thymio)
                self.kalman_handler.stop_recording()
                # TODO kalman
                self.__local_handler()
                self.kalman_handler.start_recording()
                self.camera_timer = time.time()
                self.odometry_timer = time.time()
                self.__global_handler()
                # TODO update the global path instead of __global_init()
        else:
            print("done advancing!")
            stop(self.thymio)
            left_dir = 0
            right_dir = 0
            self.path = np.delete(self.path, 0, 1)  # removes the step done from the non-concatenated lists

        if time.time() - self.camera_timer >= self.interval_camera:
            self.kalman_position = self.kalman_handler.get_kalman(True, left_dir, right_dir)
            self.camera_timer = time.time()
        elif time.time() - self.odometry_timer >= self.interval_odometry:
            self.kalman_position = self.kalman_handler.get_kalman(False, left_dir, right_dir)
            self.odometry_timer = time.time()

        if len(self.path[0]):
            time.sleep(self.interval_sleep)
            self.__global_handler()
        else:
            self.kalman_handler.stop_recording()
            stop(self.thymio)

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_odometry seconds.
        """
        print("inside __local_handler")
        ObstacleAvoidance(self.thymio, self.final_occupancy_grid, self.kalman_position)
