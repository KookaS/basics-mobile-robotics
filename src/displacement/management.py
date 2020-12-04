import time

import numpy as np

from src.displacement.movement import stop, rotate_time, move, advance_time
from src.displacement.planning import update_path
from src.kalman.kalmann_filter import KalmanHandler
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.localization import Localization
from src.path_planning.occupancy import display_occupancy, full_path_to_points
from src.thymio.Thymio import Thymio


class EventHandler:
    """
    This class manages all the different scenarios of the robot until it reaches the goal.
    """

    def __init__(self, thymio: Thymio, interval_camera=3, interval_odometry=0.1, interval_sleep=0.05,
                 obstacle_threshold=2000, epsilon_theta=8, epsilon_r=1.25):
        """
        Constructor of the class EventHandler.

        param thymio: class thymio, reference to the robot
        param interval_camera: time constant necessary to access to kalman odometry and measurement
        param interval_odometry: time constant necessary to access to kalman odometry
        param interval_sleep: time sleep constant before function loop calls
        param obstacle_threshold: condition to go into local avoidance
        param epsilion_theta: the tolerated angle deviation
        param epsilon_r: the tolerated distance deviation


        :return:
        """
        self.thymio: Thymio = thymio
        self.interval_camera = interval_camera
        self.interval_odometry = interval_odometry
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.case_size_cm = 2.5  # [cm]
        stop(self.thymio)

        self.final_occupancy_grid, self.goal = Localization().localize()
        self.kalman_handler = KalmanHandler(thymio=self.thymio)
        self.kalman_handler.camera.open_camera()
        self.kalman_position = self.kalman_handler.get_camera()
        self.epsilon_theta = epsilon_theta  # [degrees]
        self.epsilon_r = epsilon_r  # [cm]
        print("initial positions: ", self.kalman_position)
        self.path, self.full_path = display_occupancy(self.final_occupancy_grid,
                                                      (self.kalman_position[0], self.kalman_position[1]),
                                                      self.goal)
        # self.kalman_handler.start_recording() TODO
        self.camera_timer = time.time()
        self.odometry_timer = time.time()
        self.__global_handler()

    def __global_handler(self):
        """
        Function called in loop until the goal is reached. Kalman, global displacement, local avoidance happens here.
        """
        """
        # odometry and measurement kalman
        if time.time() - self.camera_timer >= self.interval_camera:
            self.kalman_position = self.kalman_handler.get_camera()
            # self.kalman_position = self.kalman_handler.get_kalman(True)
            self.camera_timer = time.time()
        """

        # odometry kalman
        if time.time() - self.odometry_timer >= self.interval_odometry:
            self.kalman_position = self.kalman_handler.get_camera()
            # self.kalman_position = self.kalman_handler.get_kalman(False)  # TODO
            self.odometry_timer = time.time()

        # get orientation and displacement needed to reach next point of the path
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
            move(self.thymio, left_dir, right_dir)

        # Apply displacement
        elif abs(delta_r) > self.epsilon_r:
            print("done rotating")
            left_dir, right_dir, distance_time = advance_time(delta_r)
            left_dir = left_dir * 0.5
            right_dir = right_dir * 0.5
            move(self.thymio, left_dir, right_dir)

            # check if local avoidance needed
            sensor_values = self.kalman_handler.sensor_handler.sensor_raw()
            if np.amax(sensor_values["sensor"]).astype(int) >= self.obstacle_threshold:
                print("INSIDE LOCAL AVOIDANCE!")
                stop(self.thymio)
                self.kalman_handler.stop_recording()
                self.kalman_handler.camera.close_camera()
                self.__local_handler()
                self.kalman_handler.camera.open_camera()
                self.kalman_handler.start_recording()
                self.camera_timer = time.time()
                self.odometry_timer = time.time()
        else:
            # point in the path has been reached
            print("done advancing!")
            stop(self.thymio)
            time.sleep(20)
            self.path = np.delete(self.path, 0, 1)  # removes the step done from the non-concatenated lists

        # if there still exist a path, iterates once more
        if len(self.path[0]):
            time.sleep(self.interval_sleep)
            self.__global_handler()

        # no more path, go back to main
        else:
            self.kalman_handler.stop_recording()
            self.kalman_handler.camera.close_camera()
            stop(self.thymio)

    def __local_handler(self):
        """
        Local avoidance handler that updates the path after done avoiding.
        """
        print("inside __local_handler")
        obstacle = ObstacleAvoidance(self.thymio, self.full_path, self.final_occupancy_grid, self.kalman_position)
        self.full_path = obstacle.full_path
        self.path = full_path_to_points(self.full_path)  # concatenated path
        self.kalman_position = obstacle.kalman_position
