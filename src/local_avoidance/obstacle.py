import time
from enum import Enum
import numpy as np

from src.kalman.kalmann_filter import KalmanHandler
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.displacement.movement import stop, advance_time, move, rotate_time

class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    RIGHT = 0
    LEFT = 1


class ObstacleAvoidance:

    def __init__(self, thymio: Thymio, kalman_handler, full_path, final_occupancy_grid, interval_sleep=0.05,
                 distance_avoidance=2.5,
                 angle_avoidance=5.0, square=2.5, wall_threshold=3600, clear_thresh=2400, case_size_cm=2.5):
        self.thymio = thymio
        self.full_path = full_path
        self.case_size_cm = case_size_cm
        self.sensor_handler = SensorHandler(thymio)
        self.interval_sleep = interval_sleep
        self.distance_avoidance = distance_avoidance
        self.angle_avoidance = angle_avoidance
        self.final_occupancy_grid = final_occupancy_grid
        self.kalman_handler = kalman_handler
        self.kalman_position = self.kalman_handler.get_camera()
        self.kalman_position = [self.kalman_position[0] / 2.5, self.kalman_position[1] / 2.5, self.kalman_position[2]]
        self.square = square
        self.wall_threshold = wall_threshold
        self.clear_thresh = clear_thresh
        self.ONE_STEP = 1
        self.FIVE_STEPS = 4  # TODO
        self.width_case = 2.5
        self.__update_path()
        self.__obstacle_avoidance()
        print("LOCAL AVOIDANCE TERMINER")
        self.__update_path()

    def __obstacle_avoidance(self):
        sensor_values = self.sensor_handler.sensor_raw()["sensor"]
        if (sensor_values[1] > self.wall_threshold) and (sensor_values[3] > self.wall_threshold):  # both sides
            if sensor_values[3] > sensor_values[1]:
                rotated = EventEnum.LEFT
            else:
                rotated = EventEnum.RIGHT
        elif (sensor_values[3] > self.wall_threshold) or (sensor_values[4] > self.wall_threshold):  # right side
            rotated = EventEnum.LEFT
        elif (sensor_values[0] > self.wall_threshold) or (sensor_values[1] > self.wall_threshold):  # left side
            rotated = EventEnum.RIGHT
        elif sensor_values[2] > self.wall_threshold:  # center
            rotated = EventEnum.LEFT
        else:
            rotated = EventEnum.LEFT
        print("rotated:", rotated)
        sensor_values = self.sensor_handler.sensor_raw()["sensor"]
        if (rotated == EventEnum.LEFT) and (sensor_values[3] < self.clear_thresh) and (
                sensor_values[2] > self.wall_threshold):  # TODO
            print("losange go left")
            self.rotate(self.thymio, 20)
            # time.sleep(5)
        elif (rotated == EventEnum.RIGHT) and (sensor_values[1] < self.clear_thresh) and (
                sensor_values[2] > self.wall_threshold):
            self.rotate(self.thymio, -20)
            print("losange go right")
            # time.sleep(5)

        condition = True
        while condition:
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            if rotated == EventEnum.LEFT:
                self.rotate(self.thymio, self.angle_avoidance)
                if sensor_values[3] <= self.clear_thresh:
                    break
            else:
                self.rotate(self.thymio, -self.angle_avoidance)
                if sensor_values[1] <= self.clear_thresh:
                    break
        if rotated == EventEnum.LEFT:
            # time.sleep(2)
            self.rotate(self.thymio, 15)
            print("reajustement gauche")
            # time.sleep(2)
        else:
            # time.sleep(2)
            self.rotate(self.thymio, -15)
            print("reajustement droite")
            # time.sleep(2)

        print("Première rotation done")
        stop(self.thymio)
        global_path = False

        while not global_path:
            obstacle, global_path = self.__cote_avoid(rotated)
            if obstacle and rotated == EventEnum.LEFT:
                print("changement de sens g->d")
                self.rotate(self.thymio, 180)
                rotated = EventEnum.RIGHT

            elif obstacle and rotated == EventEnum.RIGHT:
                print("changement de sens d->g")
                self.rotate(self.thymio, -180)
                rotated = EventEnum.LEFT

    def __turn_avoid(self, rotated):
        if rotated == EventEnum.LEFT:
            self.rotate(self.thymio, self.angle_avoidance)
        else:
            self.rotate(self.thymio, -self.angle_avoidance)

    def __cote_avoid(self, rotated):
        condition = True
        obstacle = False
        global_path = False
        while condition:

            obstacle, global_path = self.__check_global_obstacles_and_global_path(
                self.ONE_STEP)
            if obstacle:
                return obstacle, global_path
            self.advance(self.thymio, self.ONE_STEP * self.distance_avoidance)
            if global_path:
                return obstacle, global_path

            if rotated == EventEnum.LEFT:
                self.rotate(self.thymio, -30)
            else:
                self.rotate(self.thymio, 30)

            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            if (rotated == EventEnum.LEFT) and (sensor_values[4] > 2000):
                print("cas 1")
                self.rotate(self.thymio, 30)
            elif (rotated == EventEnum.RIGHT) and (sensor_values[0] > 2000):
                print("cas 2")
                self.rotate(self.thymio, -30)
            elif (rotated == EventEnum.LEFT) and (sensor_values[4] < 2000):
                print("cas 3")
                print("start rotatingn 3")
                self.rotate(self.thymio, 30)

                # time.sleep(2)
                print("done rotating 3")

                obstacle, global_path = self.__check_global_obstacles_and_global_path(
                    self.FIVE_STEPS)
                if obstacle:
                    return obstacle, global_path
                print("start advancing 3")
                self.advance(self.thymio, self.FIVE_STEPS * self.distance_avoidance)
                # time.sleep(2)
                print("done advancing 3")
                if global_path:
                    return obstacle, global_path

                sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                while sensor_values[4] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print("cas 7.2")
                    self.rotate(self.thymio, -5)
                self.rotate(self.thymio, 20)
                break
            elif (rotated == EventEnum.RIGHT) and (sensor_values[0] < 2000):
                print("cas 4.2")
                self.rotate(self.thymio, -30)

                obstacle, global_path = self.__check_global_obstacles_and_global_path(
                    self.FIVE_STEPS)
                if obstacle:
                    return obstacle, global_path
                self.advance(self.thymio, self.FIVE_STEPS * self.distance_avoidance)
                if global_path:
                    return obstacle, global_path

                sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                while sensor_values[0] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print("cas 7.2")
                    self.rotate(self.thymio, 5)
                self.rotate(self.thymio, -20)
                break
        return obstacle, global_path

    def __check_global_obstacles_and_global_path(self, length_advance):
        global_path = False
        obstacle = False
        x = self.kalman_position[0]
        y = self.kalman_position[1]
        theta = self.kalman_position[2] - 90  # TODO

        x_discrete = round(x)
        y_discrete = round(y)

        if (theta >= -22.5) and (theta < 22.5):
            dir_x = 1
            dir_y = 0
        elif (theta >= 22.5) and (theta < 67.5):
            dir_x = 1
            dir_y = 1
        elif (theta >= 67.5) and (theta < 112.5):
            dir_x = 0
            dir_y = 1
        elif (theta >= 112.5) and (theta < 157.5):
            dir_x = -1
            dir_y = 1
        elif ((theta >= 157.5) and (theta <= 181)) or ((theta >= -181) and (theta < -157.5)):
            dir_x = -1
            dir_y = 0
        elif (theta >= -157.5) and (theta < -112.5):
            dir_x = -1
            dir_y = -1
        elif (theta >= -112.5) and (theta < -67.5):
            dir_x = 0
            dir_y = -1
        elif (theta >= -67.5) and (theta < -22.5):
            dir_x = 1
            dir_y = -1
        else:
            dir_x = 0
            dir_y = 0
            print(" angle is not between -180 and 180 degrees")

        print("x_discrete, y_discrete", x_discrete, y_discrete)
        print("dir_x, dir_y", dir_x, dir_y)
        x_next_step = int(x_discrete + length_advance * dir_x)
        y_next_step = int(y_discrete + length_advance * dir_y)
        print("x_next_step, y_next_step", x_next_step, y_next_step)
        if (x_next_step > 29) or (x_next_step < 2) or (y_next_step > 26) or (y_next_step < 2):
            obstacle = True
            print("next step will be out of the map")
            return obstacle, global_path

        elif self.final_occupancy_grid[x_next_step][y_next_step] == 1:
            obstacle = True
            print("next step will reach an obstacle")
            return obstacle, global_path

        approx_position_x = [x_next_step - 1, x_next_step, x_next_step + 1]
        approx_position_y = [y_next_step - 1, y_next_step, y_next_step + 1]
        if length_advance > 3:
            x_next_step_2 = int(x_discrete + 2 * dir_x)
            y_next_step_2 = int(y_discrete + 2 * dir_y)
            approx_position_x_2 = [x_next_step - 1, x_next_step, x_next_step + 1]
            approx_position_y_2 = [y_next_step - 1, y_next_step, y_next_step + 1]

        for k in range(len(self.full_path[0])):
            x_path = self.full_path[0][k]
            y_path = self.full_path[1][k]

            for i in range(len(approx_position_x)):
                for j in range(len(approx_position_y)):

                    x_pos = approx_position_x[i]
                    y_pos = approx_position_y[j]
                    if length_advance > 3:
                        x_pos_2 = approx_position_x_2[i]
                        y_pos_2 = approx_position_y_2[j]
                        if x_pos_2 == x_path and y_pos_2 == y_path:
                            global_path = True
                            print("next step will reach global path")
                            return obstacle, global_path
                    if x_pos == x_path and y_pos == y_path:
                        global_path = True
                        print("next step will reach global path")
                        return obstacle, global_path

        return obstacle, global_path

    def __update_path(self):
        x = self.kalman_position[0]
        y = self.kalman_position[1]
        x_discrete = round(x)
        y_discrete = round(y)
        print("x_discrete,y_discrete ", x_discrete, y_discrete)

        approx_position_x = [x_discrete - 2, x_discrete - 1, x_discrete, x_discrete + 1, x_discrete + 2]
        approx_position_y = [y_discrete - 2, y_discrete - 1, y_discrete, y_discrete + 1, y_discrete + 2]
        exit_loop = False
        k_pos = []
        print("full_path:", self.full_path)
        for k in range(len(self.full_path[0])):
            x_path = self.full_path[0][k]
            y_path = self.full_path[1][k]
            print("x_path, y_path :", x_path, y_path)
            exit_for = False
            for i in range(len(approx_position_x)):
                for j in range(len(approx_position_y)):
                    x_pos = approx_position_x[i]
                    y_pos = approx_position_y[j]
                    if x_pos == x_path and y_pos == y_path:
                        print("in for loop with condition =TRue")
                        k_pos.append(k)
                        exit_for = True
                        exit_loop = True
                        break
                if exit_for:
                    break
            if (not exit_for) and exit_loop:
                print("Full list updated avec k =", k_pos)
                big_k = k_pos[-1] + 1
                print("full_path_actuel", self.full_path)
                # self.full_path = [self.full_path[0][big_k:].tolist(), self.full_path[1][big_k:].tolist()]
                for i in range(big_k):
                    self.full_path = np.delete(self.full_path, 0, 1)
                print("Updated full_path", self.full_path)
                return

    def advance(self, thymio: Thymio, distance: float, speed_ratio: int = 1, verbose: bool = False):
        """
        Moves straight of a desired distance

        :param thymio:      the class to which the robot is referred to
        :param distance:    distance in cm by which we want to move, positive or negative
        :param speed_ratio:       the speed factor at which the robot goes
        :param verbose:     printing the speed in the terminal
        :return: timer to check if it is still alive or not
        """
        left_dir, right_dir, distance_time = advance_time(distance, speed_ratio)
        # Printing the speeds if requested
        if verbose:
            # print("\t\t Advance speed & time : ", l_speed, r_speed, distance_time)
            print("\t\t Advance of cm: ", distance)

        move(thymio, left_dir, right_dir)
        # self.kalman_handler.start_recording()
        time.sleep(distance_time)
        stop(thymio)  #TODO remove if kalmann
        self.kalman_position = self.kalman_handler.get_camera()
        self.kalman_position = [self.kalman_position[0] / 2.5, self.kalman_position[1] / 2.5, self.kalman_position[2]]
        # self.kalman_position = self.kalman_handler.get_kalman(True)
        # self.kalman_handler.stop_recording()

    def rotate(self, thymio: Thymio, angle: float, verbose: bool = False):
        """
        Rotates of the desired angle

        :param thymio:      the class to which the robot is referred to
        :param angle:       angle in radians by which we want to rotate, positive or negative
        :param verbose:     printing the speed in the terminal
        :return: timer to check if it is still alive or not
        """

        left_dir, right_dir, turn_time = rotate_time(angle)
        # Printing the speeds if requested
        if verbose:
            # print("\t\t Rotate speed & time : ", l_speed, r_speed, turn_time)
            print("\t\t Rotate of degrees : ", angle)

        move(thymio, left_dir / 2, right_dir / 2)
        # self.kalman_handler.start_recording()
        time.sleep(turn_time * 2)
        stop(thymio)  # TODO if kalmann
        # self.kalman_position = self.kalman_handler.get_kalman(True)
        self.kalman_position = self.kalman_handler.get_camera()
        self.kalman_position = [self.kalman_position[0] / 2.5, self.kalman_position[1] / 2.5, self.kalman_position[2]]
        # self.kalman_handler.stop_recording()
