import math
import threading
import time
from enum import Enum
import numpy as np
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.displacement.movement import stop, rotate, advance


def test_saw_wall(thymio: Thymio, wall_threshold=3500, verbose=False) -> bool:
    """
    Tests whether one of the proximity sensors saw a wall
    :param thymio:          The file location of the spreadsheet
    :param wall_threshold:  threshold starting which it is considered that the sensor saw a wall
    :param verbose:         whether to print status messages or not
    :return: bool           existence of wall or not
    """

    if any([x > wall_threshold for x in thymio['prox.horizontal'][:-2]]):
        if verbose:
            print("\t\t Saw a wall")
        return True
    return False


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    RIGHT = 0
    LEFT = 1


class ObstacleAvoidance:

    def __init__(self, thymio: Thymio, full_path=0, final_occupancy_grid=0, position=0, interval_sleep=0.05, distance_avoidance=2.5,
                 angle_avoidance=5.0, square=2.5, wall_threshold=3000, clear_thresh=2400):
        self.thymio = thymio
        self.full_path = full_path
        self.sensor_handler = SensorHandler(thymio)
        self.interval_sleep = interval_sleep
        self.distance_avoidance = distance_avoidance
        self.angle_avoidance = angle_avoidance
        self.final_occupancy_grid = final_occupancy_grid
        self.position = position
        self.square = square
        self.wall_threshold = wall_threshold
        self.clear_thresh = clear_thresh
        self.ONE_STEP = 1
        self.FIVE_STEPS = 5
        stop(self.thymio)
        self.__obstacle_avoidance()

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

        condition = True
        while condition:
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            if rotated == EventEnum.LEFT:
                rotate(self.thymio, self.angle_avoidance)
                if sensor_values[3] <= self.clear_thresh:

                    break
            else:
                rotate(self.thymio, -self.angle_avoidance)
                if sensor_values[1] <= self.clear_thresh:
                    break

        self.thymio.set_var("motor.left.target", 0)
        self.thymio.set_var("motor.right.target", 0)
        global_path = False

        while not global_path:
            obstacle, global_path = self.__cote_avoid(rotated)
            if obstacle and rotated == EventEnum.LEFT:
                rotate(self.thymio, 180)
                rotated = EventEnum.RIGHT

            elif obstacle and rotated == EventEnum.RIGHT:
                rotate(self.thymio, -180)
                rotated = EventEnum.LEFT

    def __turn_avoid(self, rotated):
        if rotated == EventEnum.LEFT:
            rotate(self.thymio, self.angle_avoidance)
        else:
            rotate(self.thymio, -self.angle_avoidance)

    def __cote_avoid(self, rotated):
        condition = True
        obstacle = False
        global_path = False
        while condition:

            obstacle, global_path = self.__check_global_obstacles_and_global_path(
                self.ONE_STEP * self.distance_avoidance)
            if obstacle:
                return obstacle, global_path
            advance(self.thymio, self.ONE_STEP*self.distance_avoidance)
            if obstacle:
                return obstacle, global_path

            if rotated == EventEnum.LEFT:
                rotate(self.thymio, -30)
            else:
                rotate(self.thymio, 30)

            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            if (rotated == EventEnum.LEFT) and (sensor_values[4] > 2000):
                print(1)
                rotate(self.thymio, 30)
            elif (rotated == EventEnum.RIGHT) and (sensor_values[0] > 2000):
                print(2)
                rotate(self.thymio, -30)
            elif (rotated == EventEnum.LEFT) and (sensor_values[4] < 2000):
                print(3)
                rotate(self.thymio, 30)

                obstacle, global_path = self.__check_global_obstacles_and_global_path(
                    self.FIVE_STEPS * self.distance_avoidance)
                if obstacle:
                    return obstacle, global_path
                advance(self.thymio, self.FIVE_STEPS * self.distance_avoidance)
                if obstacle:
                    return obstacle, global_path

                while sensor_values[4] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print(7)
                    rotate(self.thymio, -5)
                rotate(self.thymio, 20)
                break
            elif (rotated == EventEnum.RIGHT) and (sensor_values[0] < 2000):
                print(4)
                rotate(self.thymio, -30)

                obstacle, global_path = self.__check_global_obstacles_and_global_path(
                    self.FIVE_STEPS * self.distance_avoidance)
                if obstacle:
                    return obstacle, global_path
                advance(self.thymio, self.FIVE_STEPS * self.distance_avoidance)
                if obstacle:
                    return obstacle, global_path

                while sensor_values[0] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print(7)
                    rotate(self.thymio, 5)
                rotate(self.thymio, -20)
                break
        return obstacle, global_path

    def __check_global_obstacles_and_global_path(self, length_advance):
        global_path = False
        obstacle = False
        x = self.position[0]
        y = self.position[1]
        theta = self.position[2]

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
        elif ((theta >= 157.5) and (theta <= 180)) or ((theta >= -180) and (theta < -157.5)):
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

        x_next_step = x_discrete + length_advance * dir_x
        y_next_step = y_discrete + length_advance * dir_y
        if self.final_occupancy_grid[x_next_step][y_next_step] == 1:
            obstacle = True
            print("next step will reach an obstacle")

        approx_position_x = [x_next_step - 1, x_next_step, x_next_step + 1]
        approx_position_y = [y_next_step - 1, y_next_step, y_next_step + 1]

        for k in range(len(self.full_path[0])):
            x_path = self.full_path[0][k]
            y_path = self.full_path[1][k]

            for i in range(len(approx_position_x)):
                for j in range(len(approx_position_y)):

                    x_pos = approx_position_x[i]
                    y_pos = approx_position_y[j]

                    if x_pos == x_path and y_pos == y_path:
                        global_path = True
                        print("next step will reach global path")
                        break

        return obstacle, global_path

'''
    Risque de ne pas voir un obstacle (on regarde que 5 cases plus loins, du coup on peut louper le coin d'un obstacle )
    len(self.full_path[0]) pas sûr que cela compile
    Il faut penser à reduir le full_path
    
    Comment on gère si : obstacle = True et global_path = True ?
    x_pos == x_path and y_pos == y_path: est ce que les deux sont des int ? sinon on peut pas comparer
'''