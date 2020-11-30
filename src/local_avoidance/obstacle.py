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

    def __init__(self, thymio: Thymio, final_occupancy_grid=0, position=0, interval_sleep=0.05, distance_avoidance=2.5,
                 angle_avoidance=5.0, square=2.5, wall_threshold=3000, clear_thresh=2400):
        self.thymio = thymio
        self.sensor_handler = SensorHandler(thymio)
        self.interval_sleep = interval_sleep
        self.distance_avoidance = distance_avoidance
        self.angle_avoidance = angle_avoidance
        self.final_occupancy_grid = final_occupancy_grid
        self.position = position
        self.square = square
        self.wall_threshold = wall_threshold
        self.clear_thresh = clear_thresh
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

        for i in range(3):
            self.__cote_avoid(rotated)

    def __turn_avoid(self, rotated):
        if rotated == EventEnum.LEFT:
            rotate(self.thymio, self.angle_avoidance)
        else:
            rotate(self.thymio, -self.angle_avoidance)

    def __cote_avoid(self,rotated):
        condition = True
        while condition:
            advance(self.thymio, self.distance_avoidance)

            if (rotated == EventEnum.LEFT):
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
                advance(self.thymio, 4*self.distance_avoidance)
                while sensor_values[4] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print(7)
                    rotate(self.thymio, -5)
                rotate(self.thymio, 20)
                break
            elif (rotated == EventEnum.RIGHT) and (sensor_values[0] < 2000):
                print(4)
                rotate(self.thymio, -30)
                advance(self.thymio, 4 * self.distance_avoidance)
                while sensor_values[0] < 1000:
                    sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                    print(7)
                    rotate(self.thymio, 5)
                rotate(self.thymio, -20)
                break



    def __check_global_obstacles(self):
        obstacle = False
        x = self.position[0]
        y = self.position[1]
        theta = self.position[2]

        #if(theta):

        #elif:


        if self.final_occupancy_grid[x][y] == 1:
            obstacle = True

        return math.sqrt((self.position[0] - x) * self.square ** 2 + (self.position[1] - y) * self.square ** 2)


    class ObstacleAvoidanceV2:

        def __init__(self, thymio: Thymio, final_occupancy_grid, position, interval_sleep=0.05, distance_avoidance=12.0,
                     angle_avoidance=5.0, square=2.5, wall_threshold=4000):
            self.thymio = thymio
            self.sensor_handler = SensorHandler(thymio)
            self.interval_sleep = interval_sleep
            self.distance_avoidance = distance_avoidance
            self.angle_avoidance = angle_avoidance
            self.final_occupancy_grid = final_occupancy_grid
            self.position = position
            self.square = square
            self.wall_threshold = wall_threshold
            stop(self.thymio)
            self.__obstacle_avoidance()

        def __obstacle_avoidance(self):
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            # print(sensor_values)
            rotated = None
            while np.amax(sensor_values).astype(int) >= 0:  # while still detects a wall, keep turning
                sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                if (sensor_values[1] > 0) and (sensor_values[3] > 0):  # both sides
                    angle = self.angle_avoidance * float(
                        np.sign(sensor_values[3] - sensor_values[1]))
                    rotate(self.thymio, angle, verbose=True)
                    if angle >= 0:
                        rotated = EventEnum.LEFT
                    else:
                        rotated = EventEnum.RIGHT
                elif (sensor_values[3] > 0) or (sensor_values[4] > 0):  # right side
                    rotate(self.thymio, angle=-self.angle_avoidance, verbose=True)
                    rotated = EventEnum.RIGHT
                elif (sensor_values[0] > 0) or (sensor_values[1] > 0):  # left side
                    rotate(self.thymio, angle=self.angle_avoidance, verbose=True)
                    rotated = EventEnum.LEFT
                elif sensor_values[2] > 0:  # center
                    print("rotate of ", self.angle_avoidance)
                    rotate(self.thymio, angle=self.angle_avoidance, verbose=True)
                    rotated = EventEnum.LEFT
                else:
                    break

            while True:  # check global obstacles
                if self.__check_global_obstacles() < self.distance_avoidance:
                    # if no obstacle in the way, starts moving forward
                    advance(self.thymio, distance=self.distance_avoidance, verbose=True)
                else:
                    # if obstacle in the way
                    rotate(self.thymio, angle=180.0, verbose=True)
                    self.__obstacle_avoidance()
                    return

                # after advancing, rotate of 90 degrees and verify the situation
                angle = 90.0
                if rotated == EventEnum.LEFT:
                    angle *= -1
                rotate(self.thymio, angle=angle, verbose=True)
                sensor_values = self.sensor_handler.sensor_raw()["sensor"]
                if sensor_values[2] == 0 and sensor_values[0] <= self.wall_threshold and sensor_values[
                    4] <= self.wall_threshold:
                    # if no local obstacle anymore, go forward and stop local avoidance
                    advance(self.thymio, distance=self.distance_avoidance, verbose=True)
                    return
                else:
                    # otherwise rotate back and continue
                    rotate(self.thymio, angle=-angle, verbose=True)

        def __check_global_obstacles(self):
            obstacle = False
            x = self.position[0]
            y = self.position[1]
            while not obstacle:
                value = math.cos(math.radians(self.position[2]))
                if math.fabs(value) >= 0.5:
                    x += int(math.copysign(1.0, value))

                value = math.sin(math.radians(self.position[2]))
                if math.fabs(value) >= 0.5:
                    y += int(math.copysign(1.0, value))

                if self.final_occupancy_grid[x][y] == 1:
                    obstacle = True

            return math.sqrt(
                (self.position[0] - x) * self.square ** 2 + (self.position[1] - y) * self.square ** 2)
