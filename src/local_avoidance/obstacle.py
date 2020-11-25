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

    def __init__(self, thymio: Thymio, final_occupancy_grid, position, interval_sleep=0.2, distance_avoidance=12.0,
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
        self.thread = threading.Timer(interval=interval_sleep, function=stop)
        stop(self.thymio)
        self.__obstacle_avoidance()

    def __obstacle_avoidance(self):
        sensor_values = self.sensor_handler.sensor_raw()["sensor"]
        # print(sensor_values)
        rotated = None
        while np.amax(sensor_values).astype(int) >= 0:  # while still detects a wall, keep turning
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            # print(sensor_values)
            if (sensor_values[1] > 0) and (sensor_values[3] > 0):  # both sides
                angle = self.angle_avoidance * float(
                    np.sign(sensor_values[3] - sensor_values[1]))
                print("rotate of ", angle)
                self.thread = rotate(self.thymio, angle)
                if angle >= 0:
                    rotated = EventEnum.LEFT
                else:
                    rotated = EventEnum.RIGHT
            elif (sensor_values[3] > 0) or (sensor_values[4] > 0):  # right side
                print("rotate of ", -self.angle_avoidance)
                self.thread = rotate(self.thymio, angle=-self.angle_avoidance)
                rotated = EventEnum.RIGHT
            elif (sensor_values[0] > 0) or (sensor_values[1] > 0):  # left side
                print("rotate of ", self.angle_avoidance)
                self.thread = rotate(self.thymio, angle=self.angle_avoidance)
                rotated = EventEnum.LEFT
            elif sensor_values[2] > 0:  # center
                print("rotate of ", self.angle_avoidance)
                self.thread = rotate(self.thymio, angle=self.angle_avoidance)
                rotated = EventEnum.LEFT
            else:
                break

            while self.thread.is_alive():
                time.sleep(self.interval_sleep)

        while True:
            if self.__check_global_obstacles() < self.distance_avoidance:
                # if no obstacle in the way, starts moving forward
                print("forward of ", self.distance_avoidance)
                self.thread = advance(self.thymio, distance=self.distance_avoidance)
                while self.thread.is_alive():
                    time.sleep(self.interval_sleep)
            else:
                # if obstacle in the way
                print("rotate of ", 180.0)
                self.thread = rotate(self.thymio, angle=180.0)
                while self.thread.is_alive():
                    time.sleep(self.interval_sleep)
                self.__obstacle_avoidance()
                return

            # after advancing rotate of 90 degrees and verify the situation
            angle = 90.0
            if rotated == EventEnum.LEFT:
                angle *= -1
            print("rotate of ", angle)
            self.thread = rotate(self.thymio, angle=angle)
            while self.thread.is_alive():
                time.sleep(self.interval_sleep)
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            if sensor_values[2] == 0 and sensor_values[0] <= self.wall_threshold and sensor_values[
                4] <= self.wall_threshold:
                # if no local obstacle anymore, go forward and stop local avoidance
                print("forward of ", self.distance_avoidance)
                self.thread = advance(self.thymio, distance=self.distance_avoidance)
                while self.thread.is_alive():
                    time.sleep(self.interval_sleep)
                return
            else:
                # otherwise rotate back and continue
                print("rotate of ", -angle)
                self.thread = rotate(self.thymio, angle=-angle)
                while self.thread.is_alive():
                    time.sleep(self.interval_sleep)

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
