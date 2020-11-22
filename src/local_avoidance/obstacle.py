import math
import threading
import time
from enum import Enum
from typing import Tuple

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
                 angle_avoidance=5.0):
        self.thymio = thymio
        self.sensor_handler = SensorHandler(thymio)
        self.interval_sleep = interval_sleep
        self.distance_avoidance = distance_avoidance
        self.angle_avoidance = angle_avoidance
        self.final_occupancy_grid = final_occupancy_grid
        self.position = position
        self.thread = threading.Timer(interval=interval_sleep, function=stop)
        stop(self.thymio)
        self.__obstacle_avoidance()

    def __obstacle_avoidance(self):
        sensor_values = self.sensor_handler.sensor_raw()["sensor"]
        # print(sensor_values)

        while np.amax(sensor_values).astype(int) >= 0:  # while still detects a wall, keep turning
            sensor_values = self.sensor_handler.sensor_raw()["sensor"]
            # print(sensor_values)
            if not self.thread.is_alive():
                if (sensor_values[1] > 0) and (sensor_values[3] > 0):  # both sides
                    angle = self.angle_avoidance * float(
                        np.sign(sensor_values[3] - sensor_values[1]))
                    self.thread = rotate(self.thymio, angle)
                    if angle >= 0:
                        rotated = EventEnum.LEFT
                    else:
                        rotated = EventEnum.RIGHT
                elif (sensor_values[3] > 0) or (sensor_values[4] > 0):  # right side
                    self.thread = rotate(self.thymio, angle=-self.angle_avoidance)
                    rotated = EventEnum.RIGHT
                elif (sensor_values[0] > 0) or (sensor_values[1] > 0):  # left side
                    self.thread = rotate(self.thymio, angle=self.angle_avoidance)
                    rotated = EventEnum.LEFT
                elif sensor_values[2] > 0:  # center
                    self.thread = rotate(self.thymio, angle=self.angle_avoidance)
                    rotated = EventEnum.LEFT
                else:
                    break
            time.sleep(self.interval_sleep)

        # after rotating, starts moving forward
        self.thread = advance(self.thymio, distance=self.distance_avoidance)
        while self.thread.is_alive():
            time.sleep(self.interval_sleep)

        # after moving forward, stop the robot
        stop(self.thymio)
        return
