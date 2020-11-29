import time

import numpy as np
import os
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from threading import Timer

load_dotenv()


def move(thymio: Thymio, l_speed_ratio: int = 1, r_speed_ratio: int = 1, verbose: bool = False):
    """
    Move the robot's wheels correctly. Manages the negative speed well.
    Once this function is called the robot will continue forever if no further implementation is used.
    Wrap this function to set the conditions for moving.

    :param thymio:      the class to which the robot is referred to
    :param r_speed_ratio:     left speed
    :param l_speed_ratio:     right speed
    :param verbose:     printing the speed in the terminal
    """
    # Changing negative values to the expected ones with the bitwise complement
    l_speed = int(l_speed_ratio * int(os.getenv("LEFT_WHEEL_SCALING")))
    l_speed = l_speed if l_speed >= 0 else 2 ** 16 + l_speed
    r_speed = int(r_speed_ratio * int(os.getenv("RIGHT_WHEEL_SCALING")))
    r_speed = r_speed if r_speed >= 0 else 2 ** 16 + r_speed
    thymio.set_var("motor.left.target", l_speed)
    thymio.set_var("motor.right.target", r_speed)

    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed)


def stop(thymio: Thymio, verbose=False):
    """
    Stop the robot.

    :param thymio:      the class to which the robot is referred to
    :param verbose:     printing the stop command in the terminal
    """
    # Printing the speeds if requested
    if verbose:
        print("\t\t Stopping")

    thymio.set_var("motor.left.target", 0)
    thymio.set_var("motor.right.target", 0)


def rotate(thymio: Thymio, angle: float, verbose: bool = False):
    """
    Rotates of the desired angle

    :param thymio:      the class to which the robot is referred to
    :param angle:       angle in radians by which we want to rotate, positive or negative
    :param verbose:     printing the speed in the terminal
    :return: timer to check if it is still alive or not
    """
    l_speed = -int(np.sign(angle))
    r_speed = int(np.sign(angle))
    turn_time = float(os.getenv("HALF_TURN_TIME")) * angle / 180.0

    # Printing the speeds if requested
    if verbose:
        print("\t\t Rotate speed & time : ", l_speed, r_speed, turn_time)

    move(thymio, l_speed, r_speed)
    time.sleep(turn_time)
    stop(thymio)


def advance(thymio: Thymio, distance: float, speed_ratio: int = 1, verbose: bool = False):
    """
    Moves straight of a desired distance

    :param kwargs:      function to execute at the end of advancing, default stop
    :param args:        array of non-keyworded arguments of function
    :param function:    set of keyworded arguments
    :param thymio:      the class to which the robot is referred to
    :param distance:    distance in cm by which we want to move, positive or negative
    :param speed_ratio:       the speed factor at which the robot goes
    :param verbose:     printing the speed in the terminal
    :return: timer to check if it is still alive or not
    """
    l_speed, r_speed, distance_time = advance_time(distance, speed_ratio)
    # Printing the speeds if requested
    if verbose:
        print("\t\t Advance speed & time : ", l_speed, r_speed, distance_time)

    move(thymio, l_speed, r_speed)
    time.sleep(distance_time)
    stop(thymio)


def advance_time(distance: float, speed_ratio: int = 1):
    l_speed = speed_ratio * int(np.sign(distance))
    r_speed = speed_ratio * int(np.sign(distance))
    distance_time = float(os.getenv("DISTANCE_TIME")) * distance / speed_ratio
    return l_speed, r_speed, distance_time
