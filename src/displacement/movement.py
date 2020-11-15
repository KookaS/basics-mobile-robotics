import numpy as np
import os
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from threading import Timer

load_dotenv()


def move(thymio: Thymio, l_speed: int = 500, r_speed: int = 500, verbose: bool = False):
    """
    Move the robot's wheels.

    :param thymio:      the class to which the robot is referred to
    :param r_speed:     left speed
    :param l_speed:     right speed
    :param verbose:     printing the speed in the terminal
    """

    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed)

    # Changing negative values to the expected ones with the bitwise complement
    l_speed = int(l_speed * int(os.getenv("LEFT_WHEEL_SCALING")) / 100)
    l_speed = l_speed if l_speed >= 0 else 2 ** 16 + l_speed
    r_speed = int(r_speed * int(os.getenv("RIGHT_WHEEL_SCALING")) / 100)
    r_speed = r_speed if r_speed >= 0 else 2 ** 16 + r_speed
    thymio.set_var("motor.left.target", l_speed)
    thymio.set_var("motor.right.target", r_speed)


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
    Rotates the coordinates of a matrix by the desired angle

    :param thymio:      the class to which the robot is referred to
    :param angle:       angle in radians by which we want to rotate
    :param verbose:     printing the speed in the terminal
    """

    l_speed = 100 * int(np.sign(angle))
    r_speed = -100 * int(np.sign(angle))
    turn_time = float(os.getenv("ONE_TURN_TIME")) * angle / 180.0

    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed, turn_time)

    timer = Timer(interval=turn_time, function=move, args=[thymio, 0, 0])
    move(thymio, l_speed, r_speed)
    timer.run()
    timer.cancel()  # stop the timer's action if it's still waiting
