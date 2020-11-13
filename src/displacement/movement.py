import numpy as np
from src.thymio.Thymio import Thymio


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
    l_speed = l_speed if l_speed >= 0 else 2 ** 16 + l_speed
    r_speed = r_speed if r_speed >= 0 else 2 ** 16 + r_speed

    # Setting the motor speeds
    thymio.set_var("motor.left.target", l_speed)
    thymio.set_var("motor.right.target", r_speed)


def stop(thymio: Thymio, verbose=False):
    """
    Stop the robot.

    :param thymio:      the class to which the robot is referred to
    :param verbose:     printing the speed in the terminal
    """

    # Printing the speeds if requested
    if verbose:
        print("\t\t Stopping")

    # Setting the motor speeds
    thymio.set_var("motor.left.target", 0)
    thymio.set_var("motor.right.target", 0)


def rotate(angle: float, coords):
    """
    Rotates the coordinates of a matrix by the desired angle

    :param coords:      numpy array of coordinates
    :param angle:       angle in radians by which we want to rotate
    :return: numpy.array() that contains rotated coordinates
    """

    r = np.array(((np.cos(angle), -np.sin(angle)),
                  (np.sin(angle), np.cos(angle))))

    return r.dot(coords.transpose()).transpose()
