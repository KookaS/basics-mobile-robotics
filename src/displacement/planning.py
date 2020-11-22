import time
from enum import Enum

import numpy as np

from src.displacement.movement import rotate, advance
from src.thymio.Thymio import Thymio


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    RIGHT = 0
    LEFT = 1
    STRAIGHT = 2


def path_to_command_thymio(path):
    current_x = path[0][0]
    current_y = path[1][0]
    next_x = path[0][1]
    next_y = path[1][1]

    # next-prev
    delta_x = next_x - current_x
    delta_y = next_y - current_y

    # delta_x = 0 and delta_y = -/+ 1 (or delta_x = -/+ 1 and delta_y = 0): go straight
    turn = EventEnum.STRAIGHT

    # delta_x = -1 and delta_y = 1 (or delta_x = 1 and delta_y = -1): turn to the right
    if delta_x * delta_y < 0:
        turn = EventEnum.RIGHT

    # delta_x = -1 and delta_y = -1 (or delta_x = 1 and delta_y = 1): turn to the left
    if delta_x * delta_y == 1:
        turn = EventEnum.LEFT

    new_path = np.array([path[0][1:], path[1][1:]])

    return turn, new_path


# this code gives a sequence of RIGHT; LEFT; STRAIGHT commands corresponding to the entire global path planning
# i.e to go from the start to the goal

def update_path(thymio: Thymio, path):
    new_path = path
    for i in range(len(path[0]) - 1):

        turn, new_path = path_to_command_thymio(new_path)

        if turn == EventEnum.RIGHT:
            # turn 45 degrees to the right
            timer = rotate(thymio, -45.0)
            while timer.is_alive():
                time.sleep(0.2)

            # move forward by the length of the diagonal of a square : 3.4cm
            timer = advance(thymio, 3.4)
            while timer.is_alive():
                time.sleep(0.2)

        if turn == EventEnum.LEFT:
            # turn 45 degrees to the left
            thread = rotate(thymio, 45.0)
            while thread.is_alive():
                time.sleep(0.2)

            # move forward by the length of the diagonal of a square : 3.4cm
            timer = advance(thymio, 3.4)
            while timer.is_alive():
                time.sleep(0.2)

        if turn == EventEnum.STRAIGHT:
            # move forward by the length of the side of a square : 2.5cm
            timer = advance(thymio, 2.5)
            while timer.is_alive():
                time.sleep(0.2)

        print(turn.name)
