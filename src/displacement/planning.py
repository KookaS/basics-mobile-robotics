import numpy as np

from src.thymio.Thymio import Thymio

RIGHT = 0
LEFT = 1
STRAIGHT = 2


def path_to_command_thymio(path):
    current_x = path[0][0]
    current_y = path[1][0]

    next_x = path[0][1]
    next_y = path[1][1]

    # next-prev
    delta_x = path[0][1] - path[0][0]
    delta_y = path[1][1] - path[1][0]

    # delat_x = 0 and delta_y = -/+ 1 (or delat_x = -/+ 1 and delta_y = 0): go straight
    turn = STRAIGHT

    # delat_x = -1 and delta_y = 1 (or delat_x = 1 and delta_y = -1): turn to the right
    if delta_x * delta_y < 0:
        turn = RIGHT

    # delat_x = -1 and delta_y = -1 (or delat_x = 1 and delta_y = 1): turn to the left
    if delta_x * delta_y == 1:
        turn = LEFT

    new_path = np.array([path[0][1:], path[1][1:]])

    return turn, new_path


# this code gives a sequence of RIGHT; LEFT; STRAIGHT commands corresponding to the entire global path planning
# i.e to go from the start to the goal

def update_path(thymio: Thymio, path):
    new_path = path
    for i in range(len(path[0]) - 1):

        turn, new_path = path_to_command_thymio(new_path)

        if turn == RIGHT:
            # turn 45 degrees to the right
            # --------------------------------------
            #           ADD CODE HERE
            # --------------------------------------
            #
            timer = 0

            # move forward by the length of the diagonal of a square : 3cm?
            # --------------------------------------
            #           ADD CODE HERE
            # --------------------------------------

        if turn == LEFT:
            # turn 45 degrees to the left
            # --------------------------------------
            #           ADD CODE HERE
            # --------------------------------------
            #
            # move forward by the length of the diagonal of a square : 3cm?
            # --------------------------------------
            #           ADD CODE HERE
            # --------------------------------------
            timer = 0

        if turn == STRAIGHT:
            # move forward by the length of the side of a square : 2cm?
            # --------------------------------------
            #           ADD CODE HERE
            # --------------------------------------
            timer = 0

        print(turn)
