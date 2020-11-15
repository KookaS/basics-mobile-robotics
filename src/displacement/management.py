import numpy as np
import time

from src.thymio.Thymio import Thymio
from src.displacement.movement import move, stop, rotate


def run_ann_without_memory(thymio: Thymio):
    """
    Run the thymio with the NN architecture.

    :param thymio: The file location of the spreadsheet
    """

    # Weights of neuron inputs
    w_l = np.array([40, 20, -20, -20, -40, 30, -10])
    w_r = np.array([-40, -20, -20, 20, 40, -10, 30])

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    # constant_scale = 20

    # State for start and stop
    state = 0

    # x = np.zeros(shape=(7,))
    y = np.zeros(shape=(2,))

    j = 0
    while True:
        j += 1
        if thymio["button.center"] == 1 and state == 0:
            state = 1
            print("moving!")
            move(thymio)
            time.sleep(0.1)
        elif thymio["button.left"] == 1 and state == 0:
            state = 1
            print("Rotate left!")
            rotate(thymio, -90.0)
            time.sleep(0.1)
        elif thymio["button.right"] == 1 and state == 0:
            state = 1
            print("Rotate right!")
            rotate(thymio, 90.0)
            time.sleep(0.1)
        elif thymio["button.center"] == 1 and state == 1:
            state = 0
            print("Stopping!")
            stop(thymio)
            time.sleep(0.1)

        """
        if state != 0:
            # sensing and avoiding obstacles with ann
            print("inside ann")
            # Get and scale inputs
            x = np.array(thymio["prox.horizontal"]) / sensor_scale
            # Compute outputs of neurons and set motor powers
            y[0] = np.sum(x * w_l)
            y[1] = np.sum(x * w_r)
            print(j, int(y[0]), int(y[1]), thymio["prox.horizontal"])
            move(thymio, l_speed=int(y[0]), r_speed=int(y[1]))
        """
