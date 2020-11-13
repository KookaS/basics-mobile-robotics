import os
import sys
import pprint
import serial
import time
import numpy as np
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv

load_dotenv()

# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))


def print_thymio(thymio: Thymio):
    print('All Thymio instance attributes:')
    pprint.pprint(dir(thymio))

    # see what the different read-write variables that you can access are
    variables = thymio.variable_description()
    print('\nVariables of Thymio:')
    for var in variables:
        print(var)


def move(thymio: Thymio, l_speed: int = 500, r_speed: int = 500, verbose: bool = False):
    """
    Sets the motor speeds of the Thymio
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
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
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose:
        print("\t\t Stopping")

    # Setting the motor speeds
    thymio.set_var("motor.left.target", 0)
    thymio.set_var("motor.right.target", 0)


def run_ann_without_memory(thymio: Thymio):
    # Weights of neuron inputs
    w_l = np.array([40, 20, -20, -20, -40, 30, -10])
    w_r = np.array([-40, -20, -20, 20, 40, -10, 30])

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    constant_scale = 20

    # State for start and stop
    state = 0

    x = np.zeros(shape=(7,))
    y = np.zeros(shape=(2,))

    j = 0
    while True:
        j += 1

        if thymio["button.center"] == 1 and state == 0:
            state = 1
            move(thymio)
            print("moving!")
            time.sleep(0.1)
        elif thymio["button.center"] == 1 and state == 1:
            state = 0
            stop(thymio)
            print("Stopping!")
            time.sleep(0.1)

        if state != 0:
            # Get and scale inputs
            x = np.array(thymio["prox.horizontal"]) / sensor_scale

            # Compute outputs of neurons and set motor powers
            y[0] = np.sum(x * w_l)
            y[1] = np.sum(x * w_r)

            print(j, int(y[0]), int(y[1]), thymio["prox.horizontal"])
            move(thymio, l_speed=int(y[0]), r_speed=int(y[1]))


def main():
    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)  # ser:device=\\.\COM4
    time.sleep(3)  # To make sure the Thymio has had time to connect
    print_thymio(th)
    run_ann_without_memory(th)


if __name__ == "__main__":
    main()
