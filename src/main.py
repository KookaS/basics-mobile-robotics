import os
import sys
import pprint
import time
import numpy as np
from src.displacement.management import EventHandler
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from src.vision.camera import Camera

# Adding the src folder in the current directory as it contains the script with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
load_dotenv()


def print_thymio(thymio: Thymio):
    """
    Print the variables of Thymio

    :param thymio: The file location of the spreadsheet
    """
    print('All Thymio instance attributes:')
    pprint.pprint(dir(thymio))
    variables = thymio.variable_description()  # see what the different read-write variables that you can access are
    print('\nVariables of Thymio:')
    for var in variables:
        print(var)


def main():
    """

    :return:
    """

    """
    while True:
        print(Camera().record_project())
    """

    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)
    time.sleep(3)  # To make sure the Thymio has had time to connect
    """
        position = (10,10)
        th.set_var("motor.left.target", 100)
        th.set_var("motor.right.target", 100)
        c=False
        while(not c):
            c=test_saw_wall(th)
    """
    # VelocityTuning(th)
    # MotionTuning(thymio=th, distance=15, angle=180.0)
    EventHandler(th)  # check every interval_check seconds to change scenarios

    print("END OF MAIN!")


if __name__ == "__main__":
    main()
