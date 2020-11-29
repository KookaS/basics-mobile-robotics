import os
import sys
import pprint
import time

from src.displacement.management import EventHandler
from src.sensors.tuning import MotionTuning, VelocityTuning
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv

# Adding the src folder in the current directory as it contains the script with the Thymio class
from src.vision.camera import test_camera, record_project, camera_tweak

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
    # test_camera() # Position not accurate!!!
    # camera_tweak()
    # time.sleep(30)

    """
    while True:
        temp = record_project()
        print(temp)
    """

    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)
    time.sleep(3)  # To make sure the Thymio has had time to connect

    # VelocityTuning(th)
    # MotionTuning(thymio=th, distance=15, angle=180.0)
    EventHandler(th, interval_check=1, interval_sleep=0.05,
                 goal_threshold=1)  # check every interval_check seconds to change scenarios

    print("END OF MAIN!")


if __name__ == "__main__":
    main()
