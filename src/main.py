import os
import sys
import pprint
import time

from src.displacement.management import EventHandler
from src.sensors.tuning import MotionTuning
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
    test_camera()
    camera_tweak()
    time.sleep(30)

    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)
    time.sleep(3)  # To make sure the Thymio has had time to connect

    # MotionTuning(thymio=th, distance=15, angle=180.0)
    EventHandler(th, interval_check=0.1, interval_sleep=0.1)  # check every interval_check seconds to change scenarios
    # record_project()

    print("END OF MAIN!")


if __name__ == "__main__":
    main()
