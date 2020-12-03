import os
import sys
import pprint
import time
import numpy as np
from src.displacement.management import EventHandler
from src.sensors.tuning import MotionTuning, VelocityTuning
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from src.local_avoidance.obstacle import test_saw_wall, ObstacleAvoidance


# Adding the src folder in the current directory as it contains the script with the Thymio class
from src.vision.camera import test_camera, record_project, camera_tweak

sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
load_dotenv()
LOW_BLUE = np.array([98, 134, 106])  # à changer dans management
UP_BLUE = np.array([109, 225, 174])  # à changer dans management

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
    # camera_tweak(LOW_BLUE, UP_BLUE)
    # time.sleep(30)

    """
    while True:
        temp = record_project()
        print(temp)
    """
    print("JE SUIS DANS LE MAIN")
    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)
    time.sleep(3)  # To make sure the Thymio has had time to connect
    position = (10,10)

    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 100)
    c=False
    while(not c):
        c=test_saw_wall(th)
    print("On commence a éviter", c)
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)
    ob = ObstacleAvoidance(th)
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)
    while (True):
        print("end")

    # VelocityTuning(th)
    # MotionTuning(thymio=th, distance=15, angle=180.0)
    EventHandler(th, interval_check=1, interval_sleep=0.05,
                 goal_threshold=1)  # check every interval_check seconds to change scenarios

    print("END OF MAIN!")


if __name__ == "__main__":
    main()
