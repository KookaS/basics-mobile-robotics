import os
import sys
import pprint
import time
import numpy as np
from src.displacement.management import EventHandler
from src.displacement.movement import move, stop
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from src.vision.camera import Camera
from src.local_avoidance.obstacle import test_saw_wall, ObstacleAvoidance


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
    cam = Camera()
    cam.open_camera()
    while True:
        print(cam.test_camera())
    """
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
    EventHandler(th)  # check every interval_check seconds to change scenarios

    """
    sensor_handler = SensorHandler(th)
    move(th, 1, -1)
    print("l_speed, r_speed")
    now = time.time()
    while time.time()-now < 10:
        speed = sensor_handler.speed()
        speed_right = speed['right_speed']
        speed_left = speed['left_speed']
        speed_right = speed_right if speed_right <= 2 ** 15 else speed_right - 2 ** 16
        speed_left = speed_left if speed_left <= 2 ** 15 else speed_left - 2 ** 16
        print(speed_left, speed_right)
    stop(th)
    """

    print("END OF MAIN!")


if __name__ == "__main__":
    main()
