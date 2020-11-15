import threading
import time
from enum import Enum
from src.thymio.Thymio import Thymio
from src.displacement.movement import move, stop, rotate


class EventEnum(Enum):
    GLOBAL = 0
    LOCAL = 1
    STOP = 2


class EventHandler:
    def __init__(self, thymio: Thymio, interval_check):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.events = []
        for _ in EventEnum:
            self.events.append(threading.Event())
        print(self.events)
        threading.Thread(target=self.global_thread).start()
        threading.Thread(target=self.local_thread).start()
        threading.Thread(target=self.stop_thread).start()
        print("after starting all threads!")
        self.__check_thread()

    def __check_thread(self):
        threading.Timer(self.interval_check, self.__check_handler).start()  # will check every time the conditions
        self.state = EventEnum.STOP.value
        self.events[EventEnum.STOP.value].set()

    def __check_handler(self):
        """
        Handles the different cases in which the robot will be.
        There is a thread per scenario, .set() activates the code after the .wait() in the function
        """
        threading.Timer(5.0, self.__check_handler).start()  # will check every time the conditions
        print("check_conditions")

        sensor = 2000  # that's just random for testing
        if self.state != EventEnum.GLOBAL.value and sensor > 1000:  # condition to go into global path planning
            print("changing to GLOBAL!!")
            print(self.events[self.state])
            self.events[self.state].wait()
            self.state = EventEnum.GLOBAL.value
            self.events[EventEnum.GLOBAL.value].set()

        elif self.state != EventEnum.LOCAL.value and sensor > 3000:  # condition to go into local avoidance
            print("changing to LOCAL!!")
            self.events[self.state].wait()
            self.state = EventEnum.LOCAL.value
            self.events[EventEnum.LOCAL.value].set()

        elif self.state != EventEnum.STOP.value and sensor > 3000:  # condition to go into stop
            print("changing to STOP!!")
            self.events[self.state].wait()
            self.state = EventEnum.STOP.value
            self.events[EventEnum.STOP.value].set()

    def global_thread(self):
        print("Waiting on global_thread")
        self.events[EventEnum.GLOBAL.value].wait()
        print("global_thread is fired")
        self.global_handler()

    def local_thread(self):
        print("Waiting on local_thread")
        self.events[EventEnum.LOCAL.value].wait()
        print("local_thread is fired")
        self.local_handler()

    def stop_thread(self):
        print("Waiting on stop_thread")
        self.events[EventEnum.STOP.value].wait()
        print("stop_thread is fired")
        self.stop_handler()

    def global_handler(self):
        print("inside global_handler")
        # code here
        # self.events[EventEnum.GLOBAL.value].wait(20000)
        time.sleep(0.5)
        self.global_handler()

    def local_handler(self):
        print("inside local_handler")
        # code here
        # self.events[EventEnum.LOCAL.value].wait(20000)
        time.sleep(0.5)
        self.global_handler()

    def stop_handler(self):
        print("inside stop_handler")
        # code here
        # self.events[EventEnum.STOP.value].wait(2000)
        time.sleep(0.5)
        self.stop_handler()


def ann_test(thymio: Thymio):
    """
    Run the thymio with the NN architecture.
    Forget about that one now, it's just a random test

    :param thymio: The file location of the spreadsheet
    """
    if thymio["button.center"] == 1:
        print("moving!")
        move(thymio)
        time.sleep(0.1)
    elif thymio["button.left"] == 1:
        print("Rotate left!")
        rotate(thymio, -90.0)
        time.sleep(0.1)
    elif thymio["button.right"] == 1:
        print("Rotate right!")
        rotate(thymio, 90.0)
        time.sleep(0.1)
    elif thymio["button.center"] == 1:
        print("Stopping!")
        stop(thymio)
        time.sleep(0.1)
    """
    w_l = np.array([40, 20, -20, -20, -40, 30, -10])     # Weights of neuron inputs
    w_r = np.array([-40, -20, -20, 20, 40, -10, 30])
    sensor_scale = 200  # Scale factors for sensors and constant factor
    # constant_scale = 20
    # x = np.zeros(shape=(7,))
    y = np.zeros(shape=(2,))
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
