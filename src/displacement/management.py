import threading
import time
from enum import Enum
from src.thymio.Thymio import Thymio
from src.displacement.movement import move, stop, rotate


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    GLOBAL = 0
    LOCAL = 1
    STOP = 2


class EventHandler:
    def __init__(self, thymio: Thymio, interval_check, interval_sleep=0.5):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.events = []
        for _ in EventEnum:
            self.events.append(threading.Event())
        threading.Thread(target=self.__global_thread_init).start()
        threading.Thread(target=self.__local_thread_init).start()
        threading.Thread(target=self.__stop_thread_init).start()
        print("after starting all threads!")
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        threading.Timer(self.interval_check, self.__check_handler).start()  # will check every time the conditions
        self.state = EventEnum.STOP.value
        self.events[EventEnum.STOP.value].set()

    def __global_thread_init(self):
        """
        Initialize the thread for GLOBAL scenario, then pause it until next call.
        """
        print("Waiting on global_thread")
        self.events[EventEnum.GLOBAL.value].wait()
        print("global_thread is fired")
        self.__global_handler()

    def __local_thread_init(self):
        """
        Initialize the thread for LOCAL scenario, then pause it until next call.
        """
        print("Waiting on local_thread")
        self.events[EventEnum.LOCAL.value].wait()
        print("local_thread is fired")
        self.__local_handler()

    def __stop_thread_init(self):
        """
        Initialize the thread for STOP scenario, then pause it until next call.
        """
        print("Waiting on stop_thread")
        self.events[EventEnum.STOP.value].wait()
        print("stop_thread is fired")
        self.__stop_handler()

    def __check_handler(self):
        """
        Handles the different scenarios in which the robot will be.
        This function is called on it's own thread every interval_check seconds.

        Threading:
        There is a thread has a flag to restrict the use of some of its methods.
        When .start() the flag stays false, but thread runs.
        When .wait() if the flag is false then pause the thread.
        When .clear() if the flag is true changes it to false.
        When .set() if the flag is false and thead paused, run the thread and flag to true.
        """
        threading.Timer(5.0, self.__check_handler).start()  # will check every time the conditions
        print("check_conditions")

        sensor = 2000  # that's just random for testing
        if self.state != EventEnum.GLOBAL.value and sensor > 1000:  # condition to go into global path planning
            print("changing to GLOBAL!!")
            print(self.events[self.state])
            self.events[self.state].clear()  # set the thread flag to false
            self.events[self.state].wait()  # if thread flag is false, then pause the thread
            self.state = EventEnum.GLOBAL.value
            self.events[EventEnum.GLOBAL.value].set()  # run a thread, flag from false to true

        elif self.state != EventEnum.LOCAL.value and sensor > 3000:  # condition to go into local avoidance
            print("changing to LOCAL!!")
            self.events[self.state].clear()
            self.events[self.state].wait()
            self.state = EventEnum.LOCAL.value
            self.events[EventEnum.LOCAL.value].set()

        elif self.state != EventEnum.STOP.value and sensor > 3000:  # condition to go into stop
            print("changing to STOP!!")
            self.events[self.state].clear()
            self.events[self.state].wait()
            self.state = EventEnum.STOP.value
            self.events[EventEnum.STOP.value].set()

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        print("inside global_handler")
        # code here
        threading.Timer(self.interval_sleep, self.__global_handler).start()

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        print("inside local_handler")
        # code here
        threading.Timer(self.interval_sleep, self.__local_handler).start()

    def __stop_handler(self):
        """
        Manages the thread for the STOP scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        print("inside stop_handler")
        # code here
        threading.Timer(self.interval_sleep, self.__stop_handler).start()


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
