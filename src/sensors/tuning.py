import threading

from src.displacement.movement import advance, rotate, stop
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv
from threading import Timer

load_dotenv()


class MotionTuning:
    """
    Tune the robot to go in a straight line.
    Be careful, make it run for more than one lien because apparently there are some error initially.

    Example:
    InitTuning(thymio=th, distance=15.0, angle=180.0)
    """

    def __init__(self, thymio: Thymio, interval_check=0.1, interval_sleep=0.1, distance=15.0, angle=180.0):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.distance = distance
        self.angle = angle
        self.tune_thread = threading.Event()
        self.timer_advance = Timer(interval=interval_sleep, function=stop)
        self.timer_rotate = Timer(interval=interval_sleep, function=stop)
        self.timer_check = Timer(interval=self.interval_check, function=self.__check_handler)
        threading.Thread(target=self.__tune_thread_init).start()
        print("after starting all threads!")
        self.__check_thread_init()

    def __forward(self):
        """
        Make the robot go forward
        """
        print("Forward button pressed!")
        self.timer_check.start()
        self.tune_thread.set()  # run a thread, flag from false to true

    def __center(self):
        """
        Make the robot stop

        To remove a thread .set() & .join()
        To remove a timer .cancel() & .join()
        TODO: Still has problems if you press the button in the center
        """
        print("Center Button pressed!")
        self.timer_check.run()
        # print(threading.active_count())
        if self.timer_advance.is_alive():
            self.timer_advance.cancel()
            self.timer_advance.join()
        if self.timer_rotate.is_alive():
            self.timer_rotate.cancel()
            self.timer_advance.join()
        if self.tune_thread.is_set():
            self.tune_thread.clear()  # set the thread flag to false
            self.tune_thread.wait()  # if thread flag is false, then pause the thread

    def __check_thread_init(self):
        threading.Timer(self.interval_check, self.__check_handler).start()  # will check every time the conditions
        self.tune_thread.set()

    def __tune_thread_init(self):
        print("Waiting on thread")
        self.tune_thread.wait()
        print("thread is fired")
        self.__tune_handler()

    def __check_handler(self):
        if self.thymio["button.center"] == 1:
            self.__center()
        elif self.thymio["button.forward"] == 1:
            self.__forward()
        else:
            self.timer_check.run()

    def __tune_handler(self):
        # print(threading.active_count())
        if not self.timer_rotate.is_alive() and not self.timer_advance.is_alive():
            stop(self.thymio, verbose=True)
            self.timer_advance = advance(thymio=self.thymio, distance=self.distance, verbose=True,
                                         function=self.__rot_handler, args=[self.thymio])
        else:
            Timer(interval=self.interval_sleep, function=self.__tune_handler).start()

    def __rot_handler(self, thymio: Thymio):
        stop(self.thymio, verbose=True)
        self.timer_rotate = rotate(thymio=thymio, angle=self.angle, verbose=True)
        Timer(interval=self.interval_sleep, function=self.__tune_handler).start()

# class SensorTuning:
# ADD STUFF TO
