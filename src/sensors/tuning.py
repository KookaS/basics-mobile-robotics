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
        self.thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.distance = distance
        self.angle = angle
        self.timer_advance = Timer(interval=interval_sleep, function=stop)
        self.timer_rotate = Timer(interval=interval_sleep, function=stop)
        self.__tune_handler()

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


class SensorTuning:
    """

    """

    def __init__(self, thymio: Thymio):
        self.thymio = thymio
