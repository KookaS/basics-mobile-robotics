import time
from threading import Timer

from src.thymio.Thymio import Thymio


class RepeatedTimer(object):
    """
        Timer used to get the evolution of the state of the robot

        :attribute _timer:          Timer used internally
        :attribute interval:        Interval of time
        :attribute function:        Callback function
        :attribute args:            non-keyworded variable length argument list to the function
        :attribute kwargs:          keyworded variable length of arguments to a function
        :attribute is_running:      tells if the timer runs or not
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)  # TODO: problem here with this callback

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


class State(object):
    """
    Get the data from every sensors

    :attribute ts:          the time interval
    :attribute data:        ground, sensor, left_speed and right_speed at every sample
    :attribute thymio:      class of the robot to refer to
    """

    def __init__(self, thymio: Thymio, ts: float = 0.1):
        self.ts = ts
        self.data = []
        self.thymio = thymio

    def __get_data(self):
        """
        Get the data from every sensors

        :param thymio: The file location of the spreadsheet
        """
        self.data.append({"ground": self.thymio["prox.ground.reflected"],
                          "sensor": self.thymio["prox.ground.reflected"],
                          "left_speed": self.thymio["motor.left.speed"],
                          "right_speed": self.thymio["motor.right.speed"]})

    def acquire_data(self):
        """
        Fetch the robot state
        """
        rt = RepeatedTimer(self.ts, self.__get_data())  # it auto-starts, no need of rt.start()
        try:
            time.sleep(5)
            self.thymio.set_var("motor.left.target", 55)
            self.thymio.set_var("motor.right.target", 50)
            time.sleep(25)  # your long-running job goes here...
        finally:
            rt.stop()  # better in a try/finally block to make sure the program ends!
            self.thymio.set_var("motor.left.target", 0)
            self.thymio.set_var("motor.right.target", 0)

    def data(self):
        """
        Fetch the data of thymio

        :return: return the data of thymio of the time interval
        """
        return self.data

    def ts(self) -> float:
        """
        Fetch the time interval

        :return: return the time interval
        """
        return self.ts
