import threading
import time

import cv2
import numpy as np
from enum import Enum

from src.displacement.movement import stop
from src.kalman.kalmann_filter import kalman_filter
from src.local_avoidance.obstacle import ObstacleAvoidance
from src.path_planning.occupancy import create_grid
from src.sensors.state import SensorHandler
from src.thymio.Thymio import Thymio
from src.vision.camera import record_project


class EventEnum(Enum):
    """
    This is a class based on enumeration to define constants in a clean way.
    """
    GLOBAL = 0
    LOCAL = 1
    STOP = 2
    KALMAN = 3
    CAMERA = 4


class EventHandler:
    """
    Example:
            EventHandler(thymio=th, interval_check=5)
    """

    def __init__(self, thymio: Thymio, interval_check=0.2, interval_sleep=0.1, obstacle_threshold=2000,
                 stop_threshold=3500):
        self.thymio: Thymio = thymio
        self.interval_check = interval_check
        self.interval_sleep = interval_sleep
        self.obstacle_threshold = obstacle_threshold
        self.stop_threshold = stop_threshold
        self.sensor_handler = SensorHandler(self.thymio)
        self.position = [0, 0, 0]
        self.camera_measure = [0, 0, 0]
        self.covariance = [0.01 * np.ones([3, 3])]
        self.thymio_speed_to_mms = 0.4347
        self.kalman_ts = 0
        self.delta_sr = 0
        self.delta_sl = 0
        self.running = []
        for _ in EventEnum:
            self.running.append(False)
        self.__check_thread_init()

    def __check_thread_init(self):
        """
        Initialize the thread for checking scenarios, then pause it until next call.
        """
        self.final_occupancy_grid = create_grid()
        threading.Timer(self.interval_check, self.__check_handler).start()

        self.state = EventEnum.KALMAN.value
        self.running[EventEnum.KALMAN.value] = True
        threading.Timer(self.interval_sleep, self.__kalman_handler).start()

        self.state = EventEnum.CAMERA.value
        self.running[EventEnum.CAMERA.value] = True
        threading.Timer(self.interval_sleep, self.__camera_handler).start()

        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __check_handler(self):
        """
        Handles the different scenarios in which the robot will be.
        This function is called on it's own thread every interval_check seconds.
        """
        # print(threading.active_count())
        sensor_values = self.sensor_handler.sensor_raw()
        # print(sensor_values)

        sensor = np.amax(sensor_values["sensor"]).astype(int)  # RANDOM CONDITION FOR TESTING
        if self.state != EventEnum.GLOBAL.value and sensor <= self.obstacle_threshold:  # CHECK HERE FOR THE GLOBAL CONDITION
            print("changing to GLOBAL!!")
            self.running[self.state] = False
            self.running[EventEnum.GLOBAL.value] = True
            self.state = EventEnum.GLOBAL.value
            threading.Timer(self.interval_sleep, self.__global_handler).start()

        elif self.state != EventEnum.LOCAL.value and sensor > self.obstacle_threshold:  # CHECK HERE FOR THE LOCAL CONDITION
            print("changing to LOCAL!!")
            self.running[self.state] = False
            self.running[EventEnum.LOCAL.value] = True
            self.state = EventEnum.LOCAL.value
            threading.Timer(self.interval_sleep, self.__local_handler).start()
            return  # stop the code so that __check_handler is called after avoiding objects

        elif self.state != EventEnum.STOP.value and sensor >= self.stop_threshold:  # CHECK HERE FOR THE STOP CONDITION
            print("changing to STOP!!")
            self.running[self.state] = False
            self.running[EventEnum.STOP.value] = True
            self.state = EventEnum.STOP.value
            threading.Timer(self.interval_sleep, self.__stop_handler).start()

        time.sleep(self.interval_sleep)
        self.__check_handler()
        return

    def __global_handler(self):
        """
        Manages the thread for the GLOBAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # print("inside __global_handler")
        # path = display_occupancy(self.final_occupancy_grid, self.position)
        # update_path(self.thymio, path)

        # self.running[EventEnum.KALMAN.value] = False    # stop kalman when we reached goal
        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __local_handler(self):
        """
        Manages the thread for the LOCAL scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        ObstacleAvoidance(self.thymio, self.final_occupancy_grid, self.position)
        threading.Timer(self.interval_check, self.__check_handler).start()  # restart checking the correct state

        self.state = EventEnum.STOP.value
        self.running[EventEnum.STOP.value] = True
        self.__stop_handler()

    def __stop_handler(self):
        """
        Manages the thread for the STOP scenario.
        This function is called on it's own thread every interval_sleep seconds.
        """
        # print("inside __stop_handler")
        stop(self.thymio, verbose=True)

    def __kalman_handler(self):
        print("inside __kalman_handler")
        speed = self.sensor_handler.speed()
        self.delta_sl = speed['left_speed'] * self.kalman_ts / self.thymio_speed_to_mms / 1000
        self.delta_sr = speed['right_speed'] * self.kalman_ts / self.thymio_speed_to_mms / 1000

        temp = [np.array([[self.position[0]], [self.position[1]], [self.position[2]]])]
        z = [np.array([[self.camera_measure[0]], [self.camera_measure[1]], [self.camera_measure[2]]])]

        temp, self.covariance = kalman_filter(z, temp, self.covariance, self.delta_sr, self. delta_sl)
        self.position = [temp[2, 0], temp[1, 0], temp[0, 0]]

        if self.running[EventEnum.KALMAN.value]:
            time.sleep(self.interval_sleep)
            self.__kalman_handler()

    def __camera_handler(self):
        print("inside __camera_handler")
        # self.camera_measure = record_project()
        # TODO sleep until kalman_ts

        # i = 0 pour main webcam aka built in, 1 for first usb port etc
        # cap = cv2.VideoCapture(i)

        cap = cv2.VideoCapture(0)

        # cap = cv2.VideoCapture('test.mp4')
        # cap = cv2.VideoCapture('test_video.mov')

        fW = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        fH = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # Que QR code, a rajouter cadre + depassement
        gW = 44
        gH = 42
        cam_grid_ratio = (gW / fW, gH / fH)

        low_blue = np.array([110, 50, 50])
        up_blue = np.array([132, 255, 255])
        low_green = np.array([36, 0, 0])
        up_green = np.array([86, 255, 255])
        low_red = np.array([178, 179, 0])
        up_red = np.array([[255, 255, 255]])
        low_yellow = np.array([20, 100, 100])
        up_yellow = np.array([30, 255, 255])

        while (1):

            _, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, low_green, up_green)

            # mask = cv2.inRange(hsv, low_green, up_green)
            resg = cv2.bitwise_and(frame, frame, mask)

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            areas = [cv2.contourArea(c) for c in contours]
            if len(areas) < 5:

                # Display the resulting frame
                frame = cv2.resize(frame, (0, 0), None, 1, 1)
                cv2.imshow('frame', frame)
                # If "q" is pressed on the keyboard, exit this loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


            else:

                # Find the largest moving object in the image
                max_index = np.argmax(areas)

                cnt = contours[max_index]
                xg, yg, wg, hg = cv2.boundingRect(cnt)

                # Draw circle in the center of the bounding box

                mask = cv2.inRange(hsv, low_yellow, up_yellow)
                resy = cv2.bitwise_and(frame, frame, mask)

                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
                areas = [cv2.contourArea(c) for c in contours]
                if len(areas) < 5:

                    # Display the resulting frame
                    frame = cv2.resize(frame, (0, 0), None, 1, 1)
                    cv2.imshow('frame', frame)
                    masky = cv2.inRange(hsv, low_yellow, up_yellow)
                    resy = cv2.bitwise_and(frame, frame, masky)
                    maskg = cv2.inRange(hsv, low_green, up_green)
                    resg = cv2.bitwise_and(frame, frame, maskg)
                    cv2.imshow('resg', maskg)
                    cv2.imshow('resy', masky)

                    # If "q" is pressed on the keyboard, exit this loop
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    max_index = np.argmax(areas)
                    cnt = contours[max_index]
                    xy, yy, wy, hy = cv2.boundingRect(cnt)

                    x2g = xg + int(wg / 2)
                    y2g = yg + int(hg / 2)
                    x2y = xy + int(wy / 2)
                    y2y = yy + int(hy / 2)
                    # cv2.circle(frame,(x2,y2),4,(255,255,0),-1)
                    x2 = int((x2g + x2y) / 2)
                    y2 = int((y2g + y2y) / 2)
                    text = "Robot center in map's squares"
                    cv2.putText(frame, text, (x2 - 120, y2 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # int(x2*cam_grid_ratio[0])) is the x value in grid coord
                    # gH-int(y2*cam_grid_ratio[1]) is the y value in grid coord
                    text2 = "x: " + str(int(x2 * cam_grid_ratio[0])) + ", y: " + str(gH - int(y2 * cam_grid_ratio[1]))
                    cv2.putText(frame, text2, (x2 - 50, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    cv2.circle(frame, (x2, y2), 4, (255, 255, 0), -1)

                    # frame = cv2.resize(frame, (0, 0), None, 0.5, 0.5)
                    cv2.imshow('frame', frame)
                    masky = cv2.inRange(hsv, low_yellow, up_yellow)
                    resy = cv2.bitwise_and(frame, frame, masky)
                    maskg = cv2.inRange(hsv, low_green, up_green)
                    resg = cv2.bitwise_and(frame, frame, maskg)
                    cv2.imshow('resg', maskg)
                    cv2.imshow('resy', masky)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()
        cap.release()

        if self.running[EventEnum.CAMERA.value]:
            time.sleep(self.interval_sleep)
            self.__camera_handler()
