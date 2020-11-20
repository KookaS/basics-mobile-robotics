# predict next value with kalman filter

import cv2 as cv
import numpy as np

low_green = np.array([36, 0, 0])
up_green = np.array([86, 255, 255])


# Instantiate OCV kalman filter
class KalmanFilter:
    kf = cv.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def estimate(self, coord_x, coord_y):
        """
        This function estimates the position of the object
        """
        measured = np.array([[np.float32(coord_x)], [np.float32(coord_y)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted


# Performs required image processing
def detect_ball(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, low_green, up_green)

    # Dilate
    kernel = np.ones((5, 5), np.uint8)
    green_mask_dilated = cv.dilate(mask, kernel)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
    areas = [cv.contourArea(c) for c in contours]
    if len(areas) < 1:
        return [-1, -1]
    else:
        max_index = np.argmax(areas)

        cnt = contours[max_index]

        epsilon = 0.01 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)
        hull = cv.convexHull(cnt, returnPoints=False)
        cv.drawContours(frame, [approx], -1, (255, 255, 255), 1)

        x, y, w, h = cv.boundingRect(cnt)
        # cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)

        # Draw circle in the center of the bounding box
        X = x + int(w / 2)
        Y = y + int(h / 2)
        cv.circle(frame, (X, Y), 4, (255, 255, 0), -1)

        return [X, Y]


def detect_object():
    cap = cv.VideoCapture(0)

    width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    # Create Kalman Filter Object
    kf_obj = KalmanFilter()
    predicted_coords = np.zeros((2, 1), np.float32)

    while True:
        rc, frame = cap.read()

        if rc == True:
            [X, Y] = detect_ball(frame)
            if X == -1 and Y == -1:
                cv.imshow('frame', frame)
                # If "q" is pressed on the keyboard, exit this loop
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                predicted_coords = kf_obj.estimate(X, Y)

                # Draw Actual coords from segmentation
                cv.circle(frame, (int(X), int(Y)), 4, (0, 0, 255), -1)
                cv.line(frame, (int(X), int(Y)), (int(X + 50), int(Y + 20)), [100, 100, 255], 2, 8)
                cv.putText(frame, "Actual", (int(X + 50), int(Y + 20)), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           [50, 200, 250])

                # Draw Kalman Filter Predicted output
                cv.circle(frame, (int(predicted_coords[0]), int(predicted_coords[1])), 4, (0, 255, 255), -1)
                cv.line(frame, (int(predicted_coords[0]), int(predicted_coords[1])),
                        (int(predicted_coords[0]) + 50, int(predicted_coords[1]) - 30), [100, 10, 255], 2, 8)
                cv.putText(frame, "Predicted", (int(predicted_coords[0]) + 50, int(predicted_coords[1]) - 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, [50, 200, 250])
                cv.imshow('frame', frame)

                if cv.waitKey(300) & 0xFF == ord('q'):
                    break

        else:
            break

    cap.release()
    cv.destroyAllWindows()
