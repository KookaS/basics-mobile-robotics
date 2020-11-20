# le mieux avec un mask bleu

import cv2 as cv
import numpy as np

# i = 0 pour main webcam aka built in, 1 for first usb port etc
# cap = cv.VideoCapture(i)

cap = cv.VideoCapture(0)

# cap = cv.VideoCapture('test.mp4')
# cap = cv.VideoCapture('long.mov')

fW = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
fH = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
# Que QR code, a rajouter cadre + depassement
gW = 44
gH = 42
cam_grid_ratio = (gW / fW, gH / fH)

low_green = np.array([36, 0, 0])
up_green = np.array([86, 255, 255])

while True:

    _, frame = cap.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, low_green, up_green)
    res = cv.bitwise_and(frame, frame, mask=mask)

    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
    areas = [cv.contourArea(c) for c in contours]
    if len(areas) < 1:

        # Display the resulting frame
        frame = cv.resize(frame, (0, 0), None, 1, 1)
        cv.imshow('frame', frame)
        # If "q" is pressed on the keyboard, exit this loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        # Find the largest moving object in the image
        max_index = np.argmax(areas)

        cnt = contours[max_index]

        epsilon = 0.01 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)
        hull = cv.convexHull(cnt, returnPoints=False)
        # draw the white contour
        cv.drawContours(frame, [approx], -1, (255, 255, 255), 3)

        x, y, w, h = cv.boundingRect(cnt)
        # cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)

        # Draw circle in the center of the bounding box
        x2 = x + int(w / 2)
        y2 = y + int(h / 2)
        cv.circle(frame, (x2, y2), 4, (255, 255, 0), -1)

        # Print the centroid coordinates (we'll use the center of the bounding box) on the image
        # coordinate in image coordinate
        # text = "x: " + str(x2) + ", y: " + str(y2)
        text = "Robot center in map's squares"
        cv.putText(frame, text, (x2 - 120, y2 - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # int(x2*cam_grid_ratio[0])) is the x value in grid coord
        # gH-int(y2*cam_grid_ratio[1]) is the y value in grid coord
        text2 = "x: " + str(int(x2 * cam_grid_ratio[0])) + ", y: " + str(gH - int(y2 * cam_grid_ratio[1]))
        cv.putText(frame, text2, (x2 - 50, y2 + 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        frame = cv.resize(frame, (0, 0), None, 1, 1)
        cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.destroyAllWindows()
cap.release()

# predict next value with kalman filter
low_green = np.array([36, 0, 0])
up_green = np.array([86, 255, 255])


# Instantiate OCV kalman filter
class KalmanFilter:
    kf = cv.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted


def detect_object():
    cap = cv.VideoCapture(0)

    width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    # Create Kalman Filter Object
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2, 1), np.float32)

    while (1):
        rc, frame = cap.read()

        if (rc == True):
            [X, Y] = detect_ball(frame)
            if X == -1 and Y == -1:
                cv.imshow('frame', frame)
                # If "q" is pressed on the keyboard, exit this loop
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                predictedCoords = kfObj.Estimate(X, Y)

                # Draw Actual coords from segmentation
                cv.circle(frame, (int(X), int(Y)), 4, (0, 0, 255), -1)
                cv.line(frame, (int(X), int(Y)), (int(X + 50), int(Y + 20)), [100, 100, 255], 2, 8)
                cv.putText(frame, "Actual", (int(X + 50), int(Y + 20)), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           [50, 200, 250])

                # Draw Kalman Filter Predicted output
                cv.circle(frame, (int(predictedCoords[0]), int(predictedCoords[1])), 4, (0, 255, 255), -1)
                cv.line(frame, (int(predictedCoords[0]), int(predictedCoords[1])),
                        (int(predictedCoords[0]) + 50, int(predictedCoords[1]) - 30), [100, 10, 255], 2, 8)
                cv.putText(frame, "Predicted", (int(predictedCoords[0]) + 50, int(predictedCoords[1]) - 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, [50, 200, 250])
                cv.imshow('frame', frame)

                if (cv.waitKey(300) & 0xFF == ord('q')):
                    break

        else:
            break

    cap.release()
    cv.destroyAllWindows()


# Segment the green ball in a given frame
def detect_ball(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, low_green, up_green)

    # Dilate
    kernel = np.ones((5, 5), np.uint8)
    greenMaskDilated = cv.dilate(mask, kernel)
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
        # cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)

        # Draw circle in the center of the bounding box
        X = x + int(w / 2)
        Y = y + int(h / 2)
        cv.circle(frame, (X, Y), 4, (255, 255, 0), -1)

        return [X, Y]
