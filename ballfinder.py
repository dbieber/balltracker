import cv
import cv2
import os
import random
import numpy as np
import math

class BallFinder():

    def __init__(self, filename):
        self.filename = filename
        self.vc = cv2.VideoCapture(filename)

    def get_frame(self, i=None):
        if i is not None:
            self.vc.set(cv.CV_CAP_PROP_POS_FRAMES, i)
        ret, img = self.vc.read()
        return img

    def set_frame(capture, i):
        self.vc.set(cv.CV_CAP_PROP_POS_FRAMES, i)


    def threshold_by_color(self, img):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        # lower_orange = np.array([3, 183, 247])
        # upper_orange = np.array([255, 255, 255])
        lower_orange = np.array([0, 206, 161]) # version that seems to work well
        # lower_orange = np.array([0, 0, 0])
        upper_orange = np.array([255, 255, 255])


        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        return mask


    def find_ball_in_frame(self, frame):
        # TODO(Bieber) remove redundency
        tbc = self.threshold_by_color(frame)
        ret,thresh = cv2.threshold(tbc, 127, 255, 0)
        # cv2.imshow('arc', tbc)
        # cv2.waitKey(-1)

        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        contours = sorted(contours, lambda c1, c2: 1 if cv2.contourArea(c2) - cv2.contourArea(c1) > 0 else -1)

        if contours:
            cnt = contours[0]  # largest contour
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            return (center, radius)

    def show_frame_with_ball(self, frame, ball_location, ball_radius):
        frame_copy = frame.copy()
        cv2.circle(frame_copy, ball_location, ball_radius, (0,0,255), 1)
        cv2.imshow('frame', frame_copy)
        cv2.waitKey(-1)

    def find(self):
        # Returns list of ball locations
        locations = []
        frame_count = int(self.vc.get(cv.CV_CAP_PROP_FRAME_COUNT))
        for frame_i in xrange(0, frame_count):
            frame = self.get_frame(frame_i)
            ball = self.find_ball_in_frame(frame)
            if ball:
                ball_location, ball_radius = ball
                # self.show_frame_with_ball(frame, ball_location, ball_radius)
            locations.append(ball)
            print frame_i
        return locations

def main():
    # filename_mov = "%s/data/run000/backright.MOV" % os.getcwd()
    # bf = BallFinder(filename_mov)
    # print bf.find()

    filename_mov = "%s/data/run000/backright.MOV" % os.getcwd()
    bf = BallFinder(filename_mov)
    print bf.find()

if __name__ == '__main__':
    main()