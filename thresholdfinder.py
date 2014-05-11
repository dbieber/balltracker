import cv
import cv2
import os
import random
import numpy as np
import math
import sys

class ThresholdFinder():

    def __init__(self, filename):
        self.filename = filename
        self.vc = cv2.VideoCapture(filename)
        self.centers = []

    # mouse callback function
    def print_points(self, event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.centers.append((x,y))

    def get_frame(self, i=None):
        if i is not None:
            self.vc.set(cv.CV_CAP_PROP_POS_FRAMES, i)
        ret, img = self.vc.read()
        return img

    # Click point on frame to log it.
    def click_frame(self, i):
       # f.write("Clicking Frame %d\n"%i)
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.print_points)

        cv2.imshow('image', self.get_frame(i))
        k = cv2.waitKey(-1) & 0xFF
        if k == 27:
            cv2.destroyAllWindows()
            return True
        elif k == 13:
            cv2.destroyAllWindows()
            return False

    # call click_frame 3 times, store which frames are called
    def get_optimal_hsv(self):
        # click 3 frames
        num_frames = 3
        clicked_frames = []
        frame_count = int(self.vc.get(cv.CV_CAP_PROP_FRAME_COUNT))
        for i in xrange(0, num_frames): # 3 times
            frame_index = random.randint(100, frame_count-1)
            
            # estimate the ball's center
            while (self.click_frame(frame_index)): # True means that we hit escape
                frame_index = random.randint(100, frame_count-1)

            clicked_frames.append(frame_index)

        cv2.destroyAllWindows()
        print "xx"

        min_dist = sys.maxint
        optimal_hsv = [-1, -1, -1]
        for h in xrange(0, 256, 5):
            print h
            for s in xrange(0, 256, 5):
                for v in xrange(0, 256, 5):
                    dist = 0

                    for i in xrange(0, num_frames):
                        center, radius = self.find_ball_in_frame(self.get_frame(clicked_frames[i]), np.array([h,s,v]))
                        x_found, y_found = center
                        x_clicked, y_clicked = self.centers[i]

                        # add to distance
                        dist += (x_found-x_clicked)**2 + (y_found-y_clicked)**2

                    if dist < min_dist:
                        min_dist = dist
                        optimal_hsv = [h, s, v]

        return optimal_hsv # list with 3 values (hsv)
        # if find_ball_in_frame returns None and we're assuming that the ball is in the frame... then infinity


    def set_frame(capture, i):
        self.vc.set(cv.CV_CAP_PROP_POS_FRAMES, i)


    def threshold_by_color(self, img, hue_l):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_orange = hue_l
        upper_orange = np.array([255, 255, 255])

        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        return mask


    def find_ball_in_frame(self, frame, hue_l):
        # TODO(Bieber) remove redundency
        tbc = self.threshold_by_color(frame, hue_l)
        ret,thresh = cv2.threshold(tbc, 127, 255, 0)
       # cv2.imshow('arc', tbc)
        #cv2.waitKey(-1)

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
        for frame_i in xrange(400, 1000, 30):
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

    filename_mov = "%s/data/run000/backleft.MOV" % os.getcwd()
    tf = ThresholdFinder(filename_mov)
    print tf.get_optimal_hsv()

if __name__ == '__main__':
    main()
