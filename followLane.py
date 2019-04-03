# followLane.py
# A program for an RC car to follow a lane in an automated fashion

# EE443 Final Year Project Meadhbh Griffin 15366976
# 3rd April 2019

# import necessary libraries
import numpy as np # for dealing with large arrays
from picamera.array import PiRGBArray # to grab arrays from camera
from picamera import PiCamera # to use camera
import cv2 # OpenCV
import time # sleep
import RPi.GPIO as GPIO # connect to Pi's GPIO
import math # for mathematical operations

# carries out required preprocessing
def processImage(img, h, w):
    y = np.int0(0.45 * h) # find pixels for vertical crop
    x = np.int0(0.025 * w) # find pixels for horizontal crop
    dst = img[y:y+h, x:x+w] # crop image accordingly
    grey = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY) # convert colourspace to greyscale
    blurred = cv2.GaussianBlur(grey, (11, 11), 0) # apply Gaussian Blur to remove noise
    thresh = cv2.threshold(blurred, 25, 255, cv2.THRESH_BINARY_INV)[1] # apply inverse binary threshold function
    return thresh

# find x-coordinate of lane midpoint
def findMidpoint(thresh):
    cx = 0 # initialise lane midpoint's x-coordinate
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # find image contours
    if contours: # if contours were found
        centrex = [] # initialise list of contour x-coordinates
        for contour in contours: # for each contour
            area = cv2.contourArea(contour) # calculate area
            if (80000 > area > 50): # if area within suitable range
                rect = cv2.minAreaRect(contour) # find bounding box
                box = cv2.cv.BoxPoints(rect) # get rectangular points of bounding box
                (x1,_), (_, _), _ = rect # get contour's midpoint's x-coordinate
                centrex.append(x1) # and append it to the list

        if centrex: # if the centres were found
            cx = np.mean(centrex) # find the mean of the array of x-coordinates
    return cx #return lane midpoint's x-coordinate

# find error between lane midpoint and image midpoint
def getError(cx, w):
    error = w - cx # calculate difference between lane midpoint and image midpoint
    if error > 0.1*w or error < -1*(0.1 * w): # if error is significant
        # if the error is positive
		if error > 0:
            # image mid-point is right of lane midpoint; turn left to correct
            print("Turn left to correct error of " + str(error))
            return error
        # if the error is negative
        elif error < 0:
            # image mid-point is left of lane midpoint; turn right to correct
            print("Turn right to correct error of " + str(-1*error))
            return error
    # if error is not significant no need to turn
    else:
        print("Following lane")
        return 0

# use GPIO pins to control motors to turn
def turn(error, w, l, r):
    dc = 0 # initialise duty cycle
    if error != 0: # if error is significant
        dc = (float(error)/float(w)) # find ratio between error and half image width
        dc = math.floor(100 * dc) # convert to valid number for duty cycle
		# if duty cycle is valid and positive
        if 100 >= dc >= 1:
            # image mid-point is right of lane midpoint; turn left to correct
            r.ChangeDutyCycle(0) # stop motor from turning right
            l.ChangeDutyCycle(dc) # start turning motor left using dc
        # if the duty cycle is valid and is negative
        elif 0 >= dc >= -100:
            # image mid-point is left of lane midpoint; turn right to correct
            l.ChangeDutyCycle(0) # stop motor from turning left
            r.ChangeDutyCycle(-dc) # start turning motor right using -dc
    else:
        # if error is 0 no need to turn; reset steering motors
        l.ChangeDutyCycle(dc)
        r.ChangeDutyCycle(dc)

# clean up GPIO pins and initialise them
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT); l = GPIO.PWM(12, 50); l.start(0)
GPIO.setup(16, GPIO.OUT); r = GPIO.PWM(16, 50); r.start(0)
GPIO.setup(18, GPIO.OUT); a = GPIO.PWM(18, 50); a.start(0)
GPIO.setup(22, GPIO.OUT); d = GPIO.PWM(22, 50); d.start(0)

w = 640 # set width
h = 480 # set height
camera = PiCamera() # get reference to camera
camera.resolution = (w, h) # set resolution
rawCapture = PiRGBArray(camera, size=(w, h)) # initialise capture
time.sleep(1) #allow camera to warm up

# for each frame in the continous BGR capture from video port
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array # get array representation of frame
    thresh = processImage(image, h, w) # find binary image
    cx = findMidpoint(thresh) # find lane midpoint's x-coordinate
    if cx != 0: # if the midpoint was found
        error = getError(cx, w/2) # calculate error between cx and image midpoint
        a.ChangeDutyCycle(18) # minimum duty cycle needed for wheels to rotate
        turn(error, w/2, l, r) # turn car in appropriate direction
    else:
        a.ChangeDutyCycle(0) # stop the car
        
    rawCapture.truncate(0) # clear stream in preparation for next frame

GPIO.cleanup() # cleanup GPIO pins