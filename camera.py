#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from sys import *
from math import *
from ctypes import *
import cv2 as cv
import test_detection as lateralcontroller
# import lane_detection as lateralcontroller

# img = cv.imread(argv[1])
# output,steercmd,res = lateralcontroller.lane_detection(img)



cap = cv.VideoCapture(0)

while(True):
	#capture frame-by-frame
    ret , frame = cap.read()
    
    
    #display the resulting frame
    # cv.imshow('frame',frame)
    lateralcontroller.lane_detection(frame)
    if cv.waitKey(1) &0xFF ==ord('q'):  #按q键退出
    	break
#when everything done , release the capture
cap.release()
cv.destroyAllWindows()
