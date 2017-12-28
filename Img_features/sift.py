# -*- coding: utf-8 -*-
# from cv2.cv import *
import cv2
import numpy as np
# print('hello') 
img = cv2.imread('8.jpg',cv2.IMREAD_COLOR)
gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
# print(img)
# sift = cv2.xfeatures2d.SIFT_create()
sift = cv2.SIFT()
keypoints = sift.detect(gray,None)
img = cv2.drawKeypoints(gray,keypoints)
cv2.imshow('test',img)
cv2.waitKey(0)
# cv2.destroyAllwindows()   