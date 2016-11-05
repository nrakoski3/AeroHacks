#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

from sensor_msgs.msg import Image

import numpy as np
import cv2

class drone_image_process(object):
    def __init__(path):
    	self.path = path
    
    def processImages(self):
    	for imgname in os.listdir(self.path):
	    if 'Thumbs.db' not in imgname:
		# Load images to ready for processing
    	    	img = cv2.imread(self.path + '/' + imgname)
    	    	edges = cv2.Canny(img, 100, 200)
    	    	if np.all(edges[10:(edges.shape[0]-10), 10:(edges.shape[1]-10)] < 1):
  		    os.remove(self.path + '/' + imgname)