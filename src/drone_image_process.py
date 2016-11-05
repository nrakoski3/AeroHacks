#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

import numpy as np
import cv2

class drone_image_process(object):
    # Load images to ready for processing
    img = cv2.imread('')