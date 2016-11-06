#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

from sensor_msgs.msg import Image

from PySide import QtCore
import numpy as np
import cv2

IMAGE_INTERVAL = 500 #ms

class DroneImageProcess(object):
    def __init__(self):
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subImage = rospy.Subscriber('/ardrone/bottom', Image, self.ReceiveImage)
		self.imageCount = 1
		
		# Holds the image frame received from the drone and later saved on the disk
		self.image = None
		self.imageLock = Lock()
		
		# A timer to take a picture to process
		self.imgTimer = QtCore.QTimer(self)
		self.imgTimer.timeout.connect(self.ProcessImages)
		self.imgTimer.start(IMAGE_INTERVAL)
		
		# Subscribe to the /ardrone/navdata topic
		self.subNavSat = rospy.Subscriber('/ardrone/navfix', NavSatFix, self.ReceiveNavSatFix) 
		
		# Holds the nav location to be saved if surface feature is found
		self.navfix = None
		self.navfixLock = Lock()
    	
    def ReceiveImage(self, data):
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()
    
    def ProcessImages(self):
    	self.imageLock.acquire()
		try:
			# Convert your ROS Image message to OpenCV2
			cv2_img = bridge.imgmsg_to_cv2(self.image, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			edges = cv2.Canny(cv2_img, 150, 220)
			# Save accepted image as a jpeg 
			if np.any(edges[10:(edges.shape[0]-10), 10:(edges.shape[1]-10)] > 10):			
				cv2.imwrite('saved_images/img_%03d.jpeg' % self.imageCount, cv2_img)
				self.imageCount += 1
		finally:
			self.imageLock.release()

	def ReceiveNavSat(self,navdata):
		self.navfixLock.acquire()
		try:
			self.navfix = navdata # Save the ros image for processing by the display thread
		finally:
			self.navfixLock.release()
    	
