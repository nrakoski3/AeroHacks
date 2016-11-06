#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import rosbag

from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.msg import navdata_gps

from PySide import QtCore
import numpy as np
import cv2

IMAGE_INTERVAL = 500 #ms
ALT_THRESH = 1000 #mm

class DroneImageProcess(object):
    def __init__(self):
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subImage = rospy.Subscriber('ardrone/bottom/image_raw', Image, self.ReceiveImage)
		self.imageCount = 1
		
		# Holds the image frame received from the drone and later saved on the disk
		self.image = None
		self.imageLock = Lock()
		
		# A timer to take a picture to process
		self.imgTimer = QtCore.QTimer(self)
		self.imgTimer.timeout.connect(self.ProcessImages)
		self.imgTimer.start(IMAGE_INTERVAL)
		
		# Subscribe to the /ardrone/navdata topic
		self.subNavSat = rospy.Subscriber('ardrone/navdata_gps', navdata_gps, self.ReceiveNavSat) 
		
		# Holds the nav location to be saved if surface feature is found
		self.navfix = None
		self.navfixLock = Lock()
		
		# Subscribe to the /ardrone/navdata topic
		self.subNavData = rospy.Subscriber('ardrone/navdata', Navdata, self.ReceiveNavData) 
		
		# Holds the navdata altitude for a check
		self.navdata = None
		self.navdataLock = Lock()
    	
    def ReceiveImage(self, data):
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()
    
    def ProcessImages(self):
    	self.imageLock.acquire()
    	self.navdataLock.acquire()
		try:
			# Convert your ROS Image message to OpenCV2
			cv2_img = bridge.imgmsg_to_cv2(self.image, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			edges = cv2.Canny(cv2_img, 150, 220)
			# Save accepted image as a jpeg 
			if np.any(edges[10:(edges.shape[0]-10), 10:(edges.shape[1]-10)] > 10) && self.navdata > ALT_THRESH:
				self.navfixLock.acquire()
				cv2.imwrite('saved_images/img_%03d.jpeg' % self.imageCount, cv2_img)
				self.imageCount += 1
				bag = rosbag.Bag('navfixes.bag', 'a')
				try:
					bag.write('navdata_gps_lat', self.navfix.latitude)
					bag.write('navdata_gps_long', self.navfix.longitude)
				finally:
					bag.close()
					self.navfixLock.release()
		finally:
			self.navdataLock.release()
			self.imageLock.release()

	def ReceiveNavSat(self, navdata):
		self.navfixLock.acquire()
		try:
			self.navfix = navdata # Record the navdata to be saved if image is good
		finally:
			self.navdataLock.release()
    
    def ReceiveNavData(self, navdata):
		self.navfixLock.acquire()
		try:
			self.navdata = navdata # Record the navdata to be saved if image is good
		finally:
			self.navdataLock.release()
