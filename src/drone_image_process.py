#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import rosbag

from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
# from ardrone_autonomy.msg import navdata_gps

from PySide import QtCore
import numpy as np
import cv2

# An enumeration of Drone Statuses
from drone_status import DroneStatus

IMAGE_INTERVAL = 800 #ms
ALT_THRESH = 1000 	#mm
OBJ_THRESH = 10 	# A threshold for number of corners to count as a obstacle

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
		# self.subNavSat = rospy.Subscriber('ardrone/navdata_gps', navdata_gps, self.ReceiveNavSat) 
		
		# Holds the nav location to be saved if surface feature is found
		# self.navfix = None
		# self.navfixLock = Lock()
		
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
    	if self.navdata.state == DroneStatus.Flying:
			self.imageLock.acquire()
			self.navdataLock.acquire()
			try:
				# Convert your ROS Image message to OpenCV2
				cv2_img = bridge.imgmsg_to_cv2(self.image, "bgr8")
			except CvBridgeError, e:
				print(e)
			else:
				gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
				gray = np.float32(gray)
				dst = cv2.cornerHarris(gray, 2, 3, 0.04)
				# Save accepted image as a jpeg 
				if (np.count_nonzero(dst[32:(dst.shape[0]-32), 24:(dst.shape[1]-24)] > 0.01*dst.max()) > OBJ_THRESH) and (self.navdata > ALT_THRESH):
					self.navfixLock.acquire()
					cv2.imwrite('saved_images/img_%03d.jpeg' % self.imageCount, cv2_img)
					self.imageCount += 1
					bag = rosbag.Bag('navfixes.bag', 'a')
					try:
						# bag.write('navdata_gps_lat', self.navfix.latitude)
						# bag.write('navdata_gps_long', self.navfix.longitude)
						bag.write('battery', self.navdata.batteryPercent)
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
			self.navfixLock.release()
    
    def ReceiveNavData(self, navdata):
		self.navdataLock.acquire()
		try:
			self.navdata = navdata # Record the navdata to be saved if image is good
		finally:
			self.navdataLock.release()
