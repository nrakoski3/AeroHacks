# Code estimates median line of edges of road to find relative heading

import roslib; roslib.load_manifest('AeroHacks')
import rospy

# Import images
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from PySide import QtCore, QtGui

PING_INTERVAL = 250 #ms

class DroneLaneDetect():
	def __init__(self):
		# Obtain video feed to determine 
		self.subImage = rospy.Subscriber('ardrone/front/image_raw', Image, self.ReceiveImage)
		
		self.image = None
		self.ImageLock = Lock()
		
		self.heading = 0 #degrees, right is positive
		self.displacement = 0 #mm, starboard is positive
		self.PosLock = Lock()
		
		self.ImageTimer = QtCore.QTimer(self)
		self.ImageTimer.timeout.connect(self.ImageCallback)
		self.ImageTimer.start(PING_INTERVAL)
		
	def ReceiveImage(self, data):
		self.ImageLock.acquire()
		try:
			self.image = data
		finally:
			self.ImageLock.release()
			
	def ImageCallback(self):
		self.ImageLock.acquire()
		try:
			# Convert your ROS Image message to OpenCV2
			cv2_img = bridge.imgmsg_to_cv2(self.image, "bgr8")
			cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_RGB2GRAY)
		except CvBridgeError, e:
			print(e)
		else:
			edges = cv2.Canny(cv2_img[0:round(0.5*cv2_img.shape[0], :], 100, 200)
			f = lambda(x): return np.sum(np.nonzero(x > 0))/np.sum(x>0)
			median = np.apply_along_axis(f, 1, edges)
			line = np.polyfit(range(len(median)), median(::-1), 1)
			slope = 22.5/(4*line[1])
			offset = (line[0] - edges.shape[1]/2)/edges.shape[1] * 12 * 1000
			self.PosLock.acquire()
			try:
				self.heading = math.atan(-1/slope) * 180/math.pi
				self.displacement = offset-1/slope*6*1000
			finally:
				self.PosLock.release()
		finally:
			self.ImageLock.release()
