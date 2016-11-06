#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import math
from threading import Lock

import numpy as np

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

# Navigation Vehicle Coordinates

# gains
KYAW = .1

UPDATE_PERIOD = 100 #ms

# Code estimates median line of edges of road to find relative heading

# Import images
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

import cv2

from PySide import QtCore, QtGui

PING_INTERVAL = 250 #ms

import sys, os

import rosbag

from ardrone_autonomy.msg import Navdata
# from ardrone_autonomy.msg import navdata_gps

IMAGE_INTERVAL = 800 #ms
ALT_THRESH = 1000 	#mm
OBJ_THRESH = 10 	# A threshold for number of corners to count as a obstacle

import sensor_msgs.msg


class CvBridgeError(TypeError):
    """
    This is the error raised by :class:`cv_bridge.CvBridge` methods when they fail.
    """
    pass


class CvBridge(object):
    """
    The CvBridge is an object that converts between OpenCV Images and ROS Image messages.

       .. doctest::
           :options: -ELLIPSIS, +NORMALIZE_WHITESPACE

           >>> import cv2
           >>> import numpy as np
           >>> from cv_bridge import CvBridge
           >>> br = CvBridge()
           >>> dtype, n_channels = br.encoding_as_cvtype2('8UC3')
           >>> im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
           >>> msg = br.cv2_to_imgmsg(im)  # Convert the image to a message
           >>> im2 = br.imgmsg_to_cv2(msg) # Convert the message to a new image
           >>> cmprsmsg = br.cv2_to_compressed_imgmsg(im)  # Convert the image to a compress message
           >>> im22 = br.compressed_imgmsg_to_cv2(msg) # Convert the compress message to a new image
           >>> cv2.imwrite("this_was_a_message_briefly.png", im2)

    """

    def __init__(self):
        import cv2
        self.cvtype_to_name = {}
        self.cvdepth_to_numpy_depth = {cv2.CV_8U: 'uint8', cv2.CV_8S: 'int8', cv2.CV_16U: 'uint16',
                                       cv2.CV_16S: 'int16', cv2.CV_32S:'int32', cv2.CV_32F:'float32',
                                       cv2.CV_64F: 'float64'}

        for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F"]:
            for c in [1, 2, 3, 4]:
                nm = "%sC%d" % (t, c)
                self.cvtype_to_name[getattr(cv2, "CV_%s" % nm)] = nm

        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                                        'int16': '16S', 'int32': '32S', 'float32': '32F',
                                        'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)

    def cvtype2_to_dtype_with_channels(self, cvtype):
        from cv_bridge.boost.cv_bridge_boost import CV_MAT_CNWrap, CV_MAT_DEPTHWrap
        return self.cvdepth_to_numpy_depth[CV_MAT_DEPTHWrap(cvtype)], CV_MAT_CNWrap(cvtype)

    def encoding_to_cvtype2(self, encoding):
        from cv_bridge.boost.cv_bridge_boost import getCvType

        try:
            return getCvType(encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

    def encoding_to_dtype_with_channels(self, encoding):
        return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))

    def compressed_imgmsg_to_cv2(self, cmprs_img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::CompressedImage message to an OpenCV :cpp:type:`cv::Mat`.

        :param cmprs_img_msg:   A :cpp:type:`sensor_msgs::CompressedImage` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :cpp:type:`cv::Mat` message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.

        If the image only has one channel, the shape has size 2 (width and height)
        """
        import cv2
        import numpy as np

        str_msg = cmprs_img_msg.data
        buf = np.ndarray(shape=(1, len(str_msg)),
                          dtype=np.uint8, buffer=cmprs_img_msg.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)

        if desired_encoding == "passthrough":
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor2

        try:
            res = cvtColor2(im, "bgr8", desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def imgmsg_to_cv2(self, img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::Image message to an OpenCV :cpp:type:`cv::Mat`.

        :param img_msg:   A :cpp:type:`sensor_msgs::Image` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :cpp:type:`cv::Mat` message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.

        If the image only has one channel, the shape has size 2 (width and height)
        """
        import cv2
        import numpy as np
        dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, img_msg.width),
                           dtype=dtype, buffer=img_msg.data)
        else:
            im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                           dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            im = im.byteswap().newbyteorder()

        if desired_encoding == "passthrough":
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor2

        try:
            res = cvtColor2(im, img_msg.encoding, desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def cv2_to_compressed_imgmsg(self, cvim, dst_format = "jpg"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::CompressedImage message.

        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param dst_format:  The format of the image data, one of the following strings:

           * from http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html
           * from http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)
           * bmp, dib
           * jpeg, jpg, jpe
           * jp2
           * png
           * pbm, pgm, ppm
           * sr, ras
           * tiff, tif

        :rtype:           A sensor_msgs.msg.CompressedImage message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``format``


        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        import cv2
        import numpy as np
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        cmprs_img_msg = sensor_msgs.msg.CompressedImage()
        cmprs_img_msg.format = dst_format
        ext_format = '.' + dst_format
        try:
            cmprs_img_msg.data = np.array(cv2.imencode(ext_format, cvim)[1]).tostring()
        except RuntimeError as e:
            raise CvBridgeError(e)

        return cmprs_img_msg

    def cv2_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``

        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        import cv2
        import numpy as np
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = sensor_msgs.msg.Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
                raise CvBridgeError("encoding specified as %s, but image has incompatible type %s" % (encoding, cv_type))
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) / img_msg.height

        return img_msg


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
			cv2_img = CvBridge.imgmsg_to_cv2(self.image, "bgr8")
	    except CvBridgeError, e:
			print(e)
		else:
			gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
			gray = np.float32(gray)
			dst = cv2.cornerHarris(gray, 2, 3, 0.04)
			# Save accepted image as a jpeg
			if np.count_nonzero(dst[32:(dst.shape[0]-32), 24:(dst.shape[1]-24)] > 0.01*dst.max()) > OBJ_THRESH and self.navdata > ALT_THRESH:
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
	    self.navdataLock.release()
    
    def ReceiveNavData(self, navdata):
	self.navfixLock.acquire()
	try:
	    self.navdata = navdata # Record the navdata to be saved if image is good
	finally:
            self.navdataLock.release()

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
			cv2_img = CvBridge.imgmsg_to_cv2(self.image, "bgr8")
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


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
    PitchForward = QtCore.Qt.Key.Key_E
    PitchBackward = QtCore.Qt.Key.Key_D
    RollLeft = QtCore.Qt.Key.Key_S
    RollRight = QtCore.Qt.Key.Key_F
    YawLeft = QtCore.Qt.Key.Key_W
    YawRight = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff = QtCore.Qt.Key.Key_Y
    Land = QtCore.Qt.Key.Key_H
    Emergency = QtCore.Qt.Key.Key_Space
    Waypoint = QtCore.Qt.Key.Key_N
    Hover = QtCore.Qt.Key.Key_M


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
    def __init__(self):
        super(KeyboardController, self).__init__()

        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0
        self.wayPointMode = 0
		
        self.pos = [0, 0]
        self.wp = [20, 0]
        self.posToWpMag = math.sqrt((self.wp[0] - self.pos[0]) ** 2 + (self.wp[1] - self.pos[1]) ** 2)
        self.wpVec = [(self.wp[0] - self.pos[0]) / self.posToWpMag, (self.wp[1] - self.pos[1]) / self.posToWpMag]
        if self.wpVec[0] != 0:
            if (self.wpVec[0] < 0 and self.wpVec[1] < 0) | (self.wpVec[0] < 0 and self.wpVec[1] > 0):
                self.wpAng = math.atan(self.wpVec[1] / self.wpVec[0]) * 180 / math.pi +180
            elif self.wpVec[0] < 0 and self.wpVec[1] ==0:
                self.wpAng = 180
            else:
                self.wpAng = math.atan(self.wpVec[1] / self.wpVec[0]) * 180 / math.pi
        elif self.wpVec[1] > 0:
            self.wpAng = 90
        elif self.wpVec[1] ==0:
            self.wpAng = 0
        else:
            self.wpAng = -90
		self.laneDetector = DroneLaneDetect()
		self.PosLock = Lock()
		
        self.positionTimer = QtCore.QTimer(self)
        self.positionTimer.timeout.connect(self.PosUpdateCallback)
        self.positionTimer.start(UPDATE_PERIOD)
        
        self.ImageTimer = QtCore.QTimer(self)
	    self.ImageTimer.timeout.connect(self.LaneCorrection)
		self.ImageTimer.start(PING_INTERVAL)

    # We add a keyboard handler to the DroneVideoDisplay to react to keypresses
    def keyPressEvent(self, event):
        key = event.key()
        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if (self.posToWpMag < 1) or (controller is not None and not event.isAutoRepeat()):
            # Handle the important cases first!
            if key == KeyMapping.Emergency:
                controller.SendEmergency()
            elif key == KeyMapping.Takeoff:
                controller.SendTakeoff()
            elif key == KeyMapping.Land:
                controller.SendLand()
            elif key == KeyMapping.Hover:
                self.yaw_velocity = 0

                self.pitch = 0

                self.roll = 0

                self.z_velocity = 0

                self.wayPointMode = 0

        elif (key == KeyMapping.Waypoint) or (self.wayPointMode == 1):
			self.PosLock.acquire()
			try:
				if abs(self.rotZ - self.wpAng) > 1:
					self.yaw_velocity = KYAW * (self.rotZ - selfwpAng)
				else:
					self.yaw_velocity = 0

				if math.cos((self.rotZ - self.wpAng) * math.pi/180) > 0:
					self.pitch = 10 * math.cos((self.rotZ - self.wpAng) * math.pi/180)
				else:
					self.pitch = 0

				if abs(self.posToWpMag * math.sin((self.rotZ - self.wpAng) * math.pi/180)) > 0.1:
					self.roll = KYAW * self.posToWpMag * sin((self.rotZ - self.wpAng))
				else:
					self.roll = 0

				self.z_velocity = 0

				self.wayPointMode = 1
			finally:
				self.PosLock.release()
        else:

            # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity += 1
            elif key == KeyMapping.YawRight:
                self.yaw_velocity += -1

            elif key == KeyMapping.PitchForward:
                self.pitch += 1
            elif key == KeyMapping.PitchBackward:
                self.pitch += -1

            elif key == KeyMapping.RollLeft:
                self.roll += 1
            elif key == KeyMapping.RollRight:
                self.roll += -1

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity += 1
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity += -1
            # finally we set the command to be sent. The controller handles sending this at regular intervals

        controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def keyReleaseEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= 1
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -1

            elif key == KeyMapping.PitchForward:
                self.pitch -= 1
            elif key == KeyMapping.PitchBackward:
                self.pitch -= -1

            elif key == KeyMapping.RollLeft:
                self.roll -= 1
            elif key == KeyMapping.RollRight:
                self.roll -= -1

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= 1
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -1

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def PosUpdateCallBack(self):
        self.PosLock.acquire()
        try:
			self.pos[0] = self.pos[0] + .1 * self.vx/1000 * cos(self.rotZ) - .1 * self.vy/1000 * sin(self.rotZ)
			self.pos[1] = self.pos[1] + .1 * self.vy/1000 * sin(self.rotZ) + .1 * self.vx/1000 * cos(self.rotZ)
			self.posToWpMag = math.sqrt((self.wp[0] - self.pos[0]) ** 2 + (self.wp[1] - self.pos[1]) ** 2)
			self.wpVec = [(self.wp[0] - self.pos[0]) / self.posToWpMag, (self.wp[1] - self.pos[1]) / self.posToWpMag]
			self.wpAng = math.atan(self.wpVec[1] / self.wpVec[0]) * 180 / math.pi
		finally:
			self.PosLock.release()

	def LaneCorrection(self):
		self.laneDetector.PosLock.acquire()
		self.PosLock.acquire()
		try:
			self.wpAng -= self.laneDetector.heading
			self.pos[0] += self.laneDetector.offset_from_tz_string
		finally:
			self.PosLock.release()
			self.lanedetector.PosLock.release()

# Setup the application
if __name__ == '__main__':
    import sys

    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_keyboard_controller')

    # Now we construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = BasicDroneController()
    imager = DroneImageProcess()
    display = KeyboardController()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
