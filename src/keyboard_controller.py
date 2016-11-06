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
from drone_lane_detect import DroneLaneDetect
from drone_image_process import DroneImageProcess

# Finally the GUI libraries
from PySide import QtCore, QtGui

# Navigation Vehicle Coordinates

print 'wpVec: %d %d' % (str(wpVec[0]), str(wpVec[1]))

# gains
KYAW = .1

UPDATE_PERIOD = 100 #ms
PING_INTERVAL = 250 #ms

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
