#!/usr/bin/python

# 2.12 Lab 5 me212_robot: ROS driver simulating the dynamics of the robot 
# Jerry Ng March 2021


import rospy
import tf
import numpy as np
import threading
import serial
import pdb

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from me212_robot.msg import RobotPose, EncVals, WheelVel
#takes in desired wheel velocities and outputs an encoder value based on time passed and the dynamic model
#assumes a low level controller is in place to get the actual desired wheel velocities
class Robot(object):
	def __init__(self):
		self.rev2enc = 1000
		self.gearing = 53
		self.b = 0.225
		self.r = 0.037
		self.a = 0.3
		self.slippercent = 0.1
		#Add in slip dynamics which just removes 10% of the desired wheel velocity
		self.SLIP = True
		self.enc2rev = 1.0 / self.rev2enc
		self.enc2rad = self.enc2rev * 2 * np.pi
		self.enc2wheel = self.enc2rad * self.r 

		self.encLeft = 0
		self.encRight = 0
		self.leftVel = 0
		self.rightVel = 0
		self.enc_msg = EncVals()
		rospy.init_node('robot_simulator', anonymous=True)
		rospy.Subscriber("/wheel_velocities", WheelVel, self.callback)
		self.prevtime = rospy.Time.now()

		self.enc_pub = rospy.Publisher('/robot_encoder', EncVals, queue_size=10)
		while not rospy.is_shutdown():
			self.simulate()
			rospy.sleep(0.01)

	def callback(self, msg):
		#Takes wheel velocity message and enforces that as the current wheel velocity
		self.time_passed = rospy.Time.now().to_sec() - self.prevtime.to_sec()
		self.leftVel = msg.leftVel
		self.rightVel = msg.rightVel

	def simulate(self):
		time_passed = rospy.Time.now().to_sec() - self.prevtime.to_sec()
		if self.SLIP:
			epsR = self.rightVel*self.slippercent
			epsL = self.leftVel*self.slippercent
		else:
			epsR = 0
			epsL = 0

		#Calculate encoder values
		self.encLeft = self.encLeft + self.leftVel/self.enc2wheel * time_passed + epsR/self.enc2wheel*time_passed
		self.encRight = self.encRight + self.rightVel/self.enc2wheel * time_passed + epsL/self.enc2wheel*time_passed

		#Input to the message variable
		self.enc_msg.leftEnc = self.encLeft
		self.enc_msg.rightEnc = self.encRight
		self.enc_pub.publish(self.enc_msg)
		self.prevtime = rospy.Time.now()

if __name__ == '__main__':
	robot = Robot()


