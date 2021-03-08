#!/usr/bin/python

# 2.12 Lab 5 me212_robot: ROS driver controlling the dynamics of the robot
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


class Planner(object):
	# Planner
	def __init__(self):

		self.PLAN = True


		self.RobotPose = RobotPose()
		self.velocities = WheelVel()
		self.rev2enc = 1000
		self.gearing = 53
		self.b = 0.225
		self.r = 0.037
		self.a = 0.3
		self.PERIOD = 0.01
		self.enc2rev = 1.0 / self.rev2enc
		self.enc2rad = self.enc2rev * 2 * np.pi
		self.enc2wheel = self.enc2rad * self.r 
		self.encoder1CountPrev = 0
		self.encoder2CountPrev = 0
		rospy.init_node('robot_controller', anonymous=True)
		rospy.Subscriber("/robot_encoder", EncVals, self.callback)
		self.vel_pub = rospy.Publisher('/wheel_velocities', WheelVel, queue_size=10)
		self.pose_pub = rospy.Publisher('/robot_pose', RobotPose, queue_size=10)
		rospy.spin()


	def callback(self, msg):
		# Takes in encoder values from simulator and goes through the different functions to update the pose
		[dPhiL, dPhiR] = self.calc_dPhis(msg.leftEnc, msg.rightEnc)
		self.updatePose(dPhiL, dPhiR, self.RobotPose)
		self.pose_pub.publish(self.RobotPose)
		desiredV = [0,0]
		if self.PLAN:
			#4
			#Drive in a straight direction
			#desiredV = self.PathPlanner(robotVel = 0.5, K = 0)

			#Commented out the answer for a u shaped trajectory
		    if self.RobotPose.pathDistance < 1.0:
		        robotVel = .2
		        K = 0
		        desiredV =self.PathPlanner(robotVel, K)
		   	# Hemicircle
		    elif self.RobotPose.pathDistance < (1+.25*np.pi):
		        robotVel = .2 
		        K = -1/0.25
		        desiredV =self.PathPlanner(robotVel, K)
			
		    # Straight line back
		    elif self.RobotPose.pathDistance < (2 + .25*np.pi):
		        robotVel = .2
		        K = 0
		        desiredV =self.PathPlanner(robotVel, K)
			
		    # Stop at the end
		    elif self.RobotPose.pathDistance > (2 + .25*np.pi):
				robotVel = 0 
				K = 0
				desiredV =self.PathPlanner(robotVel, K)
		self.updateVels(desiredV)
	

	def calc_dPhis(self, leftEnc, rightEnc):

		#1
		dEncoder1 = (rightEnc - self.encoder1CountPrev)
		dEncoder2 = (leftEnc - self.encoder2CountPrev)
	
		#update the angle increment in radians
		dphi1 = (dEncoder1 * self.enc2rad)
		dphi2 = (dEncoder2 * self.enc2rad)
		
		#for encoder index and motor position switching (Right is 1, Left is 2)
		dPhiR = dphi1
		dPhiL = dphi2
		
		self.encoder1CountPrev = rightEnc
		self.encoder2CountPrev = leftEnc
		return dPhiR, dPhiL

	def updatePose(self, dPhiL, dPhiR, Pose):

		#2
		#Takes in encoder values from the simulator to update the pose.
		
		#MODIFY CODE BELOW TO SET THE CORRECT VALUES
		#   Relevant constants: r, b
		#   Relevant function: np.cos, np.sin, np.sqrt()
		#   Use the equations referenced in the handout to set these values.
		r = self.r
		b = self.b

		dTh = r/(2*b) *(dPhiR - dPhiL)
		Pose.Th = Pose.Th + dTh
		
		dX = r/2 * (np.cos(Pose.Th)*dPhiR + np.cos(Pose.Th)*dPhiL)
		dY = r/2 * (np.sin(Pose.Th)*dPhiR + np.sin(Pose.Th)*dPhiL)
		
		Pose.X = Pose.X + dX
		Pose.Y = Pose.Y + dY

		Pose.pathDistance = Pose.pathDistance + np.sqrt(dX*dX + dY*dY)

	def PathPlanner(self, robotVel, K):
		#3
		#Takes in robot velocity and curvature, outputs wheel velocities for control
		desiredWV_L = robotVel - K *self.b * robotVel
		desiredWV_R = 2*robotVel - desiredWV_L
		return desiredWV_L, desiredWV_R

	def updateVels(self, velList):
		# publishes wheel velocities to the simulator
		self.velocities.leftVel = velList[0]
		self.velocities.rightVel = velList[1]
		self.vel_pub.publish(self.velocities)

		

if __name__ == '__main__':
	planner = Planner()