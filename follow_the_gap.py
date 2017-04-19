#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from math import radians, ceil
from FSM import FSM
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import CliffEvent
from nav_msgs.msg import Odometry

class follow_the_gap:
    def __init__(self):
		'''Initializes an object of this class.
		The constructor creates a publisher, a twist message.
		3 integer variables are created to keep track of where obstacles exist.
		3 dictionaries are to keep track of the movement and log messages.'''
		rospy.init_node('follow_the_gap', anonymous=True)
		rospy.loginfo("follow the gap starting")

		self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_call_back)
		self.pub = rospy.Publisher('/gapscan',LaserScan, queue_size = 10)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.orientation = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
	    	#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)

		self.gapArray = []
		self.obstacles = []
		self.r = rospy.Rate(10)
		self.forwardSpeed = 0.2
		self.angularSpeed = 2
		self.startX = None
		self.startY = None
		self.startZ = None
		
		self.currX = 0
		self.currY = 0
		self.currZ = 0
		
		self.endX = 0
		self.endY = 0
		self.endZ = 0

    def odom_callback(self, data):
	    	if self.startX == None:
	    		self.startX = data.pose.pose.position.x
	    		self.endX = self.startX - 15
	    	if self.startY == None:
	    		self.startY = data.pose.pose.position.y
	    		self.endY = self.startY
	    	if self.startZ == None:
	        	self.startZ = data.pose.pose.orientation.z
	        	self.endZ = self.startZ
	    
	    	self.currX = data.pose.pose.position.x
	    	self.currY = data.pose.pose.position.y
	    	self.currZ = data.pose.pose.orientation.z
		
    def laser_call_back(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect
		variables the proper values.  Then the movement function is run
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		# take in laserscan data, and account for NAN
		entries = len(laserscan.ranges)
		# make binary array, one if obstacle 0 if free
		nanCounter = 0
		previousVal = laserscan.range_min
		previousPreviousVal = laserscan.range_min
		binLaserscan = laserscan
		nanStartIndex = 0
		laserArr = np.zeros(entries)
		for entry in range(0,entries):
			if (laserscan.ranges[entry] > 1.5) or np.isnan(laserscan.ranges[entry]):
				#print(entry)
				laserArr[entry] = laserscan.range_max
			else:
			    laserArr[entry] = laserscan.range_min															

		# making the gap index
		previousVal = 0
		gapStart = 0
		gapIndex = []
		for entry in range(0,entries):
			if previousVal == 0:
				previousVal = laserArr[entry]
			else:
				if laserArr[entry] == laserscan.range_max and previousVal == laserscan.range_min:
					gapStart = entry
				if laserArr[entry] == laserscan.range_min and previousVal == laserscan.range_max:
					gapIndex.append((gapStart,entry))
				previousVal = laserArr[entry]
		#print gapIndex
		# gapIndexes is storing the start and end of the gap arrays.
		gapSize = []
		for gap in gapIndex:
			gapSize.append(gap[1] - gap[0])
		#print(gapSize, np.argmax(gapSize))
		if len(gapIndex) < 1:
		    gapCenterAngle = 0.1
		else:
		    largestGapIndexes = gapIndex[np.argmax(gapSize)]
		    center = ceil(entries/2)
		    leftDiff = center - largestGapIndexes[0]
		    rightDiff = center - largestGapIndexes[1]
		    middleDiff = ceil((leftDiff + rightDiff)/2)
		    gapCenterAngle = -middleDiff * laserscan.angle_increment
		
		#print(min(laserscan.ranges))
		#print (entries, largestGapIndexes, leftDiff, rightDiff, middleDiff, gapCenterAngle)
		print('Gap center angle: ', math.degrees(gapCenterAngle))
		'''
		if self.currX == self.endX and self.currY == self.startY and self.currZ == self.startZ:
			print('at goal, shutting down')
			self.shutdown()
		elif self.currX == self.endX and self.currY == self.startY:
		    # need to turn to correct angle
			angleDiff2 = self.currZ - self.startZ
			move = Twist()
			move.angular.z = angleDiff2
			self.cmd_vel.publish(move)
		elif self.currX == self.endX:
		    # move back to y location
			print('movey')
			
		else:
		'''
		if True:
			if np.abs(math.degrees(gapCenterAngle)) < 5:
				# if less than 15 deg, go forward
				print('moving forward')
				move = Twist()
				angleErr = self.currY - self.endY
				print(angleErr)
				move.angular.z = -angleErr
				move.linear.x = self.forwardSpeed
				self.cmd_vel.publish(move)
			else:
		    		angSpeed = gapCenterAngle
		    		print('angular speed', angSpeed)
		    		move = Twist()
		    		move.angular.z = angSpeed * 1.2
		    		move.linear.x = self.forwardSpeed
		    		self.cmd_vel.publish(move)

		binLaserscan.ranges = laserArr
		self.pub.publish(binLaserscan)
		#self.r.sleep() 

    def shutdown(self):
		rospy.loginfo("shutdown")
		rospy.sleep(1)

if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run'''
        follow = follow_the_gap()
        rospy.on_shutdown(follow.shutdown)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("shutting down")
