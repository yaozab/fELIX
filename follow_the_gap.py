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
		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
    	#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)

		self.gapArray = []
		self.obstacles = []
		self.r = rospy.Rate(5)
		self.forwardSpeed = 0.2
		self.angularSpeed = 0.1	
	'''
	# callback functions
	def BumperEventCallback(self, data):
		if (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.LEFT):
			self.stateMachine.setCurrentState('Hit Left')
		elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.RIGHT):
			self.stateMachine.setCurrentState('Hit Right')
		elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.CENTER):
			self.stateMachine.setCurrentState('Hit Center')
		else:
			self.stateMachine.setCurrentState('Go Forward')

	def WheelDropEventCallback(self,data):
		if (data.state == WheelDropEvent.DROPPED):
			self.stateMachine.setCurrentState('Wheel Drop')
		else:
			self.stateMachine.setCurrentState('Go Forward')
	'''
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
			if (laserscan.ranges[entry] > 0.5):
				#print(entry)
				laserArr[entry] = laserscan.range_max
			else:															
				if (laserArr[entry - 1] == laserscan.range_max):
					if (entry + 1) <= entries:
						if laserscan.ranges[entry+1] > 0.5:
							laserArr[entry] = laserscan.range_max
						else:
							laserArr[entry] = laserscan.range_min
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
		largestGapIndexes = gapIndex[np.argmax(gapSize)]
		center = ceil(entries/2)
		leftDiff = center - largestGapIndexes[0]
		rightDiff = center - largestGapIndexes[1]
		middleDiff = ceil((leftDiff + rightDiff)/2)
		gapCenterAngle = -middleDiff * laserscan.angle_increment
		
		#print(min(laserscan.ranges))
		#print (entries, largestGapIndexes, leftDiff, rightDiff, middleDiff, gapCenterAngle)
		print('Gap center angle: ', math.degrees(gapCenterAngle))
		
		if math.abs(math.degrees(gapCenterAngle)) < 15:
			# if less than 15 deg, go forward
			move = Twist()
			move.linear.x = self.forwardSpeed
			self.cmd_vel.Publish(move)
		else:
			angSpeed = gapCenterAngle * self.turnSpeed
			move = Twist()
			move.angular.z = angSpeed
			self.cmd_vel.Publish(move)

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
