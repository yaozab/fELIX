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
		self.sub = rospy.Subscriber('/scan', LaserScan, self.call_back)
		self.pub = rospy.Publisher('/gapscan',LaserScan, queue_size = 10)
		self.gapArray = []
		self.obstacles = []
		self.r = rospy.Rate(5)		

    def call_back(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect
		variables the proper values.  Then the movement function is run
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		# take in laserscan data, and account for NAN
		entries = len(laserscan.ranges)
		# make binary array, one if obstacle 0 if free
		nanCounter = 0
		previousVal = laserscan.range_min
		binLaserscan = laserscan
		nanStartIndex = 0
		laserArr = np.zeros(entries)
		for entry in range(0,entries):
			if (laserscan.ranges[entry] > 1.5):
				laserArr[entry] = laserscan.range_max
			else:
				# if nan and close
				if nanCounter > 5:
					for j in range(nanStartIndex, entry+1)
						laserArr[j] = laserscan.range_min
					nanCounter = 0
				else:
					if math.isnan(laserscan.ranges[entry]):
						if nanCounter == 0:
							nanStartIndex = entry
						laserArr[entry] = laserscan.range_max
						nanCounter += 1
			previousVal = laserscan.ranges[entry]

		# making the gap index
		previousVal = 0
		gapIndex = []
		for entry in range(0,entries):
			if previousVal == 0:
				previousVal = laserArr[entry]
			else:
				if laserArr[entry] == laserscan.range_max and previousVal == laserscan.range_min:
					gapIndex.append(entry)
				if laserArr[entry] == laserscan.range_min and previousVal == laserscan.range_max:
					gapIndex.append(entry)
				previousVal = laserArr[entry]
		print gapIndex
		# gapIndexes is storing the start and end of the gap arrays.
		#middleVal = math.ceil(entries/2)
		#for i in range(0,len(gapIndexes)):
		#	self.obstacles[i] = -(middleVal - i) * laserscan.angle_increment

		self.pub.publish(binLaserscan)
		self.r.sleep() 

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
