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
		rospy.init_node(follow_the_gap)
		rospy.loginfo("follow the gap starting")
		self.sub = rospy.Subscriber('/scan', LaserScan, call_back)
		self.pub = rospy.Publisher('/gapscan',LaserScan, queue_size = 10)
		self.gapArray = []
		self.obstacles = []
		self.r = rospy.Rate(5)
		

    def callback(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect
		variables the proper values.  Then the movement function is run
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		# take in laserscan data, and account for NAN
		entries = len(laserscan.ranges)
		# make binary array, one if obstacle 0 if free
		binLaserscan = laserscan
		objectFound = 0
		for entry in range(0,entries):
			if laserscan[entry] > 1.5 or math.isnan(laserscan[entry]):
				binLaserscan[entry] = 13
			else:
				binLaserscan[entry] = 1

		pub.publish(binLaserscan)
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
