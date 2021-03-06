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
class Scan_msg:
    def __init__(self):
		'''Initializes an object of this class.
		The constructor creates a publisher, a twist message.
		3 integer variables are created to keep track of where obstacles exist.
		3 dictionaries are to keep track of the movement and log messages.'''
		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
		self.msg = Twist()
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0
		self.sect_5 = 0
		self.ang = {0:-0.8, 1:-0.3, 2:0, 3:0.3, 4:0.8}
		self.fwd = {0:0, 1:0, 2:.25, 3:0, 4:0}
		self.dbgmsg = {0:'turn left fast', 1:'turn left', 2:'go forward', 3:'turn right', 4:'turn right fast'}
    def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0
		self.sect_5 = 0
		self.state = FSM()
		self.state.setCurrentState('Scan')
    def sort(self, laserscan):
		entries = len(laserscan.ranges)
        	#print(laserscan.ranges[3])
		for entry in range(0,entries):
			if (0 < entry < ceil(entries/5) and not math.isnan(laserscan.ranges[entry])):
				self.sect_1 = self.sect_1 + laserscan.ranges[entry] 
			if ((1 + ceil(entries/5)) < entry < ceil(entries*2/5) and not math.isnan(laserscan.ranges[entry])):
				self.sect_2 += laserscan.ranges[entry] 
			if ((1 + ceil(entries*2/5)) < entry < ceil(entries*3/5) and not math.isnan(laserscan.ranges[entry])):
				self.sect_3 += laserscan.ranges[entry] 
			if ((1 + ceil(entries*3/5)) < entry < ceil(entries*4/5) and not math.isnan(laserscan.ranges[entry])):
				self.sect_4 += laserscan.ranges[entry] 
			if ((1 + ceil(entries*4/5)) < entry < entries and not math.isnan(laserscan.ranges[entry])):
				self.sect_5 += laserscan.ranges[entry] 
		self.sect_1 = self.sect_1/(entries/5)
		self.sect_2 = self.sect_2/(entries/5)
		self.sect_3 = self.sect_3/(entries/5)
		self.sect_4 = self.sect_4/(entries/5)
		self.sect_5 = self.sect_5/(entries/5)
    def movement(self):
		'''Uses the information known about the obstacles to move robot.
		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		if self.state.getCurrentState() == 'Scan':
			array = [self.sect_1, self.sect_2, self.sect_3, self.sect_4, self.sect_5]
			sect = np.argmax(array)
			print(sect, self.ang[sect], self.fwd[sect])
			self.reset_sect()
	        	self.msg.angular.z = self.ang[sect]
			self.msg.linear.x = self.fwd[sect]
			self.pub.publish(self.msg)
		elif self.state.getCurrentState() == 'Hit Left':
			self.goBack()
			self.turn(-45)
			self.pause()
			self.state.setCurrentState('Scan')
		elif self.state.getCurrentState() == 'Hit Right':
			self.goBack()
			self.turn(45)
			self.pause()
			self.state.setCurrentState('Scan')
		elif self.state.getCurrentState() == 'Hit Center':
			self.goBack()
			self.turn(90)
			self.pause()
			self.state.setCurrentState('Scan')
		elif self.state.getCurrentState() == 'Wheel Drop':
			self.goBack()
			self.turn(180)
			self.pause()
			self.state.setCurrentState('Scan')
		elif self.state.getCurrentState() == 'Cliff':
			self.goBack()
			self.turn(180)
			self.pause()
			self.state.setCurrentState('Scan')
    def for_callback(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect
		variables the proper values.  Then the movement function is run
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		self.sort(laserscan)
		self.movement()
	# functions to move the turtlebot from sensor events
    def goBack(self):
		print ('back it up')
		for x in range(0,10):
			move_cmd = Twist()
			move_cmd.linear.x = -0.2
			# let's turn at 0 radians/s
			move_cmd.angular.z = 0
			self.pub.publish(move_cmd)
			rospy.Rate(10).sleep()
    def turn(self,degrees):
		print ('turn away')
		for x in range(0,5):
			#let's turn at 45 deg/s
			turn_cmd = Twist()
			turn_cmd.linear.x = 0
			turn_cmd.angular.z = radians(degrees); #45 deg/s in radians/s
			self.pub.publish(turn_cmd)
			rospy.Rate(10).sleep()
    def pause(self):
		rospy.sleep(2)
		print ('STOP')
	# callback functions
def BumperEventCallback(self,data):
	if (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.LEFT):
		sub_obj.state.setCurrentState('Hit Left')
	elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.RIGHT):
		sub_obj.state.setCurrentState('Hit Right')
	elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.CENTER):
		sub_obj.state.setCurrentState('Hit Center')
def WheelDropEventCallback(data):
	if (data.state == WheelDropEvent.DROPPED):
		sub_obj.state.setCurrentState('Wheel Drop')
def CliffCallback(data):
	if (data.state == CliffEvent.CLIFF):
		sub_obj.state.setCurrentState('Cliff')
def call_back(scanmsg):
	'''Passes laser scan message to for_callback function of sub_obj.
	Parameter scanmsg is laserscan message.'''
	sub_obj.for_callback(scanmsg)
def listener():
	'''Initializes node, creates subscriber, and states callback
	function.'''
	rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")
	sub = rospy.Subscriber('/scan', LaserScan, call_back)
	# create subscribers
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent, BumperEventCallback)
	rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent, WheelDropEventCallback)
	rospy.Subscriber("/mobile_base/events/cliff",CliffEvent, CliffCallback)
	rospy.spin()
if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run'''
	sub_obj = Scan_msg()
	listener()
