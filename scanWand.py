#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
		self.ang = {00000: 0, 00001: 1.5, 00010: 2.5, 00011: 2.5, 00100: 2.5, 00101: 2.5,
					00110: 2.5, 00111: 2.5, 01000: -1.5, 01001: 0, 01010: 0, 01011: 2.5,
					01100: -2.5, 01101: 2.5, 01110: 2.5, 01111: 2.5, 10000: -1.5, 10001: 0,
					10010: 0, 10011: 0, 10100: -2.5, 10101: -3.5, 10110: -2.5, 10111: 2.5,
					11000: -2.5, 11001: 0, 11010: 0, 11011: 0, 11100: -2.5, 11101: -2.5,
					11110: -2.5, 11111: -2.5}
		self.fwd = {00000: 1, 00001: 0.25, 00010: 0, 00011: 0, 00100: 0, 00101: 0,
					00110: 0, 00111: 0, 01000: 0, 01001: 1, 01010: 1, 01011: 0,
					01100: 0, 01101: 0, 01110: 0, 01111: 0, 10000: 0.25, 10001: 1,
					10010: 1, 10011: 1, 10100: 0, 10101: 0, 10110: 0, 10111: 0,
					11000: 0, 11001: 1, 11010: 1, 11011: 1, 11100: 0, 11101: 0,
					11110: 0, 11111: 0}
		self.dbgmsg = {00000: 'fwd', 00001: 'left', 00010: 'left', 00011: 'left', 00100: 'left', 00101: 'left',
					   00110: 'left', 00111: 'left', 01000: 'right', 01001: 'fwd', 01010: 'fwd', 01011: 'left',
					   01100: 'right', 01101: 'left', 01110: 'left', 01111: 'left', 10000: 'right', 10001: 'fwd',
					   10010: 'fwd', 10011: 'fwd', 10100: 'right', 10101: 'right', 10110: 'right', 10111: 'left',
					   11000: 'right', 11001: 'fwd', 11010: 'fwd', 11011: 'fwd', 11100: 'right', 11101: 'right',
					   11110: 'right', 11111: 'right'}


    def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0
		self.sect_5 = 0

    def sort(self, laserscan):
		'''Goes through 'ranges' array in laserscan message and determines 
		where obstacles are located. The class variables sect_1, sect_2, 
		and sect_3 are updated as either '0' (no obstacles within 0.7 m)
		or '1' (obstacles within 0.7 m)
		Parameter laserscan is a laserscan message.'''
		entries = len(laserscan.ranges)
		for entry in range(0,entries):
		    if 0.4 < laserscan.ranges[entry] < 0.75:
				self.sect_1 = 1 if (0 < entry < entries/5) else 0 
				self.sect_2 = 1 if (entries/5 < entry < entries*2/5) else 0
				self.sect_3 = 1 if (entries*2/5 < entry < entries*3/5) else 0
				self.sect_4 = 1 if (entries*3/5 < entry < entries*4/5) else 0
				self.sect_5 = 1 if (entries*4/5 < entry < entries) else 0
		rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3) + " sect_4: " + str(self.sect_4) + " sect_4: " + str(self.sect_4))

    def movement(self):
		'''Uses the information known about the obstacles to move robot.
		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3) + str(self.sect_4) + str(self.sect_5))
		rospy.loginfo("Sect = " + str(sect)) 
	
       	self.msg.angular.z = self.ang[sect]
		self.msg.linear.x = self.fwd[sect]
		rospy.loginfo(self.dbgmsg[sect])
		self.pub.publish(self.msg)

		self.reset_sect()
 
    def for_callback(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		self.sort(laserscan)
		self.movement()
	

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
    rospy.spin()

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()