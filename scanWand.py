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
		self.ang = {0: 0, 1: 0.5, 10: 1.5, 11: 1.5, 100: 1.5, 101: 1.5,
			    110: 1.5, 111: 1.5, 1000: -0.5, 1001: 0, 1010: 0, 1011: 1.5,
			    1100: -1.5, 1101: 1.5, 1110: 1.5, 1111: 1.5, 10000: -0.5, 10001: 0,
			    10010: 0, 10011: 0, 10100: -1.5, 10101: -2.5, 10110: -1.5, 10111: 1.5,
			    11000: -1.5, 11001: 0, 11010: 0, 11011: 0, 11100: -1.5, 11101: -1.5,
			    11110: -1.5, 11111: -1.5}
		self.fwd = {0: 0.5, 1: 0.25, 10: 0, 11: 0, 100: 0, 101: 0,
			    110: 0, 111: 0, 1000: 0, 1001: 0.5, 1010: 0.5, 1011: 0,
			    1100: 0, 1101: 0, 1110: 0, 1111: 0, 10000: 0.25, 10001: 0.5,
			    10010: 0.5, 10011: 0.5, 10100: 0, 10101: 0, 10110: 0, 10111: 0,
			    11000: 0, 11001: 0.5, 11010: 0.5, 11011: 0.5, 11100: 0, 11101: 0,
			    11110: 0, 11111: 0}
		self.dbgmsg = {0: 'fwd', 1: 'left', 10: 'left', 11: 'left', 100: 'left', 101: 'left',
			       110: 'left', 111: 'left', 1000: 'right', 1001: 'fwd', 1010: 'fwd', 1011: 'left',
			       1100: 'right', 1101: 'left', 1110: 'left', 1111: 'left', 10000: 'right', 10001: 'fwd',
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
	# functions to move the turtlebot
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
def BumperEventCallback(data):
	if (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.LEFT):
		# move right
		sub_obj.goBack()
		sub_obj.turn(-45)
		sub_obj.pause()
		print("hit left")
	elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.RIGHT):
		# move left
		sub_obj.goBack()
		sub_obj.turn(45)
		sub_obj.pause()
	elif (data.state == BumperEvent.PRESSED and data.bumper == BumperEvent.CENTER):
		# backwards
		sub_obj.goBack()
		sub_obj.turn(90)
		sub_obj.pause()
def WheelDropEventCallback(data):
	if (data.state == WheelDropEvent.DROPPED):
		# backwards
		sub_obj.goBack()
		sub_obj.turn(180)
		sub_obj.pause()
def CliffCallback(data):
	if (data.state == CliffEvent.CLIFF):
		# backwards
		sub_obj.goBack()
		sub_obj.turn(180)
		sub_obj.pause()	

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
