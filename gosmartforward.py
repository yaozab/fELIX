#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from FSM import FSM
from math import radians

class GoForward():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)


	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # create subscribers
        rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
        rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
        self.stateMachine = FSM()

     
	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        self.r = rospy.Rate(10);
        self.move_cmd = Twist()

                        
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

    # functions to move the turtlebot                  
    def goBack(self):
      print ('back it up')
      for x in range(0,10):
        self.move_cmd.linear.x = -0.2
        # let's turn at 0 radians/s
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        self.r.sleep()

    def turn(self, degrees):
      print ('turn away')
      for x in range(0,5):
        #let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(degrees); #45 deg/s in radians/s
        self.cmd_vel.publish(turn_cmd)
        self.r.sleep() 

    def pause(self):
      rospy.sleep(2)
      print ('STOP')
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	       # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	       # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

def move():
  while not rospy.is_shutdown():
    #print(gof.stateMachine.getCurrentState())
    if (gof.stateMachine.getCurrentState() == 'Hit Left'):
      # back, turn right, pause
      gof.goBack()
      gof.turn(45)
      gof.pause()
      print("hit left")
      gof.stateMachine.setCurrentState('Go Forward')
    elif (gof.stateMachine.getCurrentState() == 'Hit Right'):
      # back, turn right, pause
      gof.goBack()
      gof.turn(-45)
      gof.pause()
      print("hit right")
      gof.stateMachine.setCurrentState('Go Forward')
    elif (gof.stateMachine.getCurrentState() == 'Hit Center'):
      # back, turn right, pause
      gof.goBack()
      gof.turn(90)
      gof.pause()
      print("hit center")
      gof.stateMachine.setCurrentState('Go Forward')
    elif (gof.stateMachine.getCurrentState() == 'Wheel Drop'):
      # back, turn right, pause
      gof.goBack()
      gof.turn(180)
      gof.pause()
      print("wheel drop")
      gof.stateMachine.setCurrentState('Go Forward')
    else:
      print("move")
      # let's go forward at 0.2 m/s
      gof.move_cmd.linear.x = 0.2
      # let's turn at 0 radians/s
      gof.move_cmd.angular.z = 0
      # publish the velocity
      gof.cmd_vel.publish(gof.move_cmd)
      # wait for 0.1 seconds (10 HZ) and publish again
    gof.r.sleep()
  rospy.spin()

if __name__ == '__main__':
    gof = GoForward()
    move()

