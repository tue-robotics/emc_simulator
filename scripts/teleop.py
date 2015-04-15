#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   7    8    9
   4    5    6
   1    2    3

/ 	: rotate counterclockwise
* 	: rotate clockwise

f/v : increase/decrease altitude

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'1':(-0.7,0.7,0),
		'2':(-1,0,0),
		'3':(-0.7,-0.7,0),
		'4':(0,1,0),
		'6':(0,-1,0),
		'7':(0.7,0.7,0),
		'8':(1,0,0),
		'9':(0.7,-0.7,0),
		'/':(0,0,1),
		'*':(0,0,-1)
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .1
turn = .2

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/pico/base/reference', Twist)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					y = moveBindings[key][1]
					th = moveBindings[key][2]
				else:
					x = 0
					y = 0
					th = 0
					if (key == '\x03'):
						break
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.z = 0; twist.linear.y = y*speed
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


