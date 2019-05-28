#!/usr/bin/env python

import rospy
import serial
from std_msgs import string
from geometry_msgs.msg import Twist

class driver:
	def __init__(self):
		rospy.init_node('jarvis_driver',anonymous = True)
		self.serial = serial.Serial('/dev/ttyUSB0',115200)
		self.get_arduino_message()
	def get_arduino_message(self):
		pub = rospy.Publisher('arduino',String,queue_size = 10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			message = self.serial.readline()
			pub.publish(message)
			r.sleep()


if __name__ == '__main__':
	try:	
		d = driver()
	except rospy.ROSInterruptException:
		pass		
