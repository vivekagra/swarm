#!/usr/bin/python
import rospy
import roslib

import math 
import numpy

# Messages
from std_msgs.msg import Float32

# Query Jarvis robot for left and right wheel encoders.
# Publish the estimated left and right angular wheel velocities
class WheelEncoderPublisher:
	def __init__(self):
		rospy.init_node('jarvis_state_updater')
		# Read in tangential velocity targets
		self.front_lwheel_w_motor_sub = rospy.Subscriber('front_lwheel_w_motor', Float32, self.front_lwheel_w_motor_callback)
		self.front_rwheel_w_motor_sub = rospy.Subscriber('front_rwheel_w_motor', Float32, self.front_rwheel_w_motor_callback)
		self.rear_lwheel_w_motor_sub = rospy.Subscriber('rear_lwheel_w_motor', Float32, self.rear_lwheel_w_motor_callback)
		self.rear_rwheel_w_motor_sub = rospy.Subscriber('rear_rwheel_w_motor', Float32, self.rear_rwheel_w_motor_callback)
	

		self.front_lwheel_w_control_pub = rospy.Subscriber('front_lwheel_w_control', Float32, self.front_lwheel_w_control_callback)
		self.front_rwheel_w_control_pub = rospy.Subscriber('front_rwheel_w_control', Float32, self.front_rwheel_w_control_callback)
		self.rear_lwheel_w_control_pub = rospy.Subscriber('rear_lwheel_w_control', Float32, self.rear_lwheel_w_control_callback)
		self.rear_rwheel_w_control_pub = rospy.Subscriber('rear_rwheel_w_control', Float32, self.rear_rwheel_w_control_callback)


		self.front_lwheel_w_enc_pub = rospy.Publisher('front_lwheel_w_enc', Float32, queue_size=10)
		self.front_rwheel_w_enc_pub = rospy.Publisher('front_rwheel_w_enc', Float32, queue_size=10)
		self.rear_lwheel_w_enc_pub = rospy.Publisher('rear_lwheel_w_enc', Float32, queue_size=10)
		self.rear_rwheel_w_enc_pub = rospy.Publisher('rear_rwheel_w_enc', Float32, queue_size=10)
		
	
		self.rate = rospy.get_param('~rate', 10)
		self.err_tick_incr = rospy.get_param('~err_tick_incr',20) # Filter out clearly erroneous encoder readings
		self.time_prev_update = rospy.Time.now();
		self.gopigo_on = rospy.get_param('~gopigo_on',True)
		
		if self.gopigo_on:
			import gopigo   
			self.front_lwheel_encs = [gopigo.enc_read(1)]*5
			self.front_rwheel_encs = [gopigo.enc_read(0)]*5
			self.rear_lwheel_encs = [gopigo.enc_read(1)]*5
			self.rear_rwheel_encs = [gopigo.enc_read(0)]*5

		self.R = rospy.get_param('~robot_wheel_radius', .03)

		# Need a little hack to incorporate direction wheels are spinning
		self.front_lwheel_dir = 1;
		self.front_rwheel_dir = 1;
		self.rear_lwheel_dir = 1;
		self.rear_rwheel_dir = 1;
		
		self.front_rwheel_w_control = 0;
		self.front_lwheel_w_control = 0;
		self.rear_rwheel_w_control = 0;
		self.rear_lwheel_w_control = 0;


  # Really bad hack to get motor spin direction
	def front_lwheel_w_motor_callback(self,msg):
		if msg.data >= 0: self.front_lwheel_dir = 1
		else: self.front_lwheel_dir = -1
	# Really bad hack to get motor spin direction
	def front_rwheel_w_motor_callback(self,msg):
		if msg.data >= 0: self.front_rwheel_dir = 1
		else: self.front_rwheel_dir = -1
	def rear_lwheel_w_motor_callback(self,msg):
		if msg.data >= 0: self.rear_lwheel_dir = 1
		else: self.rear_lwheel_dir = -1
	# Really bad hack to get motor spin direction
	def rear_rwheel_w_motor_callback(self,msg):
		if msg.data >= 0: self.rear_rwheel_dir = 1
		else: self.rear_rwheel_dir = -1


	def front_lwheel_w_control_callback(self,msg):
		self.front_lwheel_w_control = msg.data
	def front_rwheel_w_control_callback(self,msg):
		self.front_rwheel_w_control = msg.data
	def rear_lwheel_w_control_callback(self,msg):
		self.rear_lwheel_w_control = msg.data
	def rear_rwheel_w_control_callback(self,msg):
		self.rear_rwheel_w_control = msg.data


	def enc_2_rads(self,enc_cms):
		prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
		rads =  prop_revolution * (2*math.pi)
		return rads

	def update(self):
		if self.gopigo_on: # Running on actual robot
			import gopigo
			front_lwheel_enc = self.front_lwheel_dir * gopigo.enc_read(1) * .01 # cm's moved
			front_rwheel_enc = self.front_rwheel_dir * gopigo.enc_read(0) * .01 # cm's moved
			rear_lwheel_enc = self.rear_lwheel_dir * gopigo.enc_read(1) * .01 # cm's moved
			rear_rwheel_enc = self.rear_rwheel_dir * gopigo.enc_read(0) * .01 # cm's moved

			self.front_lwheel_encs = self.front_lwheel_encs[1:] + [front_lwheel_enc]
			self.front_rwheel_encs = self.front_rwheel_encs[1:] + [front_rwheel_enc]
			self.rear_lwheel_encs = self.rear_lwheel_encs[1:] + [rear_lwheel_enc]
			self.rear_rwheel_encs = self.rear_rwheel_encs[1:] + [rear_rwheel_enc]

			# History of past three encoder reading
			time_curr_update = rospy.Time.now()
			dt = (time_curr_update - self.time_prev_update).to_sec()

			# Compute angular velocity in rad/s
			front_lwheel_enc_delta = abs(self.front_lwheel_encs[-1]) - abs(self.front_lwheel_encs[-2])
			front_rwheel_enc_delta = abs(self.front_rwheel_encs[-1]) - abs(self.front_rwheel_encs[-2])
			rear_lwheel_enc_delta = abs(self.rear_lwheel_encs[-1]) - abs(self.rear_lwheel_encs[-2])
			rear_rwheel_enc_delta = abs(self.rear_rwheel_encs[-1]) - abs(self.rear_rwheel_encs[-2])
						
			front_lwheel_w_enc = self.enc_2_rads(front_lwheel_enc_delta) / dt
			front_rwheel_w_enc = self.enc_2_rads(front_rwheel_enc_delta) / dt
			rear_lwheel_w_enc = self.enc_2_rads(rear_lwheel_enc_delta) / dt
			rear_rwheel_w_enc = self.enc_2_rads(rear_rwheel_enc_delta) / dt

			# Adjust sign
			if self.front_lwheel_encs[-1] < 0: front_lwheel_w_enc = -front_lwheel_w_enc
			if self.front_rwheel_encs[-1] < 0: front_rwheel_w_enc = -front_rwheel_w_enc
			if self.rear_lwheel_encs[-1] < 0: rear_lwheel_w_enc = -rear_lwheel_w_enc
			if self.rear_rwheel_encs[-1] < 0: rear_rwheel_w_enc = -rear_rwheel_w_enc

			self.front_lwheel_w_enc_pub.publish(front_lwheel_w_enc)
			self.front_rwheel_w_enc_pub.publish(front_rwheel_w_enc)
			self.rear_lwheel_w_enc_pub.publish(rear_lwheel_w_enc)
			self.rear_rwheel_w_enc_pub.publish(rear_rwheel_w_enc)

			self.time_prev_update = time_curr_update

		else: # Running in simulation -- blindly copy from target assuming perfect execution
			self.front_lwheel_w_enc_pub.publish(self.front_lwheel_w_control)
			self.front_rwheel_w_enc_pub.publish(self.front_rwheel_w_control)
			self.rear_lwheel_w_enc_pub.publish(self.rear_lwheel_w_control)
			self.rear_rwheel_w_enc_pub.publish(self.rear_rwheel_w_control)


	def spin(self):
		rospy.loginfo("Start gopigo_state_updater")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)

		while not rospy.is_shutdown():
			self.update();
			rate.sleep()
			rospy.spin()

	def shutdown(self):
		rospy.loginfo("Stop gopigo_state_updater")
		# Stop message
		self.front_lwheel_w_enc_pub.publish(0)
		self.front_rwheel_w_enc_pub.publish(0)
		self.rear_lwheel_w_enc_pub.publish(0)
		self.rear_rwheel_w_enc_pub.publish(0)

		rospy.sleep(1)

def main():
	encoder_publisher = WheelEncoderPublisher();
	encoder_publisher.spin()

if __name__ == '__main__':
	main(); 


