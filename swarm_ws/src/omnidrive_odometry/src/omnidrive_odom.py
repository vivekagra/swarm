#!/usr/bin/python
import rospy
import roslib
import math 
import numpy as np
import tf

# Messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

# Use left and right angular velocities to compute robot unicycle velocities
# Publish the estimated velocity update
class OdomPublisher:
	def __init__(self):
		rospy.init_node('omnidrive_odom')
		
		self.front_lwheel_w_enc_sub = rospy.Subscriber('front_lwheel_w_enc', Float32, self.front_lwheel_w_enc_callback)    
		self.front_rwheel_w_enc_sub = rospy.Subscriber('front_rwheel_w_enc', Float32, self.front_rwheel_w_enc_callback)    
		self.rear_lwheel_w_enc_sub = rospy.Subscriber('rear_lwheel_w_enc', Float32, self.rear_lwheel_w_enc_callback)    
		self.rear_rwheel_w_enc_sub = rospy.Subscriber('rear_rwheel_w_enc', Float32, self.rear_rwheel_w_enc_callback)    
		

		# self.lwheel_tangent_vel_enc_pub = rospy.Publisher('lwheel_tangent_vel_enc', Float32, queue_size=10)
		# self.rwheel_tangent_vel_enc_pub = rospy.Publisher('rwheel_tangent_vel_enc', Float32, queue_size=10)
		
		self.cmd_vel_enc_pub = rospy.Publisher('cmd_vel_enc', Twist, queue_size=10)

		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

		self.H = rospy.get_param('~robot_wheel_center_horizontal_separation_distance', 0.125)
		self.V = rospy.get_param('~robot_wheel_center_vertical_separation_distance', 0.075) 
		self.R = rospy.get_param('~robot_wheel_radius', 0.03)
		self.rate = rospy.get_param('~rate', 50)
		self.N = rospy.get_param('~robot_wheel_ticks', 280)
		self.frame_id = rospy.get_param('~frame_id','/odom')
		self.child_frame_id = rospy.get_param('~child_frame_id','/base_link')

		self.tf_broadcaster = tf.TransformBroadcaster()

		self.front_lwheel_w_enc = 0;
		self.front_rwheel_w_enc = 0;
		self.rear_lwheel_w_enc  = 0;
		self.rear_rwheel_w_enc  = 0;

		self.pose = {'x':0, 'y': 0, 'th': 0}
		self.time_prev_update = rospy.Time.now();



	def front_lwheel_w_enc_callback(self, msg):
	    self.front_lwheel_w_enc = msg.data



	def front_rwheel_w_enc_callback(self, msg):
	    self.front_rwheel_w_enc = msg.data



	def rear_lwheel_w_enc_callback(self, msg):
		self.rear_lwheel_w_enc = msg.data



	def rear_rwheel_w_enc_callback(self, msg):
		self.rear_rwheel_w_enc = msg.data



	def angularvel_2_tangentvel(self,w):
		tangent_vel = w * self.R
		return tangent_vel



	def pose_next(self, front_lwheel_tangent_vel_enc, front_rwheel_tangent_vel_enc, rear_lwheel_tangent_vel_enc, rear_rwheel_tangent_vel_enc):
	  	x = self.pose['x']; y = self.pose['y']; th = self.pose['th']
		time_curr_update = rospy.Time.now()
		dt = (time_curr_update - self.time_prev_update).to_sec()
		self.time_prev_update = time_curr_update
		vx		 = 0.25*(-front_rwheel_tangent_vel_enc + front_lwheel_tangent_vel_enc - rear_lwheel_tangent_vel_enc + rear_rwheel_tangent_vel_enc)
		vy		 = 0.25*(+front_rwheel_tangent_vel_enc + front_lwheel_tangent_vel_enc + rear_lwheel_tangent_vel_enc + rear_rwheel_tangent_vel_enc)
		w		 = 0.25*(+front_rwheel_tangent_vel_enc - front_lwheel_tangent_vel_enc - rear_lwheel_tangent_vel_enc + rear_rwheel_tangent_vel_enc)/(self.H+self.V)    
		del_x_r  = vx*dt
		del_y_r  = vy*dt
		del_th_r = w*dt
		del_x    = del_x_r*np.cos(th) + del_y_r*np.sin(th)
		del_y    = del_x_r*np.sin(th) + del_y_r*np.cos(th)
		del_th   = del_th_r
		x        = x  + del_x
		y        = y  + del_y
		th       = th + del_th
		#+++++++++++++++++++++-------------------------- update odometry data here ------------------------+++++++++++++++++++++#
		return {'x':x, 'y':y, 'th':th,'vx':vx,'vy':vy,'w':w}



	def pose_update(self):
		front_lwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.front_lwheel_w_enc)
		front_rwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.front_rwheel_w_enc)
		rear_lwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.rear_lwheel_w_enc)
		rear_rwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.rear_rwheel_w_enc)

		# self.front_lwheel_tangent_vel_enc_pub.publish(front_lwheel_tangent_vel_enc)
		# self.front_rwheel_tangent_vel_enc_pub.publish(front_rwheel_tangent_vel_enc)
		# self.rear_lwheel_tangent_vel_enc_pub.publish(rear_lwheel_tangent_vel_enc)
		# self.rear_rwheel_tangent_vel_enc_pub.publish(rear_rwheel_tangent_vel_enc)

		pose_next = self.pose_next(front_lwheel_tangent_vel_enc, front_rwheel_tangent_vel_enc, rear_lwheel_tangent_vel_enc, rear_rwheel_tangent_vel_enc)

		cmd_vel_enc = Twist()
		cmd_vel_enc.linear.x = pose_next['vx']
		cmd_vel_enc.linear.y = pose_next['vy']
		cmd_vel_enc.angular.z = pose_next['w']
		self.cmd_vel_enc_pub.publish(cmd_vel_enc)

		return pose_next



	def pub_odometry(self,pose):
		# Construct odometry message
		odom_msg = Odometry()
		odom_msg.header.stamp = self.time_prev_update
		odom_msg.header.frame_id = self.frame_id
		odom_msg.child_frame_id = self.child_frame_id
		odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
		odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
		#P = numpy.mat(numpy.diag([0.0]*3)) # Dummy covariance
		#odom_msg.pose.covariance = tuple(P.ravel().tolist())
		self.odom_pub.publish(odom_msg)



	def pub_tf(self,pose):
		self.tf_broadcaster.sendTransform( \
                              (pose['x'], pose['y'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              self.time_prev_update, \
                              self.child_frame_id, \
                              self.frame_id \
                              )



	def update(self):
		self.pose = self.pose_update();
		self.pose['th'] = math.atan2(math.sin(self.pose['th']),math.cos(self.pose['th'])) # squash the orientation to between -pi,pi
		self.pub_odometry(self.pose)
		self.pub_tf(self.pose)        



	def spin(self):
		rospy.loginfo("Start omnidrive_odom")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update();
			rate.sleep()
		rospy.spin()



	def shutdown(self):
		rospy.loginfo("Stop omnidrive_odom")
		rospy.sleep(1)



def main():
	odom_publisher = OdomPublisher();
	odom_publisher.spin()



if __name__ == '__main__':
	main(); 


