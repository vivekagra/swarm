import rospy
import roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ObstcaleAvoidance:
	def __init__(self):
		rospy.initnode('obstacle_avoidance')

		self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.rospy.Subscriber('dist_back', Float32, self.dist_back_callback)
		self.rospy.Subscriber('dist_right', Float32, self.dist_right_callback)
		self.rospy.Subscriber('dist_left', Float32, self.dist_left_callback)
		
    	self.vx_target = rospy.get_param('~vx_target',0)
    	self.vy_target = rospy.get_param('~vy_target',0.1)
    	self.w_target = rospy.get_param('~w_target',0) 
		# distance in metres
		self.dist_back = 4.0
		self.dist_right = 4.0
		self.dist_left = 4.0
		self.rate = rospy.get_param('~rate', 50)
		
	def dist_back_callback(self,msg):
		self.dist_back = msg.data

	def dist_right_callback(self,msg):
		self.dist_right = msg.data

	def dist_left_callback(self,msg):
		self.dist_left = msg.data
	
	def shutdown(self):
		rospy.loginfo("Jarvis Stopped moving")
		self.cmdvel_pub.publish(Twist())
		self.dist_back = 4.0
		self.dist_right = 4.0
		self.dist_left = 4.0
		rospy.sleep(1)

	def update(self):
		if(dist_back < 0.1):
			self.vx_target = self.vx_target
			self.vy_target = self.vy_target + 0.1
		if(dist_left < 0.1):
			self.vx_target = self.vx_target + 0.1
			self.vy_target = self.vy_target
		if(dist_right< 0.1):
			self.vx_target = self.vx_target - 0.1
			self.vy_target = self.vy_target
		move_cmd = Twist()
		move_cmd.linear.x = self.vx_target
		move_cmd.linear.y = self.vy_target
		move_cmd.linear.w = self.w_target
		self.cmdvel_pub.publish(move_cmd)
		
		
	def spin(self):
		rospy.loginfo("Jarvis start navigating and avoiding obstacles")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)		
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()			
		rospy.spin()

def main():
	jarvis_avoid_obstacle = ObstacleAvoidance()
	jarvis_avoid_obstacle.spin()
	
if __name__ == '__main__':
	main()
