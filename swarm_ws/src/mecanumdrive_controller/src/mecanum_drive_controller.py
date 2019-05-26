#!/usr/bin/python
import rospy
import roslib

# Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelToDiffDriveMotors:
  def __init__(self):
    rospy.init_node('diffdrive_controller')
    self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    self.front_lwheel_tangent_vel_target_pub = rospy.Publisher('front_lwheel_tangent_vel_target', Float32, queue_size=10)
    self.front_rwheel_tangent_vel_target_pub = rospy.Publisher('front_rwheel_tangent_vel_target', Float32, queue_size=10)
    self.rear_lwheel_tangent_vel_target_pub = rospy.Publisher('rear_lwheel_tangent_vel_target', Float32, queue_size=10)
    self.rear_rwheel_tangent_vel_target_pub = rospy.Publisher('rear_rwheel_tangent_vel_target', Float32, queue_size=10)
    
    # set geometry parameters for robot default parameters are given
    self.H = rospy.get_param('~robot_wheel_center_horizontal_sepration_distance', 0.125)
    self.V = rospy.get_param('~robot_wheel_center_vertical_separation_distance', 0.075) 
    self.R = rospy.get_param('~robot_wheel_radius', 0.03)

    self.rate = rospy.get_param('~rate', 50)
    self.timeout_idle = rospy.get_param('~timeout_idle', 2)
    self.time_prev_update = rospy.Time.now()

    self.target_vx = 0;
    self.target_vy = 0;
    self.target_w = 0;

  # When given no commands for some time, do not move
  def spin(self):
    rospy.loginfo("Start diffdrive_controller")
    rate = rospy.Rate(self.rate)
    time_curr_update = rospy.Time.now()
    
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
      if time_diff_update < self.timeout_idle: # Only move if command given recently
        self.update();
      rate.sleep()
    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop diffdrive_controller")
  	# Stop message

    self.front_lwheel_tangent_vel_target_pub.publish(0)
    self.front_rwheel_tangent_vel_target_pub.publish(0)
    self.rear_lwheel_tangent_vel_target_pub.publish(0)
    self.rear_rwheel_tangent_vel_target_pub.publish(0)

    rospy.sleep(1)    

  def update(self):
    v_fr = (1/self.R)*(self.target_vx + self.target_vy - (self.H + self.V)*self.target_w)
    v_fl = (1/self.R)*(self.target_vx - self.target_vy + (self.H + self.V)*self.target_w)
    v_rr = (1/self.R)*(self.target_vx + self.target_vy + (self.H + self.V)*self.target_w)
    v_rl = (1/self.R)*(self.target_vx - self.target_vy - (self.H + self.V)*self.target_w)
    
    self.front_rwheel_tangent_vel_target_pub.publish(v_fr)
    self.front_lwheel_tangent_vel_target_pub.publish(v_fl)
    self.rear_rwheel_tangent_vel_target_pub.publish(v_rr)
    self.rear_lwheel_tangent_vel_target_pub.publish(v_rl)

  def twistCallback(self,msg):
    self.target_vx = msg.linear.x;
    self.target_vy = msg.linear.y;
    self.target_w = msg.angular.z;
    self.time_prev_update = rospy.Time.now()


def main():
  cmdvel_to_motors = CmdVelToDiffDriveMotors();
  cmdvel_to_motors.spin()

if __name__ == '__main__':
  main(); 
