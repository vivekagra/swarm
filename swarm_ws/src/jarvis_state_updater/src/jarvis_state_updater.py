#!/usr/bin/python
import rospy
import roslib

import math 
import numpy

# Messages
from std_msgs.msg import Float32

# Query Jarvis robot for front_left, front_right, rear_left and rear_right wheel encoders.
# Publish the estimated left and right angular wheel velocities
class WheelEncoderPublisher:
  def __init__(self):
    rospy.init_node('jarvis_state_updater')
    # Read in tangential velocity targets
    
    self.front_lwheel_angular_vel_motor_sub = rospy.Subscriber('front_lwheel_angular_vel_motor', Float32, self.front_lwheel_angular_vel_motor_callback)
    self.front_rwheel_angular_vel_motor_sub = rospy.Subscriber('front_rwheel_angular_vel_motor', Float32, self.front_rwheel_angular_vel_motor_callback)
    self.rear_lwheel_angular_vel_motor_sub  = rospy.Subscriber('rear_lwheel_angular_vel_motor' , Float32, self.rear_lwheel_angular_vel_motor_callback)
    self.rear_rwheel_angular_vel_motor_sub  = rospy.Subscriber('rear_rwheel_angular_vel_motor' , Float32, self.rear_rwheel_angular_vel_motor_callback)

    self.front_lwheel_angular_vel_control_pub = rospy.Subscriber('front_lwheel_angular_vel_control', Float32, self.front_lwheel_angular_vel_control_callback)
    self.front_rwheel_angular_vel_control_pub = rospy.Subscriber('front_rwheel_angular_vel_control', Float32, self.front_rwheel_angular_vel_control_callback)
    self.rear_lwheel_angular_vel_control_pub = rospy.Subscriber('rear_lwheel_angular_vel_control', Float32, self.rear_lwheel_angular_vel_control_callback)
    self.rear_rwheel_angular_vel_control_pub = rospy.Subscriber('rear_rwheel_angular_vel_control', Float32, self.rear_rwheel_angular_vel_control_callback)


    self.front_lwheel_angular_vel_enc_pub = rospy.Publisher('front_lwheel_angular_vel_enc', Float32, queue_size=10)
    self.front_rwheel_angular_vel_enc_pub = rospy.Publisher('front_rwheel_angular_vel_enc', Float32, queue_size=10)
    self.rear_lwheel_angular_vel_enc_pub = rospy.Publisher('rear_lwheel_angular_vel_enc', Float32, queue_size=10)
    self.rear_rwheel_angular_vel_enc_pub = rospy.Publisher('rear_rwheel_angular_vel_enc', Float32, queue_size=10)

    self.rate = rospy.get_param('~rate', 10)
    self.err_tick_incr = rospy.get_param('~err_tick_incr',20) # Filter out clearly erroneous encoder readings
    self.time_prev_update = rospy.Time.now();
    self.jarvis_on = rospy.get_param('~jarvis_on',True)
    if self.jarvis_on:
      import gopigo   
      self.front_lwheel_encs = [gopigo.enc_read(1)]*5
      self.front_rwheel_encs = [gopigo.enc_read(0)]*5
      self.rear_lwheel_encs  = [gopigo.enc_read(1)]*5
      self.rear_rwheel_encs  = [gopigo.enc_read(0)]*5

    self.R = rospy.get_param('~robot_wheel_radius', .03)

    # Need a little hack to incorporate direction wheels are spinning
    self.front_lwheel_dir = 1;
    self.front_rwheel_dir = 1;
    self.rear_lwheel_dir = 1;
    self.rear_rwheel_dir = 1;
    
    self.front_rwheel_angular_vel_control = 0;
    self.front_lwheel_angular_vel_control = 0;
    self.rear_rwheel_angular_vel_control = 0;
    self.rear_lwheel_angular_vel_control = 0;

########################################################## cross check because our encoder are incremental ###################################################
  # Really bad hack to get motor spin direction
  def front_lwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.front_lwheel_dir = 1
    else: self.front_lwheel_dir = -1
  # Really bad hack to get motor spin direction
  def front_rwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.front_rwheel_dir = 1
    else: self.front_rwheel_dir = -1
  # Really bad hack to get motor spin direction
  def rear_lwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.rear_lwheel_dir = 1
    else: self.rear_lwheel_dir = -1
  # Really bad hack to get motor spin direction
  def rear_rwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.rear_rwheel_dir = 1
    else: self.rear_rwheel_dir = -1


  def front_lwheel_angular_vel_control_callback(self,msg):
    self.front_lwheel_angular_vel_control = msg.data
  def front_rwheel_angular_vel_control_callback(self,msg):
    self.front_rwheel_angular_vel_control = msg.data
  def rear_lwheel_angular_vel_control_callback(self,msg):
    self.rear_lwheel_angular_vel_control = msg.data
  def rear_rwheel_angular_vel_control_callback(self,msg):
    self.rear_rwheel_angular_vel_control = msg.data

  # convert distance travelled by wheels to no of revolutions to radians
  def enc_2_rads(self,enc_cms):
    prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
    rads =  prop_revolution * (2*math.pi)
    return rads

  def update(self):
    if self.jarvis_on: # Running on actual robot
      import jarvis ################################### initially it was import gopigo ######################################
      #################### check here ###############################
      front_lwheel_enc = self.lwheel_dir * gopigo.enc_read(1) * .01 # cm's moved
      front_rwheel_enc = self.rwheel_dir * gopigo.enc_read(0) * .01 # cm's moved

      self.lwheel_encs = self.lwheel_encs[1:] + [lwheel_enc]
      self.rwheel_encs = self.rwheel_encs[1:] + [rwheel_enc]

      # History of past three encoder reading
      time_curr_update = rospy.Time.now()
      dt = (time_curr_update - self.time_prev_update).to_sec()

      # Compute angular velocity in rad/s
      lwheel_enc_delta = abs(self.lwheel_encs[-1]) - abs(self.lwheel_encs[-2])
      rwheel_enc_delta = abs(self.rwheel_encs[-1]) - abs(self.rwheel_encs[-2])
      lwheel_angular_vel_enc = self.enc_2_rads(lwheel_enc_delta) / dt
      rwheel_angular_vel_enc = self.enc_2_rads(rwheel_enc_delta) / dt

      # Adjust sign
      if self.lwheel_encs[-1] < 0: lwheel_angular_vel_enc = -lwheel_angular_vel_enc
      if self.rwheel_encs[-1] < 0: rwheel_angular_vel_enc = -rwheel_angular_vel_enc
      self.lwheel_angular_vel_enc_pub.publish(lwheel_angular_vel_enc)
      self.rwheel_angular_vel_enc_pub.publish(rwheel_angular_vel_enc)

      self.time_prev_update = time_curr_update

    else: # Running in simulation -- blindly copy from target assuming perfect execution
      self.lwheel_angular_vel_enc_pub.publish(self.lwheel_angular_vel_control)
      self.rwheel_angular_vel_enc_pub.publish(self.rwheel_angular_vel_control)
      

  def spin(self):
    rospy.loginfo("Start jarvis_state_updater")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop jarvis_state_updater")
    # Stop message
    self.lwheel_angular_vel_enc_pub.publish(0)
    self.rwheel_angular_vel_enc_pub.publish(0)
    rospy.sleep(1)

def main():
  encoder_publisher = WheelEncoderPublisher();
  encoder_publisher.spin()

if __name__ == '__main__':
  main(); 


