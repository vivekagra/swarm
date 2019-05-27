#!/usr/bin/python
import rospy
import roslib

# Messages
from std_msgs.msg import Float32

# Issue commands to the GoPiGo motors to achieve the target velocity
# Use a PID that compares the error based on encoder readings
class ControlsToMotors:
  def __init__(self):
    rospy.init_node('gopigo_controller')
    self.rate = rospy.get_param('~rate', 50)
    
    # set pid constants
    self.Kp = rospy.get_param('~Kp', 1.0)
    self.Ki = rospy.get_param('~Ki', 1.0)
    self.Kd = rospy.get_param('~Kd', 1.0)


    """  CHANGE PARAMETERS  BY MEASUREMENT  """

    # Wheel can turn ~17 ticks per second which is approx 5.34 rad / s when motor_cmd = 255
    self.motor_max_angular_vel = rospy.get_param('~motor_max_angular_vel',5.32) 
    # Wheel can turn ~6 ticks per second which is approx 5.34 rad / s when motor_cmd = 125
    self.motor_min_angular_vel = rospy.get_param('~motor_min_angular_vel',1.28)
    
    # Corresponding motor commands
    self.motor_cmd_max = rospy.get_param('~motor_cmd_max',255)
    self.motor_cmd_min = rospy.get_param('~motor_cmd_min',110)

    self.R = rospy.get_param('~robot_wheel_radius', 0.03)
    self.pid_on = rospy.get_param('~pid_on',True)
    self.controller_on = rospy.get_param('~gopigo_on',False)
    
    if self.controller_on:
      import gopigo ############################## check it  ###############################
      import atexit ############################## check it  ###############################
      atexit.register(gopigo.stop)
    # (Optional) Publish the computed angular velocity targets
    self.fornt_lwheel_angular_vel_target_pub = rospy.Publisher('front_lwheel_angular_vel_target', Float32, queue_size=10)
    self.front_rwheel_angular_vel_target_pub = rospy.Publisher('front_rwheel_angular_vel_target', Float32, queue_size=10)
    self.rear_lwheel_angular_vel_target_pub  = rospy.Publisher('rear_lwheel_angular_vel_target' , Float32, queue_size=10)
    self.rear_rwheel_angular_vel_target_pub  = rospy.Publisher('rear_rwheel_angular_vel_target' , Float32, queue_size=10)
 
    # (Optional) Publish the computed angular velocity control command
    self.front_lwheel_angular_vel_control_pub = rospy.Publisher('front_lwheel_angular_vel_control', Float32, queue_size=10)
    self.front_rwheel_angular_vel_control_pub = rospy.Publisher('front_rwheel_angular_vel_control', Float32, queue_size=10)
    self.rear_lwheel_angular_vel_control_pub  = rospy.Publisher('rear_lwheel_angular_vel_control' , Float32, queue_size=10)
    self.rear_rwheel_angular_vel_control_pub  = rospy.Publisher('rear_rwheel_angular_vel_control' , Float32, queue_size=10)

    # (Optional) Publish the computed angular velocity motor command
    self.front_lwheel_angular_vel_motor_pub = rospy.Publisher('front_lwheel_angular_vel_motor', Float32, queue_size=10)
    self.front_rwheel_angular_vel_motor_pub = rospy.Publisher('front_rwheel_angular_vel_motor', Float32, queue_size=10)
    self.rear_lwheel_angular_vel_motor_pub  = rospy.Publisher('rear_lwheel_angular_vel_motor' , Float32, queue_size=10)
    self.rear_rwheel_angular_vel_motor_pub  = rospy.Publisher('rear_rwheel_angular_vel_motor' , Float32, queue_size=10)

    # Read in encoders for PID control
    self.front_lwheel_angular_vel_enc_sub = rospy.Subscriber('front_lwheel_angular_vel_enc', Float32, self.front_lwheel_angular_vel_enc_callback)    
    self.front_rwheel_angular_vel_enc_sub = rospy.Subscriber('front_rwheel_angular_vel_enc', Float32, self.front_rwheel_angular_vel_enc_callback)    
    self.rear_lwheel_angular_vel_enc_sub  = rospy.Subscriber('rear_lwheel_angular_vel_enc',  Float32, self.rear_lwheel_angular_vel_enc_callback)
    self.rear_rwheel_angular_vel_enc_sub  = rospy.Subscriber('rear_rwheel_angular_vel_enc',  Float32, self.rear_rwheel_angular_vel_enc_callback)

    # Read in tangential velocity targets
    self.front_lwheel_tangent_vel_target_sub = rospy.Subscriber('front_lwheel_tangent_vel_target', Float32, self.front_lwheel_tangent_vel_target_callback)
    self.front_rwheel_tangent_vel_target_sub = rospy.Subscriber('front_rwheel_tangent_vel_target', Float32, self.front_rwheel_tangent_vel_target_callback)
    self.rear_lwheel_tangent_vel_target_sub = rospy.Subscriber('rear_lwheel_tangent_vel_target', Float32, self.rear_lwheel_tangent_vel_target_callback)
    self.rear_rwheel_tangent_vel_target_sub = rospy.Subscriber('rear_rwheel_tangent_vel_target', Float32, self.rear_rwheel_tangent_vel_target_callback)

    # Tangential velocity target
    self.front_lwheel_tangent_vel_target = 0;
    self.front_rwheel_tangent_vel_target = 0;
    self.rear_lwheel_tangent_vel_target  = 0;
    self.rear_rwheel_tangent_vel_target  = 0;

    # Angular velocity target
    self.front_lwheel_angular_vel_target = 0
    self.front_rwheel_angular_vel_target = 0
    self.rear_lwheel_angular_vel_target = 0
    self.rear_rwheel_angular_vel_target = 0
    
    # Angular velocity encoder readings
    self.front_lwheel_angular_vel_enc = 0
    self.front_rwheel_angular_vel_enc = 0
    self.rear_lwheel_angular_vel_enc = 0
    self.rear_rwheel_angular_vel_enc = 0

    # PID control variables
    self.front_lwheel_pid = {}
    self.front_rwheel_pid = {}
    self.rear_lwheel_pid = {}
    self.rear_rwheel_pid = {}


  # ==================================================
  # Read in tangential velocity targets
  # ==================================================
  def front_lwheel_tangent_vel_target_callback(self, msg):
    self.front_lwheel_tangent_vel_target = msg.data

  def front_rwheel_tangent_vel_target_callback(self, msg):
    self.front_rwheel_tangent_vel_target = msg.data
   
  def rear_lwheel_tangent_vel_target_callback(self, msg):
    self.rear_lwheel_tangent_vel_target = msg.data

  def rear_rwheel_tangent_vel_target_callback(self, msg):
    self.rear_rwheel_tangent_vel_target = msg.data
 
  # ==================================================
  # Read in encoder readings for PID
  # ==================================================
  def front_lwheel_angular_vel_enc_callback(self, msg):
    self.front_lwheel_angular_vel_enc = msg.data

  def front_rwheel_angular_vel_enc_callback(self, msg):
    self.front_rwheel_angular_vel_enc = msg.data

  def rear_lwheel_angular_vel_enc_callback(self, msg):
    self.rear_lwheel_angular_vel_enc = msg.data

  def rear_rwheel_angular_vel_enc_callback(self, msg):
    self.rear_rwheel_angular_vel_enc = msg.data

  # ==================================================
  # Update motor commands
  # ==================================================

  # Compute angular velocity target
  def tangentvel_2_angularvel(self,tangent_vel):
    # v = wr
    # v - tangential velocity (m/s)
    # w - angular velocity (rad/s)
    # r - radius of wheel (m)
    angular_vel = tangent_vel / self.R;
    return angular_vel

  ################################# read it #############################
  # PID control
  def pid_control(self,wheel_pid,target,state):


    # Initialize pid dictionary
    if len(wheel_pid) == 0:
      wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0,'error_curr':0})

    wheel_pid['time_curr'] = rospy.Time.now()

    # PID control
    wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
    if wheel_pid['dt'] == 0: return 0

    wheel_pid['error_curr'] = target - state
    wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])]
    wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev'])/wheel_pid['dt']

    wheel_pid['error_prev'] = wheel_pid['error_curr']
    control_signal = (self.Kp*wheel_pid['error_curr'] + self.Ki*sum(wheel_pid['integral']) + self.Kd*wheel_pid['derivative'])
    target_new = target + control_signal

    # Ensure control_signal does not flip sign of target velocity
    if target > 0 and target_new < 0: target_new = target;
    if target < 0 and target_new > 0: target_new = target;

    if (target == 0): # Not moving
      target_new = 0
      return target_new

    wheel_pid['time_prev'] = wheel_pid['time_curr']
    return target_new

  # Mapping angular velocity targets to motor commands
  # Note: motor commands are ints between 0 - 255
  # We also assume motor commands are issues between motor_min_angular_vel and motor_max_angular_vel
  def angularvel_2_motorcmd(self, angular_vel_target):
    if angular_vel_target == 0: return 0;
    slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_angular_vel - self.motor_min_angular_vel)
    intercept = self.motor_cmd_max - slope * self.motor_max_angular_vel

    if angular_vel_target > 0: # positive angular velocity
      motor_cmd = slope * angular_vel_target + intercept
      if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
      if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
    
    else: # negative angular velocity
      motor_cmd = slope * abs(angular_vel_target) + intercept
      if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
      if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
      motor_cmd = -motor_cmd      

    return motor_cmd

  ############# decide various actions here for movement of robot ################
  # Send motor command to robot
  # motor1 for front_right wheel. motor1(0, ?) tells wheel to move backwards. motor1(1, ?) tells wheel to move forwards
  # motor2 for front_left wheel.
  # motor3 for rear_left wheel.
  # motor4 for rear_right wheel.
  def motorcmd_2_robot(self, wheel='front_left', motor_command=0):
    if self.gopigo_on:
      motor_command_raw = int(abs(motor_command))
      import gopigo
      if wheel == 'front_left':
        if motor_command >= 0: gopigo.motor2(1,motor_command_raw)
        elif motor_command < 0: gopigo.motor2(0,motor_command_raw)
      if wheel == 'front_right':
        if motor_command >= 0: gopigo.motor1(1,motor_command_raw)
        elif motor_command < 0: gopigo.motor1(0,motor_command_raw)
      if wheel == 'rear_left':
        if motor_command >= 0: gopigo.motor3(1,motor_command_raw)
        elif motor_command < 0: gopigo.motor3(0,motor_command_raw)
      if wheel == 'rear_right':
        if motor_command >= 0: gopigo.motor4(1,motor_command_raw)
        elif motor_command < 0: gopigo.motor4(0,motor_command_raw)

  def front_lwheel_update(self):
    # Compute target angular velocity
    self.front_lwheel_angular_vel_target = self.tangentvel_2_angularvel(self.front_lwheel_tangent_vel_target)
    self.front_lwheel_angular_vel_target_pub.publish(self.front_lwheel_angular_vel_target)
    
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on: 
      self.front_lwheel_angular_vel_target = self.pid_control(self.front_lwheel_pid, self.front_lwheel_angular_vel_target,self.front_lwheel_angular_vel_enc)
    self.front_lwheel_angular_vel_control_pub.publish(self.front_lwheel_angular_vel_target)

    # Compute motor command
    front_lwheel_motor_cmd = self.angularvel_2_motorcmd(self.front_lwheel_angular_vel_target)
    self.front_lwheel_angular_vel_motor_pub.publish(front_lwheel_motor_cmd)    

    # Send motor command
    self.motorcmd_2_robot('front_left',front_lwheel_motor_cmd)

  def front_rwheel_update(self):
    # Compute target angular velocity
    self.front_rwheel_angular_vel_target = self.tangentvel_2_angularvel(self.front_rwheel_tangent_vel_target)
    self.front_rwheel_angular_vel_target_pub.publish(self.front_rwheel_angular_vel_target)
    
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on: 
      self.front_rwheel_angular_vel_target = self.pid_control(self.front_rwheel_pid, self.front_rwheel_angular_vel_target,self.front_rwheel_angular_vel_enc)
    self.front_rwheel_angular_vel_control_pub.publish(self.front_rwheel_angular_vel_target)

    # Compute motor command
    front_rwheel_motor_cmd = self.angularvel_2_motorcmd(self.front_rwheel_angular_vel_target)
    self.front_rwheel_angular_vel_motor_pub.publish(front_rwheel_motor_cmd)    

    # Send motor command
    self.motorcmd_2_robot('front_right',front_rwheel_motor_cmd)

  def rear_lwheel_update(self):
    # Compute target angular velocity
    self.rear_lwheel_angular_vel_target = self.tangentvel_2_angularvel(self.rear_lwheel_tangent_vel_target)
    self.rear_lwheel_angular_vel_target_pub.publish(self.rear_lwheel_angular_vel_target)
    
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on: 
      self.lwheel_angular_vel_target = self.pid_control(self.rear_lwheel_pid, self.rear_lwheel_angular_vel_target,self.rear_lwheel_angular_vel_enc)
    self.rear_lwheel_angular_vel_control_pub.publish(self.rear_lwheel_angular_vel_target)

    # Compute motor command
    rear_lwheel_motor_cmd = self.angularvel_2_motorcmd(self.rear_lwheel_angular_vel_target)
    self.rear_lwheel_angular_vel_motor_pub.publish(rear_lwheel_motor_cmd)    

    # Send motor command
    self.motorcmd_2_robot('rear_left',rear_lwheel_motor_cmd)

  def rear_rwheel_update(self):
    # Compute target angular velocity
    self.rear_rwheel_angular_vel_target = self.tangentvel_2_angularvel(self.rear_rwheel_tangent_vel_target)
    self.rear_rwheel_angular_vel_target_pub.publish(self.rear_rwheel_angular_vel_target)
    
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on: 
      self.rear_rwheel_angular_vel_target = self.pid_control(self.rear_rwheel_pid, self.rear_rwheel_angular_vel_target,self.rear_rwheel_angular_vel_enc)
    self.rear_rwheel_angular_vel_control_pub.publish(self.rear_rwheel_angular_vel_target)

    # Compute motor command
    rear_rwheel_motor_cmd = self.angularvel_2_motorcmd(self.rear_rwheel_angular_vel_target)
    self.rear_rwheel_angular_vel_motor_pub.publish(rear_rwheel_motor_cmd)    

    # Send motor command
    self.motorcmd_2_robot('rear_right',rear_rwheel_motor_cmd)

  # When given no commands for some time, do not move
  def spin(self):
    rospy.loginfo("Start motor_controller")
    rate = rospy.Rate(self.rate)
    
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.front_rwheel_update()
      self.front_lwheel_update()
      self.rear_rwheel_update()
      self.rear_lwheel_update()
      rate.sleep()
    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop motor_controller")
  	# Stop message
    self.front_lwheel_angular_vel_target_pub.publish(0)
    self.front_rwheel_angular_vel_target_pub.publish(0)
    self.rear_lwheel_angular_vel_target_pub.publish(0)
    self.rear_rwheel_angular_vel_target_pub.publish(0)
    
    self.front_lwheel_angular_vel_control_pub.publish(0)
    self.front_rwheel_angular_vel_control_pub.publish(0)
    self.rear_lwheel_angular_vel_control_pub.publish(0)
    self.rear_rwheel_angular_vel_control_pub.publish(0)
    
    self.front_lwheel_angular_vel_motor_pub.publish(0)
    self.front_rwheel_angular_vel_motor_pub.publish(0)
    self.rear_lwheel_angular_vel_motor_pub.publish(0)
    self.rear_rwheel_angular_vel_motor_pub.publish(0)
    
    rospy.sleep(1)        

def main():
  controls_to_motors = ControlsToMotors();
  controls_to_motors.spin()

if __name__ == '__main__':
  main(); 

