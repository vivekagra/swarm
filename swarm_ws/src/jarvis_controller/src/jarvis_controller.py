#!/usr/bin/python
import rospy
import roslib

# Messages
from std_msgs.msg import Float32
from std_msgs.msg import Int16

# Issue commands to the motors to achieve the target velocity
# Use a PID that compares the error based on encoder readings
class ControlsToMotors:
  def __init__(self):
    rospy.init_node('jarvis_controller')	
    self.rate = rospy.get_param('~rate', 50)
    
    # set pid constants
    self.Kp = rospy.get_param('~Kp', 1.0)
    self.Ki = rospy.get_param('~Ki', 1.0)
    self.Kd = rospy.get_param('~Kd', 1.0)


    """  CHANGE PARAMETERS  BY MEASUREMENT  """

    # Wheel can turn ~17 ticks per second which is approx 5.34 rad / s when motor_cmd = 255
    self.motor_max_w = rospy.get_param('~motor_max_w',62.8319) 
    # Wheel can turn ~6 ticks per second which is approx 5.34 rad / s when motor_cmd = 125
    self.motor_min_w = rospy.get_param('~motor_min_w',31.4159)
    
    # Corresponding motor commands
    self.motor_cmd_max = rospy.get_param('~motor_cmd_max',255)
    self.motor_cmd_min = rospy.get_param('~motor_cmd_min',100)

    self.pid_on = rospy.get_param('~pid_on',False)
    self.controller_on = rospy.get_param('~jarvis_on',False)
    
    if self.controller_on:
		print("Controller_on")
 	
	# Publish the computed w velocity after applying pid on  it
    #self.front_lwheel_w_target_pub = rospy.Publisher('front_lwheel_w_target', Float32, queue_size=10)
    #self.front_rwheel_w_target_pub = rospy.Publisher('front_rwheel_w_target', Float32, queue_size=10)
    #self.rear_lwheel_w_target_pub  = rospy.Publisher('rear_lwheel_w_target' , Float32, queue_size=10)
    #self.rear_rwheel_w_target_pub  = rospy.Publisher('rear_rwheel_w_target' , Float32, queue_size=10)

    # Publish the computed w velocity after applying pid on  it
    self.front_lwheel_w_control_pub = rospy.Publisher('front_lwheel_w_control', Float32, queue_size=10)
    self.front_rwheel_w_control_pub = rospy.Publisher('front_rwheel_w_control', Float32, queue_size=10)
    self.rear_lwheel_w_control_pub  = rospy.Publisher('rear_lwheel_w_control' , Float32, queue_size=10)
    self.rear_rwheel_w_control_pub  = rospy.Publisher('rear_rwheel_w_control' , Float32, queue_size=10)

    # Publish the computed w velocity control command
    self.front_lwheel_w_motor_pub = rospy.Publisher('front_lwheel_w_motor', Int16, queue_size=10)
    self.front_rwheel_w_motor_pub = rospy.Publisher('front_rwheel_w_motor', Int16, queue_size=10)
    self.rear_lwheel_w_motor_pub  = rospy.Publisher('rear_lwheel_w_motor' , Int16, queue_size=10)
    self.rear_rwheel_w_motor_pub  = rospy.Publisher('rear_rwheel_w_motor' , Int16, queue_size=10)

    # Read in encoders for PID control
    self.front_lwheel_w_enc_sub = rospy.Subscriber('front_lwheel_w_enc', Float32, self.front_lwheel_w_enc_callback)    
    self.front_rwheel_w_enc_sub = rospy.Subscriber('front_rwheel_w_enc', Float32, self.front_rwheel_w_enc_callback)    
    self.rear_lwheel_w_enc_sub  = rospy.Subscriber('rear_lwheel_w_enc',  Float32, self.rear_lwheel_w_enc_callback)
    self.rear_rwheel_w_enc_sub  = rospy.Subscriber('rear_rwheel_w_enc',  Float32, self.rear_rwheel_w_enc_callback)

    # Read in angular velocity targets
    self.front_lwheel_w_target_sub = rospy.Subscriber('front_lwheel_w_target', Float32, self.front_lwheel_w_target_callback)
    self.front_rwheel_w_target_sub = rospy.Subscriber('front_rwheel_w_target', Float32, self.front_rwheel_w_target_callback)
    self.rear_lwheel_w_target_sub = rospy.Subscriber('rear_lwheel_w_target', Float32, self.rear_lwheel_w_target_callback)
    self.rear_rwheel_w_target_sub = rospy.Subscriber('rear_rwheel_w_target', Float32, self.rear_rwheel_w_target_callback)

    # w velocity target
    self.front_lwheel_w_target = 0
    self.front_rwheel_w_target = 0
    self.rear_lwheel_w_target = 0
    self.rear_rwheel_w_target = 0
    
    # w velocity encoder readings
    self.front_lwheel_w_enc = 0
    self.front_rwheel_w_enc = 0
    self.rear_lwheel_w_enc = 0
    self.rear_rwheel_w_enc = 0

    # target velocity after applying control algorithm
    self.front_lwheel_control_cmd = 0
    self.front_rwheel_control_cmd = 0
    self.rear_lwheel_control_cmd = 0
    self.rear_rwheel_control_cmd = 0
    
    # motor commands
    self.front_lwheel_motor_cmd = 0
    self.front_rwheel_motor_cmd = 0
    self.rear_lwheel_motor_cmd = 0
    self.rear_rwheel_motor_cmd = 0
    
    # PID control variables
    self.front_lwheel_pid = {}
    self.front_rwheel_pid = {}
    self.rear_lwheel_pid = {}
    self.rear_rwheel_pid = {}


  # ==================================================
  # Read in angular velocity targets
  # ==================================================
  def front_lwheel_w_target_callback(self, msg):
    self.front_lwheel_w_target = msg.data

  def front_rwheel_w_target_callback(self, msg):
    self.front_rwheel_w_target = msg.data

  def rear_lwheel_w_target_callback(self, msg):
    self.rear_lwheel_w_target = msg.data

  def rear_rwheel_w_target_callback(self, msg):
    self.rear_rwheel_w_target = msg.data
 
  # ==================================================
  # Read in encoder readings for PID
  # ==================================================
  def front_lwheel_w_enc_callback(self, msg):
    self.front_lwheel_w_enc = msg.data

  def front_rwheel_w_enc_callback(self, msg):
    self.front_rwheel_w_enc = msg.data

  def rear_lwheel_w_enc_callback(self, msg):
    self.rear_lwheel_w_enc = msg.data

  def rear_rwheel_w_enc_callback(self, msg):
    self.rear_rwheel_w_enc = msg.data

    

  # PID control
  def pid_control(self,wheel_pid,target,state):
    # Initialize pid dictionary
    if len(wheel_pid) == 0:
      wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':0, 'error_prev':0,'error_curr':0})

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


  # Mapping w velocity targets to motor commands
  # Note: motor commands are ints between 0 - 255
  # We also assume motor commands are issues between motor_min_w and motor_max_w
  def wvel_2_motorcmd(self, w_target):
    if w_target == 0: return 0;
    slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_w - self.motor_min_w)
    intercept = self.motor_cmd_max - slope * self.motor_max_w

    if w_target > 0: # positive w velocity
      motor_cmd = slope * w_target + intercept
      if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
      if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
    
    else: # negative w velocity
      motor_cmd = slope * abs(w_target) + intercept
      if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
      if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
      motor_cmd = -motor_cmd      

    return motor_cmd


  def front_lwheel_update(self):
    
    # If we want to adjust target w velocity using PID controller to incorporate encoder readings
    #self.front_lwheel_w_target_pub.publish(self.front_lwheel_w_target)    
    if self.pid_on: 
      self.front_lwheel_w_target = self.pid_control(self.front_lwheel_pid, self.front_lwheel_w_target,self.front_lwheel_w_enc)
    self.front_lwheel_w_control_pub.publish(self.front_lwheel_w_target)

    # Compute motor command
    self.front_lwheel_motor_cmd = self.wvel_2_motorcmd(self.front_lwheel_w_target)
    self.front_lwheel_w_motor_pub.publish(self.front_lwheel_motor_cmd)    


  def front_rwheel_update(self):
    
    # If we want to adjust target w velocity using PID controller to incorporate encoder readings
    #self.front_rwheel_w_target_pub.publish(self.front_rwheel_w_target)    
    if self.pid_on: 
      self.front_rwheel_w_target = self.pid_control(self.front_rwheel_pid, self.front_rwheel_w_target,self.front_rwheel_w_enc)
    self.front_rwheel_w_control_pub.publish(self.front_rwheel_w_target)

    # Compute motor command
    self.front_rwheel_motor_cmd = self.wvel_2_motorcmd(self.front_rwheel_w_target)
    self.front_rwheel_w_motor_pub.publish(self.front_rwheel_motor_cmd)    


  def rear_lwheel_update(self):
    
    # If we want to adjust target w velocity using PID controller to incorporate encoder readings
    #self.rear_lwheel_w_target_pub.publish(self.rear_lwheel_w_target)    
    if self.pid_on: 
      self.rear_lwheel_w_target = self.pid_control(self.rear_lwheel_pid, self.rear_lwheel_w_target,self.rear_lwheel_w_enc)
    self.rear_lwheel_w_control_pub.publish(self.rear_lwheel_w_target)

    # Compute motor command
    self.rear_lwheel_motor_cmd = self.wvel_2_motorcmd(self.rear_lwheel_w_target)
    self.rear_lwheel_w_motor_pub.publish(self.rear_lwheel_motor_cmd)    


  def rear_rwheel_update(self):
    
    # If we want to adjust target w velocity using PID controller to incorporate encoder readings
    #self.rear_rwheel_w_target_pub.publish(self.rear_rwheel_w_target)    
    if self.pid_on: 
      self.rear_rwheel_w_target = self.pid_control(self.rear_rwheel_pid, self.rear_rwheel_w_target,self.rear_rwheel_w_enc)
    self.rear_rwheel_w_control_pub.publish(self.rear_rwheel_w_target)

    # Compute motor command
    self.rear_rwheel_motor_cmd = self.wvel_2_motorcmd(self.rear_rwheel_w_target)
    self.rear_rwheel_w_motor_pub.publish(self.rear_rwheel_motor_cmd)    


  # When given no commands for some time, do not move
  def spin(self):
    rospy.loginfo("Starting jarvis_controller")
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
    rospy.loginfo("Shutting down  jarvis_controller")
  	# Stop message
    #self.front_lwheel_w_target_pub.publish(0)
    #self.front_rwheel_w_target_pub.publish(0)
    #self.rear_lwheel_w_target_pub.publish(0)
    #self.rear_rwheel_w_target_pub.publish(0)
    
    self.front_lwheel_w_control_pub.publish(0)
    self.front_rwheel_w_control_pub.publish(0)
    self.rear_lwheel_w_control_pub.publish(0)
    self.rear_rwheel_w_control_pub.publish(0)
    
    self.front_lwheel_w_motor_pub.publish(0)
    self.front_rwheel_w_motor_pub.publish(0)
    self.rear_lwheel_w_motor_pub.publish(0)
    self.rear_rwheel_w_motor_pub.publish(0)
    
    rospy.sleep(1)        

def main():
  controls_to_motors = ControlsToMotors();
  controls_to_motors.spin()

if __name__ == '__main__':
  main(); 

