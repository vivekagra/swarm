# swarm

This Repository will contain all cad models, reading material, scripts and code that are used in making swarm squad

## jarvis_state_updater

### Parameters - 
	1. rate          = 10
	2. err_tick_incr = 20
	3. jarvis_on     = True
	4. R		 = 0.03

### Published topics - 
	1.  front_lwheel_w_enc
	2.  front_rwheel_w_enc
	3.  rear_lwheel_w_enc
	4.  rear_rwheel_w_enc
	
### Subscribed topics - 
	1.  front_lwheel_w_control 
	2.  front_rwheel_w_control 
	3.  rear_lwheel_w_control
	4.  rear_rwheel_w_control
	5.  front_lwheel_w_motor
	6.  front_rwheel_w_motor
	7.  rear_lwheel_w_motor
	8.  rear_rwheel_w_motor

## jarvis_controller

Reads the target w velocity for each wheels and convert them to a pwm signal and send it to arduino.
Also reads angular velocity of each wheel and apply pid on it and modify the signal to reduce the error

### Parameters - 
	1.  kp            = 1 
	2.  ki    	  = 1
	3.  kd    	  = 1
	4.  rate  	  = 50
	5.  motor_max_w   = 62.8319
	6.  motor_min_w   = 31.4159
	7.  motor_cmd_max = 255
	8.  motor_cmd_min = 100
	9.  pid_on        = False
	10. controller_on = False

### Published topics - 
	1.  front_lwheel_w_control 
	2.  front_rwheel_w_control 
	3.  rear_lwheel_w_control
	4.  rear_rwheel_w_control
	5.  front_lwheel_w_motor
	6.  front_rwheel_w_motor
	7.  rear_lwheel_w_motor
	8.  rear_rwheel_w_motor

### Subsribing topics - 
	1.  front_lwheel_w_enc
	2.  front_rwheel_w_enc
	3.  rear_lwheel_w_enc
	4.  rear_rwheel_w_enc
	5.  front_lwheel_w_target
	6.  front_rwheel_w_target
	7.  rear_lwheel_w_target
	8.  rear_rwheel_w_target

### Work Pending - 
	1.  vel to cmd is assumed linear change it after finding appropriate relation
	2.  Check pid integral



## jarvis_teleop

Command velocity and direction to bot to move in a desired direction

### Parameters - 
	1.  speed = 0.5
	2.  turn  = 1.0

### Published topics - 
	1.  cmd_vel



## mecanum_drive_controller

This package take target velocities in x y z diretion and convert them to angular velociy of individual wheels

### Parameters - 
	1.  H		 = 0.5
	2.  V    	 = 1.0
	3.  R    	 = 0.03
	4.  rate 	 = 50
	5.  timeout_idle = 2

### Published topics - 
	1.  front_lwheel_w_target 
	2.  front_rwheel_w_target 
	3.  rear_lwheel_w_target
	4.  rear_rwheel_w_target

### Subsribing topics - 
	1.  cmd_vel


# omnidrive_odometry

This package try to localize the robot based on odometry data

### Parameters - 
	1.  H              = 0.125
	2.  V    	   = 0.075
	3.  R    	   = 0.03
	4.  rate  	   = 50
	5.  N   	   = 280
	6.  frame_id       = /odom
	7.  child_frame_id = /base_link

### Published topics - 
	1.  cmd_vel_enc 
	2.  odom 

### Subsribing topics - 
	1.  front_lwheel_w_enc
	2.  front_rwheel_w_enc
	3.  rear_lwheel_w_enc
	4.  rear_rwheel_w_enc

### Pending Work -
	Compute the odomtery


## arduino

### Parameters - 

### Published topics - 
	1. front_lwheel_w_enc
	2. front_rwheel_w_enc
	3. rear_lwheel_w_enc
	4. rear_rwheel_w_enc


### Subsribing topics - 
	1. front_lwheel_w_motor
	2. front_rwheel_w_motor
	3. rear_lwheel_w_motor
	4. rear_rwheel_w_motor
