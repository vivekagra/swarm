# swarm

This Repository will contain all cad models, reading material, scripts and code that are used in making swarm squad



# jarvis_controller

Reads the target w velocity for each wheels and convert them to a pwm signal and send it to arduino.
Also reads angular velocity of each wheel and apply pid on it and modify the signal to reduce the error

Parameters - 
	i.    kp            = 1
	ii.   ki    		= 1
	iii.  kd    		= 1
	iv.   rate  		= 50
	v.    motor_max_w   = 62.8319
	vi.   motor_min_w   = 31.4159
	vii.  motor_cmd_max = 255
	viii. motor_cmd_min = 100
	ix.   pid_on        = False
	x.    controller_on = False

Published topics - 
	i.	 front_lwheel_w_control 
	ii.	 front_rwheel_w_control 
	iii. rear_lwheel_w_control
	iv.	 rear_rwheel_w_control
	v.	 front_lwheel_w_motor
	vi.	 front_rwheel_w_motor
	vii. rear_lwheel_w_motor
	viii.rear_rwheel_w_motor

Subsribing topics - 
	i.	 front_lwheel_w_enc
	ii.	 front_rwheel_w_enc
	iii. rear_lwheel_w_enc
	iv.	 rear_rwheel_w_enc
	v.	 front_lwheel_w_target
	vi.	 front_rwheel_w_target
	vii. rear_lwheel_w_target
	viii.rear_rwheel_w_target

Work Pending - 
	i.  vel to cmd is assumed linear change it after finding appropriate relation
	ii. Check pid integral



# jarvis_teleop

Command velocity and direction to bot to move in a desired direction

Parameters - 
	i.	speed = 0.5
	ii. turn  = 1.0

Published topics - 
	i.	cmd_vel



# mecanum_drive_controller

This package take target velocities in x y z diretion and convert them to angular velociy of individual wheels

Parameters - 
	i.	H    		 = 0.5
	ii. V    		 = 1.0
	iii.R    		 = 0.03
	iv. rate 		 = 50
	v.  timeout_idle = 2

Published topics - 
	i.	 front_lwheel_w_target 
	ii.	 front_rwheel_w_target 
	iii. rear_lwheel_w_target
	iv.	 rear_rwheel_w_target

Subsribing topics - 
	i.	 cmd_vel


# omnidrive_odometry

This package try to localize the robot based on odometry data

Parameters - 
	i.    H              = 0.125
	ii.   V    			 = 0.075
	iii.  R    			 = 0.03
	iv.   rate  		 = 50
	v.    N   			 = 280
	vi.   frame_id   	 = /odom
	vii.  child_frame_id = /base_link

Published topics - 
	i.	 cmd_vel_enc 
	ii.	 odom 

Subsribing topics - 
	i.	 front_lwheel_w_enc
	ii.	 front_rwheel_w_enc
	iii. rear_lwheel_w_enc
	iv.	 rear_rwheel_w_enc

Pending Work -
	Compute the odomtery


# arduino

Parameters - 
	i.    kp            = 1
	ii.   ki    		= 1
	iii.  kd    		= 1
	iv.   rate  		= 50
	v.    motor_max_w   = 62.8319
	vi.   motor_min_w   = 31.4159
	vii.  motor_cmd_max = 255
	viii. motor_cmd_min = 100
	ix.   pid_on        = False
	x.    controller_on = False

Published topics - 
	i.	 front_lwheel_w_enc
	ii.	 front_rwheel_w_enc
	iii. rear_lwheel_w_enc
	iv.	 rear_rwheel_w_enc


Subsribing topics - 
	i.	 front_lwheel_w_motor
	ii.	 front_rwheel_w_motor
	iii. rear_lwheel_w_motor
	iv.  rear_rwheel_w_motor
