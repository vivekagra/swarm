// This code basically commands my arduino to run 4 motors of Jarvis and send the feed back data to controller.
// The input to the code is pwm command and motn of dirxn for each motor and the outpur will be encoder data. 

/******************************** Check Time or delay for motorControl ****************************************************/
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

ros::NodeHandle  nh;


// Encoder Instances
Encoder myEnc_fl(6, 7);
Encoder myEnc_fr(8, 9);
Encoder myEnc_rl(10, 11);
Encoder myEnc_rr(12, 13);

// pin settings
int pwmPin_fl = 2;
int pwmPin_fr = 3;
int pwmPin_rl = 4;
int pwmPin_rr = 5;

int ENa_fl = 22;
int ENb_fl = 23;
int ENa_fr = 24;
int ENb_fr = 25;
int ENa_rl = 26;
int ENb_rl = 27;
int ENa_rr = 28;
int ENb_rr = 29;


float rpm_fl = 0;
float rpm_fr = 0;
float rpm_rl = 0;
float rpm_rr = 0;

int pwm_fl = 0;
int pwm_fr = 0;
int pwm_rl = 0;
int pwm_rr = 0;

// function to run a motor
void controlMotor(int pwmSignal,int pwmPin,int ENa,int ENb)
{
	if(pwmSignal>=0)
	{
		digitalWrite(ENa,HIGH);
		digitalWrite(ENb,LOW);
	}
	else
	{
		digitalWrite(ENa,LOW);
		digitalWrite(ENb,HIGH);
		pwmSignal = -pwmSignal;
	}
	analogWrite(pwmPin,pwmSignal);
}


void front_lwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_fl,ENa_fl,ENb_fl);
  Serial.print(pwm.data);
	pwm_fl = pwm.data;
}

void front_rwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_fr,ENa_fr,ENb_fr);
	pwm_fr = pwm.data;
}

void rear_lwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_rl,ENa_rl,ENb_rl);
	pwm_rl = pwm.data;
}

void rear_rwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_rr,ENa_rr,ENb_rr);
	pwm_rr = pwm.data;
}


std_msgs::Float32MultiArray fl_msg;
std_msgs::Float32MultiArray fr_msg;
std_msgs::Float32MultiArray rl_msg;
std_msgs::Float32MultiArray rr_msg;


ros::Subscriber<std_msgs::Int16> motor_fl("front_lwheel_w_motor",&front_lwheel_cb);
ros::Subscriber<std_msgs::Int16> motor_fr("front_rwheel_w_motor",&front_rwheel_cb);
ros::Subscriber<std_msgs::Int16> motor_rl("rear_lwheel_w_motor" ,&rear_lwheel_cb );
ros::Subscriber<std_msgs::Int16> motor_rr("rear_rwheel_w_motor" ,&rear_rwheel_cb );


ros::Publisher encoder_fl("encoder_fl", &fl_msg);
ros::Publisher encoder_fr("encoder_fr", &fr_msg);
ros::Publisher encoder_rl("encoder_rl", &rl_msg);
ros::Publisher encoder_rr("encoder_rr", &rr_msg);

void encoder()
{
	// encoder initial readings
	long long oldPos_fl = myEnc_fl.read();
	long long newPos_fl = myEnc_fl.read();

	long long oldPos_fr = myEnc_fr.read();
	long long newPos_fr = myEnc_fr.read();

	long long oldPos_rl = myEnc_rl.read();
	long long newPos_rl = myEnc_rl.read();

	long long oldPos_rr = myEnc_rr.read();
	long long newPos_rr = myEnc_rr.read();
	
	long long t0 = millis();
	long long t1 = millis();
	
	while(t1-t0<50)
	{
		newPos_fl = myEnc_fl.read();
		newPos_fr = myEnc_fr.read();
		newPos_rl = myEnc_rl.read();
		newPos_rr = myEnc_rr.read();
		t1 = millis();
	}

	rpm_fl = (float(newPos_fl-oldPos_fl))*1200/280;
	rpm_fr = (float(newPos_fr-oldPos_fr))*1200/280;
	rpm_rl = (float(newPos_rl-oldPos_rl))*1200/280;
	rpm_rr = (float(newPos_rr-oldPos_rr))*1200/280;

}
void setup()
{
	Serial.begin(57600);

	pinMode(pwmPin_fl,OUTPUT);
	pinMode(pwmPin_fl,OUTPUT);
	pinMode(pwmPin_rl,OUTPUT);
	pinMode(pwmPin_rr,OUTPUT);

	pinMode(ENa_fl,OUTPUT);
	pinMode(ENb_fl,OUTPUT);
	pinMode(ENa_fr,OUTPUT);
	pinMode(ENb_fr,OUTPUT);
	pinMode(ENa_rl,OUTPUT);
	pinMode(ENb_rl,OUTPUT);
	pinMode(ENa_rr,OUTPUT);
	pinMode(ENb_rr,OUTPUT);

	fl_msg.data = (float *)malloc(sizeof(float)*2);
	fr_msg.data = (float *)malloc(sizeof(float)*2);
	rl_msg.data = (float *)malloc(sizeof(float)*2);
	rr_msg.data = (float *)malloc(sizeof(float)*2);

	fl_msg.data_length = 2;
	fr_msg.data_length = 2;
	rl_msg.data_length = 2;
	rr_msg.data_length = 2;

	nh.initNode();
	
	nh.advertise(encoder_fl);
	nh.advertise(encoder_fr);
	nh.advertise(encoder_rl);
	nh.advertise(encoder_rr);

	nh.subscribe(motor_fl);
	nh.subscribe(motor_fr);
	nh.subscribe(motor_rl);
	nh.subscribe(motor_rr);	
}


void loop()
{   
	encoder();
	controlMotor(255,pwmPin_fl,ENa_fl,ENb_fl);
	fl_msg.data[0] = pwm_fl;
	fr_msg.data[0] = pwm_fr;
	rl_msg.data[0] = pwm_rl;
	rr_msg.data[0] = pwm_rr;

	fl_msg.data[1] = rpm_fl;
	fr_msg.data[1] = rpm_fr;
	rl_msg.data[1] = rpm_rl;
	rr_msg.data[1] = rpm_rr;
  
	encoder_fl.publish(&fl_msg);
	encoder_fr.publish(&fr_msg);
	encoder_rl.publish(&rl_msg);
	encoder_rr.publish(&rr_msg);
	nh.spinOnce();	
}
