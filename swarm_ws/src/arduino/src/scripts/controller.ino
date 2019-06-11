// This code basically commands my arduino to run 4 motors of Jarvis and send the feed back data to controller.
// The input to the code is pwm command and motn of dirxn for each motor and the outpur will be encoder data. 

/******************************** Check Time or delay for motorControl ****************************************************/
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>

ros::NodeHandle  nh;


// Encoder Instances
Encoder myEnc_fl(6, 7);
Encoder myEnc_fr(8, 9);
Encoder myEnc_rl(10, 11);
Encoder myEnc_rr(12, 13);

// pin settings
int pwmPin_fl = 2;
int pwmPin_bl = 3;
int pwmPin_fr = 4;
int pwmPin_br = 5;

int ENa_fl = 22;
int ENb_fl = 23;
int ENa_bl = 24;
int ENb_bl = 25;
int ENa_fr = 26;
int ENb_fr = 27;
int ENa_br = 28;
int ENb_br = 29;


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
	controlMotor(pwm.data,pwmPin_fl,Ena_fl,Enb_fl);
	pwm_fl = pwm.data;
}

void front_rwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_fr,Ena_fr,Enb_fr);
	pwm_fl = pwm.data;
}

void front_lwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_rl,Ena_rl,Enb_rl);
	pwm_fl = pwm.data;
}

void front_lwheel_cb(const std_msgs::Int16& pwm)
{
	controlMotor(pwm.data,pwmPin_rr,Ena_rr,Enb_rr);
	pwm_fl = pwm.data;
}


std_msgs::Float32MultiArray fl_msg;
std_msgs::Float32MultiArray fr_msg;
std_msgs::Float32MultiArray rl_msg;
std_msgs::Float32MultiArray rr_msg;


ros::Subsciber<std_msgs::Int16> motor_fl("front_lwheel_w_motor",&front_lwheel_cb);
ros::Subsciber<std_msgs::Int16> motor_fr("front_rwheel_w_motor",&front_rwheel_cb);
ros::Subsciber<std_msgs::Int16> motor_rl("rear_lwheel_w_motor" ,&rear_lwheel_cb );
ros::Subsciber<std_msgs::Int16> motor_rr("rear_rwheel_w_motor" ,&rear_rwheel_cb );


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
	
	t0 = millis();
	t1 = millis();
	
	while(t1-t0<1000)
	{
		newPos_fl = myEnc_fl.read();
		newPos_fr = myEnc_fr.read();
		newPos_rl = myEnc_rl.read();
		newPos_rr = myEnc_rr.read();
		t1 = millis();
	}

	rpm_fl = (float(newPos_fl-oldPos_fl))*60/280;
	rpm_fr = (float(newPos_fr-oldPos_fr))*60/280;
	rpm_rl = (float(newPos_rl-oldPos_rl))*60/280;
	rpm_rr = (float(newPos_rr-oldPos_rr))*60/280;

}
void setup()
{
	Serial.begin(57600);

	pinMode(pwm_fl,OUTPUT);
	pinMode(pwm_fl,OUTPUT);
	pinMode(pwm_rl,OUTPUT);
	pinMode(pwm_rr,OUTPUT);

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
