/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Encoder.h>

ros::NodeHandle  nh;

std_msgs::Float32MultiArray fl_msg;
std_msgs::Float32MultiArray fr_msg;
std_msgs::Float32MultiArray rl_msg;
std_msgs::Float32MultiArray rr_msg;

ros::Publisher encoder_fl("encoder_fl", &fl_msg);
ros::Publisher encoder_fr("encoder_fr", &fr_msg);
ros::Publisher encoder_rl("encoder_rl", &rl_msg);
ros::Publisher encoder_rr("encoder_rr", &rr_msg);


// Encoder Instances
Encoder myEnc_fl(6, 7);
Encoder myEnc_fr(8, 9);
Encoder myEnc_rl(10, 11);
Encoder myEnc_rr(12, 13);


// pin settings
int pwm_fl = 2;
int pwm_bl = 3;
int pwm_fr = 4;
int pwm_br = 5;

int ENa_fl = 22;
int ENb_fl = 23;
int ENa_bl = 24;
int ENb_bl = 25;
int ENa_fr = 26;
int ENb_fr = 27;
int ENa_br = 28;
int ENb_br = 29;


// take initial enocder readings
long long oldPos_fl = myEnc_fl.read();
long long newPos_fl = myEnc_fl.read();

long long oldPos_fr = myEnc_fr.read();
long long newPos_fr = myEnc_fr.read();

long long oldPos_rl = myEnc_rl.read();
long long newPos_rl = myEnc_rl.read();

long long oldPos_rr = myEnc_rr.read();
long long newPos_rr = myEnc_rr.read();


// rate determines whether pwm increases or decreases
int pwm = -1;
int rate = 1;

// function to run a motor
void controlMotor(int rpm,int pwm,int ENa,int ENb)
{
	if(rpm>=0)
	{
		digitalWrite(ENa,HIGH);
		digitalWrite(ENb,LOW);
	}
	else
	{
		digitalWrite(ENa,LOW);
		digitalWrite(ENb,HIGH);
		rpm = -rpm;
	}
	analogWrite(pwm,rpm);
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

}


void loop()
{  

	//set pwm
	if(pwm>=255)
	{
		rate = -1;
	}
	else if(pwm<=-255)
	{
		rate = 1;
	}
	pwm = pwm + rate;
	
	// T1-T0 are time for running a pwm value
	long T0 = millis();
	long T1 = millis();

	// run motor for a pwm for 5 sec;
	while(T1-T0<5000)
	{
		// t0 and t1 are time after which we measure a rpm
		long t0 = millis();
		long t1 = millis();
		// measure rpm for every second
		while(t1-t0<1000)
		{
			controlMotor(pwm, pwm_fl, ENa_fl, ENb_fl);
			newPos_fl = myEnc_fl.read();		

			controlMotor(pwm, pwm_fr, ENa_fr, ENb_fr);
			newPos_fr = myEnc_fr.read();

			controlMotor(pwm, pwm_rl, ENa_rl, ENb_rl);
			newPos_rl = myEnc_rl.read();

			controlMotor(pwm, pwm_rr, ENa_rr, ENb_rr);
			newPos_rr = myEnc_rr.read();

			t1=millis();
		}

		// calculate rpm;
		float rpm_fl = (float(newPos_fl-oldPos_fl))*60/280;
		float rpm_fl = (float(newPos_fr-oldPos_fr))*60/280;
		float rpm_fl = (float(newPos_rl-oldPos_rl))*60/280;
		float rpm_fl = (float(newPos_rr-oldPos_rr))*60/280;

		// publish data
		fl_msg.data[0] = pwm;
		fr_msg.data[0] = pwm;
		rl_msg.data[0] = pwm;
		rr_msg.data[0] = pwm;

		fl_msg.data[1] = rpm_fl;
		fr_msg.data[1] = rpm_fr;
		rl_msg.data[1] = rpm_rl;
		rr_msg.data[1] = rpm_rr;

		encoder_fl.publish(&fl_msg);
		encoder_fr.publish(&fr_msg);
		encoder_rl.publish(&rl_msg);
		encoder_rr.publish(&rr_msg);

		nh.spinOnce();
		T1 = millis();
	}
}
