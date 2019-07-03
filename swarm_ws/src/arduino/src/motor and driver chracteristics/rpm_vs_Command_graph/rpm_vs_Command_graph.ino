#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include <Keyboard.h>
#include <string.h>

ros::NodeHandle nh;
std_msgs::Float64MultiArray msg;

ros::Publisher encoder_fl("encoder_fl", &msg);
ros::Publisher encoder_fr("encoder_fr", &msg);
ros::Publisher encoder_rl("encoder_rl", &msg);
ros::Publisher encoder_rr("encoder_rr", &msg);

// encoder Instances
Encoder myEnc_fl(6, 7);
Encoder myEnc_fl(8, 9);
Encoder myEnc_rl(10, 11);
Encoder myEnc_rr(12, 13);

// pin settings
int pwm_fl = 2;
int pwm_bl = 3;
int pwm_fr = 4;
int pwm_br = 5;

int INa_fl = 22;
int INb_fl = 23;
int INa_bl = 24;
int INb_bl = 25;
int INa_fr = 26;
int INb_fr = 27;
int INa_br = 28;
int INb_br = 29;


void setup()
{
	Serial.begin(57600);
	nh.initNode();
	nh.advertise(encoder_fl);
	nh.advertise(encoder_fr);
	nh.advertise(encoder_rl);
	nh.advertise(encoder_rr);

	msg.data = (float *)malloc(sizeof(float)*2);
	msg.data_length = 2;

	// set pins to output
	pinMode(pwm_fl, OUTPUT);
	pinMode(pwm_bl, OUTPUT);
	pinMode(pwm_fr, OUTPUT);
	pinMode(pwm_br, OUTPUT);

	pinMode(INa_fl, OUTPUT);
	pinMode(INb_fl, OUTPUT);

	pinMode(INa_bl, OUTPUT);
	pinMode(INb_bl, OUTPUT);

	pinMode(INa_fr, OUTPUT);
	pinMode(INb_fr, OUTPUT);

	pinMode(INa_br, OUTPUT);
	pinMode(INb_br, OUTPUT);




}


// function to control motor
// speed is how fast the motor rotates
// Please set pwmPin, InaPin and INbPin for the motor you want to drive

void control_motor(int speed, int pwmPin, int INaPin, int INbPin){
    if(speed > 0){
        analogWrite(pwmPin, speed);
        digitalWrite(INaPin, HIGH);
        digitalWrite(INbPin, LOW);
    }
    else if(speed < 0){
        analogWrite(pwmPin, -speed);
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, HIGH);
    }
    else{
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, LOW);
    }
}

// In time loop, receive from serial and control 4 motors
// 280 count per revolutin is encoder precision

long oldPosition_fl  = myEnc_fl.read();
long oldPosition_fr  = myEnc_fl.read();
long oldPosition_rl  = myEnc_fl.read();
long oldPosition_rr  = myEnc_fl.read();

long p = oldPosition_fl;
long q = oldPosition_fr;
long r = oldPosition_rl;
long s = oldPosition_rr;

void loop() {
    int i = 0;

    // increase speed of motor 
    for(i=0;i<256;i++)
    {
      // control motors
      control_motor(100, pwm_fl, INa_fl, INb_fl);
      control_motor(0, pwm_bl, INa_bl, INb_bl);
      control_motor(0, pwm_fr, INa_fr, INb_fr);
      control_motor(0, pwm_br, INa_br, INb_br);

      int t0 = millis();
      int t1 = millis();
      long newPosition = myEnc_fl.read();
      x = newPosition;
      while(t1-t0<1000)
      {
        if(oldPosition!=newPosition)
        {
          oldPosition = newPosition;
        }
        t1 = millis();  
      }
      float rpm = ((newPosition -  x)*60)/float(280);
      msg.data[0] = 100;
      msg.data[1] = rpm;
      encoder.publish( &msg );  
    }
}
