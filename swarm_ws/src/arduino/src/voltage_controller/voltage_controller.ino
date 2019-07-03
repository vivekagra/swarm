/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

//#include <ros.h>
// #include <std_msgs/Float32MultiArray.h>
#include <Encoder.h>

// ros::NodeHandle  nh;

// std_msgs::Float32MultiArray msg;
//ros::Publisher encoder_fl("encoder", &msg);


// Encoder Instances
Encoder myEnc(10, 11);


// pin settings
int ENA = 4;

int IN1 = 26;
int IN2 = 27;

// take initial enocder readings
long long oldPos = myEnc.read();
long long newPos = myEnc.read();

// rate determines whether pwm increases or decreases
int pwm = 256;
int rate = 1;

// function to run a motor
void controlMotor(int pwm,int ENA,int IN1,int IN2)
{
	if(pwm>=0)
	{
		digitalWrite(IN1,HIGH);
		digitalWrite(IN2,LOW);
	}
	else
	{
		digitalWrite(IN1,LOW);
		digitalWrite(IN2,HIGH);
		pwm = -pwm;
	}
	analogWrite(ENA,pwm);
}


void setup()
{
	Serial.begin(57600);

	pinMode(ENA,OUTPUT);
	pinMode(IN1,OUTPUT);
	pinMode(IN2,OUTPUT);

	// msg.data = (float *)malloc(sizeof(float)*3);

	// msg.data_length = 3;

	// nh.initNode();
	// nh.advertise(encoder);

}


void loop()
{  

	//set pwm
	if(pwm>=256)
	{
		rate = -1;
	}
	else if(pwm<=-1)
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
   
    oldPos = myEnc.read();
    newPos = myEnc.read();
		while(t1-t0<1000)
		{
			controlMotor(pwm, ENA, IN1, IN2);
			newPos = myEnc.read();		
			t1=millis();
		}

		// calculate rpm;
		float rpm = (float(newPos-oldPos))*60/280;
		float voltage = analogRead(A7); 
		// // publish data
		// msg.data[0] = pwm;

		// msg.data[1] = rpm;
		Serial.print(pwm);Serial.print(" -> ");Serial.print(voltage);Serial.print(" -> ");Serial.println(rpm);
		// encoder.publish(&msg);
		// nh.spinOnce();
		T1 = millis();
	}
}