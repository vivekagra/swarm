

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Encoder.h>
ros::NodeHandle  nh;

std_msgs::Float32MultiArray float_msg;
ros::Publisher encoder("encoder", &float_msg);

Encoder myEnc(8, 9);
int pwm_FL = 2;
int ENa_FL = 22;
int ENb_FL = 23;

void controlMotor(int rpm,int pwmPin,int ENaPin,int ENbPin)
{
  if(rpm>=0)
  {
    digitalWrite(ENaPin,HIGH);
    digitalWrite(ENbPin,LOW);
  }
  else
  {
    digitalWrite(ENaPin,LOW);
    digitalWrite(ENbPin,HIGH);
    rpm = -rpm;
  }
  analogWrite(pwmPin,rpm);
}

void setup()
{
  Serial.begin(57600);
  pinMode(pwm_FL,OUTPUT);
  pinMode(ENa_FL,OUTPUT);
  pinMode(ENb_FL,OUTPUT);
  float_msg.data = (float *)malloc(sizeof(float)*2);
  float_msg.data_length = 2;
  nh.initNode();
  nh.advertise(encoder);
}
// T0 and T1 are global time for running a pwm value
// t0 and t1 are local time after which we measure a rpm
// rate determines whether pwm increases or decreases
int pwm = -1;
int rate = 1;
long T0 = millis();
long T1 = millis();

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
  // run a batch for a pwm;
  while(T1-T0<5000)
  {
    long t0 = millis();
    long t1 = t0;
    long oldPosition = myEnc.read();
    long newPosition = myEnc.read();
    // run a instance of a pwm
    while(t1-t0<1000)
    {
      controlMotor(pwm, pwm_FL, ENa_FL, ENb_FL);
      newPosition = myEnc.read();
      t1=millis();
    }

    // calculate rpm;
    long rpm = (newPosition-oldPosition)*60/280;
    
    // publish data
    //Serial.print(pwm);Serial.print(" ");Serial.println(rpm);
    float_msg.data[0] = pwm;
    float_msg.data[1] = rpm;
    encoder.publish( &float_msg );
    nh.spinOnce();
    T1 = millis();
  }
  T0 = millis();
  T1 = millis();
}
