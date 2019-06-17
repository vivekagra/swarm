#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include <Keyboard.h>
#include <string.h>

ros::NodeHandle nh;
std_msgs::Float64MultiArray msg;

ros::Publisher encoder("encoder", &msg);

Encoder myEnc_fl(8, 9);
// pin settings
int pwm_fl = 2;
int pwm_fr = 3;
int pwm_rl = 4;
int pwm_rr = 5;

int INa_fl = 22;
int INb_fl = 23;
int INa_fr = 24;
int INb_fr = 25;
int INa_rl = 26;
int INb_rl = 27;
int INa_rr = 28;
int INb_rr = 29;


void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.advertise(encoder);
    msg.data = (float *)malloc(sizeof(float)*2);
    msg.data_length = 2;
    // set pins to output
    pinMode(pwm_fl, OUTPUT);
    pinMode(pwm_fr, OUTPUT);
    pinMode(pwm_rr, OUTPUT);
    pinMode(pwm_rl, OUTPUT);
    
    pinMode(INa_fl, OUTPUT);
    pinMode(INb_fl, OUTPUT);
    
    pinMode(INa_fr, OUTPUT);
    pinMode(INb_fr, OUTPUT);
    
    pinMode(INa_rl, OUTPUT);
    pinMode(INb_rl, OUTPUT);
    
    pinMode(INa_rr, OUTPUT);
    pinMode(INb_rr, OUTPUT);




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
void loop() {
    int i = 0;

    // increase speed of motor 
    for(i=0;i<256;i++)
    {
      // control motors
      control_motor(255, pwm_fl, INa_fl, INb_fl);
      control_motor(255, pwm_fr, INa_fr, INb_fr);
      control_motor(255, pwm_rl, INa_rl, INb_rl);
      control_motor(255, pwm_rr, INa_rr, INb_rr);

    }
}
