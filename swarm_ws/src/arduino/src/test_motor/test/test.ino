#include <Encoder.h>

// pin settings
int pwmPin_fl = 4;
int pwmPin_fr = 3;
int pwmPin_rl = 4;
int pwmPin_rr = 5;

int ENa_fl = 5;
int ENb_fl = 6;
int ENa_fr = 24;
int ENb_fr = 25;
int ENa_rl = 26;
int ENb_rl = 27;
int ENa_rr = 28;
int ENb_rr = 29;

int pwm_fl = 255;
int pwm_fr = 0;
int pwm_rl = 0;
int pwm_rr = 0;

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

void setup() {
  Serial.begin(57600);
  pinMode(pwmPin_fl,OUTPUT);
//  pinMode(pwmPin_fl,OUTPUT);
//  pinMode(pwmPin_rl,OUTPUT);
//  pinMode(pwmPin_rr,OUTPUT);

  pinMode(ENa_fl,OUTPUT);
  pinMode(ENb_fl,OUTPUT);
//  pinMode(ENa_fr,OUTPUT);
//  pinMode(ENb_fr,OUTPUT);
//  pinMode(ENa_rl,OUTPUT);
//  pinMode(ENb_rl,OUTPUT);
//  pinMode(ENa_rr,OUTPUT);
//  pinMode(ENb_rr,OUTPUT);


}

void loop() {
  controlMotor(255,pwmPin_fl,ENa_fl,ENb_fl);
//  controlMotor(255,pwmPin_fr,ENa_fr,ENb_fr);
//  controlMotor(255,pwmPin_rl,ENa_rl,ENb_rl);
//  controlMotor(255,pwmPin_rr,ENa_rr,ENb_rr);
  delay(1);
}
