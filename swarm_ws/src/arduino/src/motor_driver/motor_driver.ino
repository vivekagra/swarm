#include <string.h>

// pin settings
int pwm_fl = 3;
int pwm_bl = 4;
int pwm_fr = 5;
int pwm_br = 6;
int INa_fl = 22;
int INb_fl = 23;
int INa_bl = 24;
int INb_bl = 25;
int INa_fr = 26;
int INb_fr = 27;
int INa_bl = 28;
int INb_br = 29;


void setup() {
    Serial.begin(115200);

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

// In time loop, receive from serial and control 6 motors
void loop() {
    int i = 0;

    // increase speed of motor 
    for(i=0;i<256;i++)
    {
      // control motors
      control_motor(i, pwm_fl, INa_fl, INb_fl);
      control_motor(i, pwm_bl, INa_bl, INb_bl);
      control_motor(i, pwm_fr, INa_fr, INb_fr);
      control_motor(i, pwm_br, INa_br, INb_br);
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(i);
      delay(1000);
      
    }
    for(i=255;i>=0;i--)
    {
      // control motors
      control_motor(i, pwm_fl, INa_fl, INb_fl);
      control_motor(i, pwm_bl, INa_bl, INb_bl);
      control_motor(i, pwm_fr, INa_fr, INb_fr);
      control_motor(i, pwm_br, INa_br, INb_br);
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(i);
      delay(1000);
    }
    for(i=0;i>=-255;i--)
    {
      // control motors
      control_motor(i, pwm_fl, INa_fl, INb_fl);
      control_motor(i, pwm_bl, INa_bl, INb_bl);
      control_motor(i, pwm_fr, INa_fr, INb_fr);
      control_motor(i, pwm_br, INa_br, INb_br);
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(i);
      delay(1000);
      
    }
    for(i=-255;i<=0;i++)
    {
      // control motors
      control_motor(i, pwm_fl, INa_fl, INb_fl);
      control_motor(i, pwm_bl, INa_bl, INb_bl);
      control_motor(i, pwm_fr, INa_fr, INb_fr);
      control_motor(i, pwm_br, INa_br, INb_br);
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.print(i); Serial.print(",");
      Serial.println(i);
      delay(1000);
    }

}
