int ENA  = 2;
int INA = 22;
int INB = 23;
int output = 0;
float battery_voltage = 12.0;
void setup() {
  Serial.begin(57600);
  pinMode(ENA, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  
}

void loop() {
  digitalWrite(INA,HIGH);
  // put your main code here, to run repeatedly:
  digitalWrite(INB,LOW);
  for(int i = 0;i<=255;i++)
  {
    analogWrite(ENA,255);
    output = battery_voltage*analogRead(A0)/1023;
    Serial.print("At pwm = ");Serial.print(i);Serial.print("output volatge is ");Serial.println(output);
    delay(1000);
  }
}
