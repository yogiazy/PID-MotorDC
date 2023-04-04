#include <PID_v1.h>

#define POT_PIN A0
#define enA 9
#define in2 8
#define in1 7
#define SENSOR 2
#define VCC 13

volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
unsigned int rpm = 0;
double Setpoint, Input, Output;
double aggKp=0.0992, aggKi=0.2027212, aggKd=0,0031;
double consKp=0, consKi=0, consKd=0;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(SENSOR, INPUT);
  pinMode(VCC, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseDetect, RISING);

  Setpoint = 100;
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  digitalWrite(VCC, HIGH);
  int rpm = readRPM();
  Input = rpm;
  double gap = abs(Setpoint-Input);
  if(gap<10)
  { 
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();
  int pwmValue = Output;

  if (pwmValue > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmValue = -pwmValue;
  }

  analogWrite(enA, pwmValue);
  Serial.print("SET: ");
  Serial.print(Setpoint);
  Serial.print(", RPM: ");
  Serial.println(rpm);
  delay(1000);
}

int readRPM() {
  if (millis() - lastTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(SENSOR));
    rpm = pulseCount * 60 / 20;
    pulseCount = 0;
    lastTime = millis();
    attachInterrupt(digitalPinToInterrupt(SENSOR), pulseDetect, RISING); 
  }
  return rpm;
}

void pulseDetect() {
  pulseCount++;
}
