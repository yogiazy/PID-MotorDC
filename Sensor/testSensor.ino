#define SENSOR 2
#define VCC 13

volatile unsigned int pulseCount = 0; // pulse counter
unsigned long lastTime = 0; // last time a pulse was detected
unsigned int rpm = 0; // revolutions per minute

void setup() {
  pinMode(SENSOR, INPUT); // set digital pin 2 as input
  pinMode(VCC, OUTPUT);
  Serial.begin(9600); // initialize serial communication at 9600 bits per second
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseDetect, RISING);
}

void loop() {
  digitalWrite(VCC, HIGH);
  int rpm = readRPM();
  Serial.print("RPM: ");
  Serial.println(rpm); // print the RPM to the serial monitor
  delay(1000); // wait for 1 second
}

int readRPM() {
  if (millis() - lastTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(SENSOR)); // disable interrupt during RPM calculation
    rpm = pulseCount * 60 / 20; // calculate RPM (20 pulses per revolution)
    pulseCount = 0; // reset pulse counter
    lastTime = millis(); // record last measurement time
    attachInterrupt(digitalPinToInterrupt(SENSOR), pulseDetect, RISING); // re-attach interrupt to sensor pin
  }

  return rpm;
}

void pulseDetect() {
  pulseCount++; // increment pulse counter
}
