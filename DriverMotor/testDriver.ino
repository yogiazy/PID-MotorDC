#define POT_PIN A0
#define enA 9
#define in2 8
#define in1 7

void setup() {
  Serial.begin(9600);
  pinMode(POT_PIN, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  int analogValue = analogRead(POT_PIN);
  int analogMap = map(analogValue, 0, 1023, 0, 255);
  digitalWrite(in1, HIGH); // control the motor's direction in clockwise
  digitalWrite(in2, LOW);  // control the motor's direction in clockwise
  analogWrite(enA, analogMap); // speed up
  Serial.print("POT :");
  Serial.println(analogMap);
  delay(3000);
}
