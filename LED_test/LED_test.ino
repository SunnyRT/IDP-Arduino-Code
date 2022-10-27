

void setup() {
  // put your setup code here, to run once:
  pinMode(A3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(A3, 255);
  delay(1000);
  analogWrite(A3, 0);
  delay(1000);
}
