#define hall_pn 8

void setup() {
  // put your setup code here, to run once:
 pinMode(hall_pn, INPUT);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(hall_pn));
}
