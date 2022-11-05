const int l0_pn = 2;   // left
const int l1_pn = 3;   // right
const int l2_pn = 4;   // far right (for juntion counting)
int l0, l1, l2;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(l0_pn, INPUT);
  pinMode(l1_pn, INPUT);
  pinMode(l2_pn, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  l0 = digitalRead(l0_pn);
  l1 = digitalRead(l1_pn);
  l2 = digitalRead(l2_pn);

  Serial.print("Sensor Readings: ");
  Serial.print(l0);
  Serial.print(l1);
  Serial.println(l2);
}
