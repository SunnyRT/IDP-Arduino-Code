#define hall_pn 8
#define ledG_pn A2
#define ledR_pn A3


bool hall = false;
int box_intend = 1;
bool flag_detected = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(hall_pn, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (flag_detected == false) 
  {
    for (int i = 0; i < 10; i++) //obtain 10 readings from hall effect sensor
    {
      delay(100);
      hall = digitalRead(hall_pn);
      Serial.println(hall);
      if (hall == HIGH) 
      {
        box_intend = 3;
      }
    }
    flag_detected == true;
    Serial.println("finish detecting!");
    Serial.print("box_intend: ");
    Serial.println(box_intend);
    if (box_intend == 3) //magnet present
    {
      analogWrite(ledR_pn, 255);
      delay(5000);
      analogWrite(ledR_pn, 0);
    }
    else //no magnet present
    {
      analogWrite(ledG_pn, 255);
      delay(5000);
      analogWrite(ledG_pn, 0);
    }
    
    
    delay(1000);
    box_intend = 1;
  }


}
