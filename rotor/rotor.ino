
#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
 
Servo myservo;  

// Control and feedback pins
int servoPin = 9;
int feedbackPin = A0;

// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int tolerance = 2; // max feedback measurement error


void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.write(minPos);
  minDegrees = minPos;
  delay(2000); // make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  
  // Move to the maximum position and record the feedback value
  servo.write(maxPos);
  maxDegrees = maxPos;
  delay(2000); // make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
}


 
void setup() 
{ 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  Serial.begin(115200);
  
  myservo.attach(servoPin); 
  
  calibrate(myservo, feedbackPin, 0, 270);  // calibrate for the 20-270 degree range
 
  displayPosition(); 
    
} 

void loop()
{

  delay(3000);
  myservo.write(0);
  delay(2000);
  displayPosition(); 

  delay(3000);
  myservo.write(90);
  delay(2000);
  displayPosition();


  delay(3000);
  myservo.write(180);
  delay(2000);
  displayPosition();

  delay(3000);
  myservo.write(270);
  delay(2000);
  displayPosition(); 
  
  
}
int getPos(int analogPin)
{
  return abs(map(analogRead(analogPin), minFeedback, maxFeedback, minDegrees, maxDegrees));
}
void displayPosition()
{
  
  
   display.clearDisplay();

  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Position");
  
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(10,30);
  display.println( getPos(feedbackPin) );
  display.display();
  
}