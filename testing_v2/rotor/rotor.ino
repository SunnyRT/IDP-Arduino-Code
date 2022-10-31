// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 9;  //pin 11 in actual main


// Create a servo object 
Servo servo_claw; 
void setup() { 
   // We need to attach the servo to the used pin number 
   servo_claw.attach(servoPin); 
   Serial.begin(9600);
   servo_claw.write(0); 
   delay(1000); 
   Serial.print("go to 0");
}
void loop(){ 
   // Make servo go to 0 degrees 

   // Make servo go to 90 degrees 
   servo_claw.write(45); 
   delay(1000); 
   // Make servo go to 180 degrees 
   servo_claw.write(0); 
   delay(1000); 
}
