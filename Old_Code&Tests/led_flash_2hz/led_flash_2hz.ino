// constants won't change. Used here to set a pin number:
const int ledA_pn = 10;  // 13 for testing (built-in led) --> change to 9 in actual main progrm

int ledAState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 250;  // 2hz --> 0.5 second interval at which to blink (milliseconds)

void setup() {
  // set the digital pin as output:
  pinMode(ledA_pn, OUTPUT);
}

void loop() {
  // here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledAState == LOW) {
      ledAState = HIGH;
    } else {
      ledAState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledA_pn, ledAState);
  }
}
