#define BUTTON_PIN 2
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;
bool flag_onoff= false;

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
}
void loop() {
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    byte buttonState = digitalRead(BUTTON_PIN);
    if (buttonState != lastButtonState) {
      lastTimeButtonStateChanged = millis();
      lastButtonState = buttonState;
      if (buttonState == LOW) {
        // do an action, for example print on Serial
        Serial.println("Button released");
        flag_onoff = !flag_onoff;
      }
    }
  }
  Serial.println(flag_onoff);
}
