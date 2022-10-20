float moving_avg(float new_reading){

  const int nvalues = 20;               // Moving average window size
  static int current = 0;               // Index for current value
  static int value_count = 0;           // Count of values read (<= nvalues)
  static float sum = 0;                  // Rolling sum
  static float values[nvalues];

  sum += new_reading;

  // If the window is full, adjust the sum by deleting the oldest value
  if (value_count == nvalues)
    sum -= values[current];

  values[current] = new_reading;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (value_count < nvalues)
    value_count += 1;

  return sum/value_count;
}

const int ir1_pn = A5; // short range
int ir1;
float ir1_avg;



void setup(){
  Serial.begin(9600);
  pinMode(ir1_pn, INPUT); 

}

void loop(){
  // Serial.println("Test");
  ir1 = analogRead(ir1_pn);
  ir1_avg = moving_avg(ir1);
  Serial.println(ir1_avg);
  delay(2);
}