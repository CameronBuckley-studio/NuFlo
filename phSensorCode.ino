void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(A0, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float ph = (-5.76923 * voltage) + 21.48077;

  // print out the value you read:
  Serial.println(ph);
  delay(100);        // delay in between reads for stability
}
