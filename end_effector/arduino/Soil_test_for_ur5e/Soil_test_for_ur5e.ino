/* Change these values based on your observations */
#define wetSoil 277   // Define max value we consider soil 'wet'
#define drySoil 380   // Define min value we consider soil 'dry'

// Define pins
#define sensorPin A0   // Analog input pin

void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Teensy is alive");
}

void loop() {
  // Read the Analog Input
  int moisture = analogRead(sensorPin);

  Serial.println(moisture);

  // Map the reading to PWM output range (0–255)
  // int outputValue = map(moisture, 0, 1023, 0, 255);


  //Serial.print("PWM Output Value: ");
  //Serial.println(outputValue);
  //Serial.println();

  delay(1000); // Take a reading every second
}