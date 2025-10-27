/* Change these values based on your observations */
#define wetSoil 277   // Define max value we consider soil 'wet'
#define drySoil 380   // Define min value we consider soil 'dry'

// Define pins
#define sensorPin A0   // Analog input pin
#define outputPin 9    // PWM output pin (must support PWM)

void setup() {
  Serial.begin(9600);
  pinMode(outputPin, OUTPUT);
}

void loop() {
  // Read the Analog Input
  int moisture = analogRead(sensorPin);
  Serial.print("Analog output: ");
  Serial.println(moisture);

  // Map the reading to PWM output range (0–255)
  int outputValue = map(moisture, 0, 1023, 0, 255);
  analogWrite(outputPin, outputValue);

  // Determine soil status
  if (moisture < wetSoil) {
    Serial.println("Status: Soil is too wet");
  } else if (moisture >= wetSoil && moisture < drySoil) {
    Serial.println("Status: Soil moisture is perfect");
  } else {
    Serial.println("Status: Soil is too dry - time to water!");
  }

  Serial.print("PWM Output Value: ");
  Serial.println(outputValue);
  Serial.println();

  delay(1000); // Take a reading every second
}
