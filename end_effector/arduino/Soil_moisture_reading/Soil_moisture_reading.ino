/* Change these values based on your observations */
#define wetSoil 277   // Define max value we consider soil 'wet'
#define drySoil 380   // Define min value we consider soil 'dry'

#define MOISTURE_PIN A0       // Analog pin for soil moisture sensor
#define READ_INTERVAL 150     // Time between serial outputs (ms)
#define NUM_SAMPLES 10        // Number of samples for averaging

void setup() {
  Serial.begin(9600);         // Match teensy_bridge_node.py baud rate
  pinMode(MOISTURE_PIN, INPUT);
}

void loop() {
  // Read the Analog Input
  int moisture = analogRead(MOISTURE_PIN);
  Serial.println(moisture);

  delay(1000); // Take a reading every second
}
