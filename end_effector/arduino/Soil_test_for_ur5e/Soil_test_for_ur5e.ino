// Teensy Moisture Sensor Bridge
// Reads soil moisture sensor on A0 and sends averaged analog readings (0–1023)
// to ROS2 via serial at 9600 baud.

#define MOISTURE_PIN A0       // Analog pin for soil moisture sensor
#define READ_INTERVAL 150     // Time between serial outputs (ms)
#define NUM_SAMPLES 10        // Number of samples for averaging

void setup() {
  Serial.begin(9600);         // Match teensy_bridge_node.py baud rate
  pinMode(MOISTURE_PIN, INPUT);
}

void loop() {
  long total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += analogRead(MOISTURE_PIN);
    delay(10);                 // Small delay between samples (for stability)
  }

  int averageValue = total / NUM_SAMPLES;  // Compute average
  Serial.println(averageValue);            // Send as integer (0–1023)
  delay(READ_INTERVAL);                    // Wait before next group of samples
}
