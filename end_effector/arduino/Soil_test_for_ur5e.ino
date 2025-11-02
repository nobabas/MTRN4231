#define sensorPin A0   // Analog input pin

// Define thresholds
#define airThreshold 600   // Above this value = sensor in air

void setup() {
  Serial.begin(9600);
  pinMode(outputPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int moisture = analogRead(sensorPin);
  Serial.println(moisture);

  if (moisture < airThreshold) {
    // Sensor is in soil
    // Change to this: digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    delay(1000);  
  } else {
    // Sensor is in air
    // Change to this digitalWrite(LED_BUILTIN, LOW); 
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000);  

  } 

  delay(3000);  // Check twice per second
}
