// Define the ESP32 GPIO pin connected to the relay's signal (IN) pin
const int relayPin = 19; 

void setup() {
  // Configure the relay pin as an output
  pinMode(relayPin, OUTPUT);
}

void loop() {
  // Switch Relay OFF
  digitalWrite(relayPin, LOW); 
  delay(5000); // Wait for 5 seconds (5000 milliseconds)

  // Switch Relay ON
  digitalWrite(relayPin, HIGH); 
  delay(5000); // Wait for 5 seconds before repeating the cycle
}