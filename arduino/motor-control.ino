void setup() {
  pinMode(7, OUTPUT); // Set pin 7 as output for the relay
  digitalWrite(7, HIGH); // Ensure relay starts in the OFF state for active-low relay
                        // Change to HIGH if your relay is active-high
  Serial.begin(9600); // Start serial communication at 9600 baud rate
}

void loop() {
  if (Serial.available()) { // Check if data is available to read
    char command = Serial.read(); // Read the incoming byte
    if (command == '0') {
      digitalWrite(7, HIGH); // Turn ON relay connected to pin 7
    } else if (command == '1') {
      digitalWrite(7, LOW); // Turn OFF relay
    }
    // Add more conditions for other functionalities as needed
  }
}
