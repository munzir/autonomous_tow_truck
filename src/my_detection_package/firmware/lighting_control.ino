// void setup() {
//   Serial.begin(9600);
//   pinMode(7, OUTPUT);  // Headlight pin
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char input = Serial.read();
//     if (input == '1') {
//       digitalWrite(7, HIGH);  // Turn on headlights
//     } else if (input == '0') {
//       digitalWrite(7, LOW);   // Turn off headlights
//     }
//   }
// }


void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);     // Headlight control pin
  pinMode(A2, INPUT);     // ADC2 as input for feedback
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '1') {
      digitalWrite(7, HIGH);  // Turn on headlight
      delay(100);             // Wait briefly for signal to propagate
      int feedback = analogRead(A2);
      if (feedback > 100) {   // Adjust threshold as needed
        Serial.println("ON_CONFIRMED");  // Send confirmation back to ROS
      } else {
        Serial.println("ON_FAILED");
      }

    } else if (input == '0') {
      digitalWrite(7, LOW);   // Turn off headlight
      delay(100);
      int feedback = analogRead(A2);
      if (feedback < 50) {
        Serial.println("OFF_CONFIRMED");
      } else {
        Serial.println("OFF_FAILED");
      }
    }
  }
}
