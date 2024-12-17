#ifndef JOYSTICK_H
#define JOYSTICK_H

bool reinitialize = false;
String mode = "";
bool brake = false;
String direction = "";
bool debug = false;
float ref_speed;
int ref_steering_angle = 0;

void joystick_setup() {
  Serial.begin(115200);
}
   
void JoystickLoop() {
  //Serial.println(steering_angle);
  if (Serial.available() > 0)
  {
   // Take in Serial input
    String input = Serial.readStringUntil('\n'); // Read the incoming string until a newline character

    // Split the input into individual parameters
    int lastIndex = 0;
    while (lastIndex < input.length()) {
      // Find the next comma
      int nextComma = input.indexOf(',', lastIndex);
      if (nextComma == -1) nextComma = input.length(); // Handle the last parameter

      // Extract the key-value pair
      String pair = input.substring(lastIndex, nextComma);
      pair.trim(); // Remove any leading/trailing whitespace

      // Parse the key-value pair
      if (pair.startsWith("Reinitialize = ")) {
        reinitialize = pair.substring(15).equalsIgnoreCase("True");
      } else if (pair.startsWith("Mode = ")) {
        mode = pair.substring(7);
      } else if (pair.startsWith("Brake = ")) {
        brake = pair.substring(8).equalsIgnoreCase("True");
      } else if (pair.startsWith("Direction = ")) {
        direction = pair.substring(12);
      } else if (pair.startsWith("Speed = ")) {
        ref_speed = pair.substring(8).toFloat();
      } else if (pair.startsWith("Steering Angle = ")) {
        ref_steering_angle = pair.substring(17).toInt();
      } else if (pair.startsWith("Debug Mode = ")){
        debug = pair.substring(13).equalsIgnoreCase("True");        
      }

      // Move to the next key-value pair
      lastIndex = nextComma + 1;
    }
  }
}

#endif // JOYSTICK_H

