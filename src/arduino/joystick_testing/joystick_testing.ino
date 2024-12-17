#include "include/joystick.h"

volatile long steering_angle = 0;

void setup()
{
  joystick_setup();
}

void loop() {
  // Send the value of steering_angle
  Serial.println (steering_angle);
  
  if (steering_angle>60){
    steering_angle=0;
    
  }
  else if (steering_angle<-60){
    steering_angle=0;
    
  }
  steering_angle++; //dummy data
  delay(100); //dummy delay
 
  JoystickLoop();  
  
  if (debug == true){
      Serial.print("Reinitialize: "); Serial.print(reinitialize);
      Serial.print(", Mode: "); Serial.print(mode);
      Serial.print(", Brake: "); Serial.print(brake);
      Serial.print(", Direction: "); Serial.print(direction);
      Serial.print(", Speed: "); Serial.print(ref_speed);
      Serial.print(", Steering Angle: "); Serial.println(ref_steering_angle);
      Serial.print(", Debug Mode: "); Serial.println(debug);
  }
}
 
