#include "include/steering_control.h"
#include "include/steering_angle_sensing.h"

void setup()
{   
  Serial.begin(115200);    
  steering_control_setup();  
}

void loop()
{
  myTime = millis();

  // pick input 

  // value is in the range -255 to 255
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (Serial.available() > 0)
  {
   // Take in Serial input
    setpoint = Serial.parseFloat();
  }
  
  // control logic
  ControlLoop(setpoint);
  
  // if ( myTime<17000)
  // {
    Serial.print("Time: ");
    Serial.print(myTime);
    Serial.print(", ");
    Serial.print("Angle: ");
    Serial.print(steering_angle);
    Serial.print(", ");
    Serial.print("DutyCycle: ");
    Serial.println(dutyCycle);
  // }
  // Serial.print(", e0: ");
  // Serial.print(edges_0);
  // Serial.print(", ie0: ");
  // Serial.print(interesting_edges_0);
  // Serial.print(", cwe0: ");
  // Serial.print(clockwise_edges_0);
  // Serial.print(", acwe0: ");
  // Serial.print(anti_clockwise_edges_0);
  // Serial.print(", e1: ");
  // Serial.print(edges_1);
  // Serial.print(", ie1: ");
  // Serial.print(interesting_edges_1);
  // Serial.print(", cwe1: ");
  // Serial.print(clockwise_edges_1);
  // Serial.print(", acwe1: ");
  // Serial.println(anti_clockwise_edges_1);
}
