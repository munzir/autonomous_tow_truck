#include "include/steering_angle_sensing.h"

// Input is Duty cycle which varies from -255 to 255, beyond those values it stops. 0 is unavailable for use
int RPWM_Output = 6; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 5; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

int dutyCycle = 0;


void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  steering_angle_sensing_setup();
}

void loop()
{

  // pick input 

  // value is in the range -255 to 255
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (Serial.available() > 0)
  {
    dutyCycle = Serial.parseFloat();
    Serial.println(dutyCycle);
  }
  // Compute the PID output (duty cycle)


  // Compute the PID output (duty cycle)
 

  
  if (dutyCycle  == 0)
  {
    // Serial.println("STOP");
    digitalWrite(LPWM_Output, 1);
    digitalWrite(RPWM_Output, 1);
  }
  else if (dutyCycle < 0)
  {
    // reverse rotation
    // Serial.println("reverse");
    int reversePWM = -dutyCycle;
    digitalWrite(LPWM_Output, 1);
    analogWrite(RPWM_Output, 255-reversePWM);
  }
  else
  {
    // forward rotation
    // Serial.println("frwrd");
    int forwardPWM = dutyCycle;
    digitalWrite(RPWM_Output, 1);
    analogWrite(LPWM_Output, 255-forwardPWM);
  }

  Serial.print(steering_angle);
  Serial.print(", e0: ");
  Serial.print(edges_0);
  Serial.print(", ie0: ");
  Serial.print(interesting_edges_0);
  Serial.print(", cwe0: ");
  Serial.print(clockwise_edges_0);
  Serial.print(", acwe0: ");
  Serial.print(anti_clockwise_edges_0);
  Serial.print(", e1: ");
  Serial.print(edges_1);
  Serial.print(", ie1: ");
  Serial.print(interesting_edges_1);
  Serial.print(", cwe1: ");
  Serial.print(clockwise_edges_1);
  Serial.print(", acwe1: ");
  Serial.println(anti_clockwise_edges_1);
}
