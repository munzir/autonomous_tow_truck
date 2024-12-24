#include "include/steering_angle_sensing.h"
#include "include/steering_control.h"
#include "include/joystick.h"
#include "include/timer2_1ms.h"

int control_timer_ticks = 0;
int control_timer_period = 40;
int prev_steering_angle = 999;
bool recalibration_completed = false;
void recalibrate()
{
  recalibration_completed = false;
  prev_steering_angle = 999;
  while (abs(steering_angle - prev_steering_angle)>1)
  {
    prev_steering_angle = steering_angle;
    digitalWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, 180);
    delay(100);
  }
  digitalWrite(LPWM_Output, 0);
  digitalWrite(RPWM_Output, 0);
  steering_angle = 56;
  recalibration_completed = true;
}

void setup()
{ 
  steering_control_setup();  
  joystick_setup();
  Timer2Reset();
}


void loop()
{
  JoystickLoop();  

  manual_mode = (manual == true);

  // Send the value of steering_angle
  if (recalibration_completed == true) { Serial.println (steering_angle); }
  if (reinitialize) { recalibrate(); }
  if (debug == true){
      Serial.print("Reinitialize: "); Serial.print(reinitialize);
      Serial.print(", Manual: "); Serial.print(manual);
      Serial.print(", Brake: "); Serial.print(brake);
      Serial.print(", Reverse: "); Serial.print(reverse);
      Serial.print(", Speed: "); Serial.print(ref_speed);
      Serial.print(", Steering Angle: "); Serial.println(ref_steering_angle);
      Serial.print(", Debug Mode: "); Serial.println(debug);
  }
}


ISR(TIMER2_COMPA_vect) {
  control_timer_ticks += timer2TickPeriod;
  if (control_timer_ticks < control_timer_period)
    return;

  // control logic
  if (recalibration_completed == true)
  {
  ControlLoop(ref_steering_angle);
  }
  control_timer_ticks = 0;
}
