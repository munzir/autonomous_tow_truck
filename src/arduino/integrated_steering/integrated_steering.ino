#include "include/steering_angle_sensing.h"
#include "include/steering_control.h"
#include "include/joystick.h"
#include "include/timer2_1ms.h"
#include "include/brakes.h"

int control_timer_ticks = 0;
int control_timer_period = 40;
int prev_steering_angle = 999;
bool recalibration_completed = false;

//Global brake instance
struct Pins pins = {
  .dir = 9,
  .pul = 8,
  .ena = 10,
  //.relay = 10,
};

Brake brakes;

void recalibrate()
{
  recalibration_completed = false;
  prev_steering_angle = 999;
  while (abs(steering_angle - prev_steering_angle)>0)
  {
    prev_steering_angle = steering_angle;
    digitalWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, 180);
    delay(200);
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
  brake_setup(&pins);

  pinMode(13, OUTPUT); // Initializing onboard LED for testing; remove later
}


void loop()
{
  JoystickLoop();

  // gear switching
  if (reverse) {
    digitalWrite(4, LOW);
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
  }

  // braking
  if (brake) {
    press_brake(&brakes, &pins);
    digitalWrite(13, HIGH); // Turn on LED for testing; remove later
  } else {
    release_brake(&brakes, &pins);
    digitalWrite(13, LOW);  // Turn off LED for testing; remove later
  }

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
