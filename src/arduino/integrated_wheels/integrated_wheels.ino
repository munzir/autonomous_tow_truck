#include "include/joystick.h"
#include "include/timer2_1ms.h"
#include "include/wheels_control.h"
#include "include/wheels_sensing_robust.h"

unsigned long current_time, prev_time, iter_time = 0;

void setup() {
  WheelsSensingSetup(timer2TickPeriod);
  WheelsControlSetup(timer2TickPeriod);
  Timer2Reset();
  joystick_setup();
}

void loop() {
  JoystickLoop();

  manual_mode = manual;
  wheel_speed = ref_speed;

  if (ControlLoop()) {
    if (debug) {
      current_time = millis();
      iter_time = current_time - prev_time;
      prev_time = current_time;
    }
  }

  // Send the value of wheel speed
  if (frq_updated) {
    Serial.println(frq, 2);
    frq_updated = false;
  } // 2 decimal places for float
  if (debug == true) {
    Serial.print("Reinitialize: ");
    Serial.print(reinitialize);
    Serial.print(", Manual: ");
    Serial.print(manual);
    Serial.print(", Brake: ");
    Serial.print(brake);
    Serial.print(", Reverse: ");
    Serial.print(reverse);
    Serial.print(", Speed: ");
    Serial.print(ref_speed);
    Serial.print(", Steering Angle: ");
    Serial.print(ref_steering_angle);
    Serial.print(", Debug Mode: ");
    Serial.print(debug);
    Serial.print(", Cycle Time: ");
    Serial.println(iter_time);
  }
}

//********
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR(TIMER2_COMPA_vect) {
  SensingLoop();
  ControlTick();
}
