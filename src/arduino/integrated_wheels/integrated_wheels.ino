#include "include/joystick.h"
#include "include/timer2_1ms.h"
#include "include/wheels_control.h"
#include "include/wheels_sensing_robust.h"
#include "include/brakes.h"

unsigned long current_time, prev_time, iter_time = 0;
bool prev_brake_state = false;

//Global brake instance
struct Pins pins = {
  .dir = 9,
  .pul = 8,
  .ena = 10,
  //.relay = 10,
};

Brake brakes;

void setup() {
  WheelsSensingSetup(timer2TickPeriod);
  WheelsControlSetup(timer2TickPeriod);
  Timer2Reset();
  joystick_setup();
  brake_setup(&pins);

  pinMode(13, OUTPUT); // Initializing onboard LED for testing; remove later
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

  // braking (Call press_brake or release_brake only on state change)
  if (brake != prev_brake_state) {
    if (brake) {
      press_brake(&brakes, &pins);
      // digitalWrite(13, HIGH); // Turn onboard LED ON (use for troubleshooting)
    } else {
      release_brake(&brakes, &pins);
      // digitalWrite(13, LOW);  // Turn onbaord LED OFF (use for troubleshooting)
    }
    prev_brake_state = brake; // Update previous state
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
