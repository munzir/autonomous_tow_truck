#include "include/wheels_control.h"
#include "include/wheels_sensing.h"
#include "include/joystick.h"
#include "include/timer2_10ms.h"

void setup() {
  WheelsSensingSetup(timer2TickPeriod);
  WheelsControlSetup(timer2TickPeriod);
  Timer2Reset();
  joystick_setup();
}

void loop() {
  JoystickLoop();  

  manual_mode = (mode == "Manual");
  wheel_speed = ref_speed;

  // Send the value of wheel speed
  Serial.println(frq, 2); // 2 decimal places for float
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

//********
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR(TIMER2_COMPA_vect) {
  SensingLoop();
  ControlLoop();
}
