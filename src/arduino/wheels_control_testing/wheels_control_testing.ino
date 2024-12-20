#include "include/wheels_control.h"
#include "include/wheels_sensing.h"
#include "include/timer2_10ms.h"

void setup() {
  Serial.begin(9600);
  WheelsSensingSetup(timer2TickPeriod);
  WheelsControlSetup(timer2TickPeriod);
  Timer2Reset();
}

unsigned long prev_time = 0;
void loop() {
  if (Serial.available() > 0) {
    // Take in Serial input
    wheel_speed = Serial.parseFloat();
    // Serial.println(wheel_speed);
  }
  Serial.print("Setpoint:");
  Serial.print(wheel_speed);
  Serial.print(", Pout:");
  Serial.print(Pout);
  Serial.print(", Iout:");
  Serial.print(Iout);
  Serial.print(", Dout:");
  Serial.print(Dout);
  Serial.print(", PID:");
  Serial.print(output);
  Serial.print(", Speed:");
  Serial.println(frq, 2); // 2 decimal places for float
}

//********
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR(TIMER2_COMPA_vect) {
  SensingLoop();
  ControlLoop();
}
