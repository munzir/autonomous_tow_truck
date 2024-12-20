#include "include/wheels_sensing.h"
#include "include/timer2_10ms.h"
/* code for tick period to be 1 ms */
unsigned long prev_time = 0;
void setup ()
{
  Serial.begin(9600); // serial is at baud rate 115200, which should be noted wherever else you use serial
  WheelsSensingSetup(timer2TickPeriod);
  Timer2Reset();
} // end of setup

void loop ()
{
  if (frq_updated){
    //unsigned long current_time = millis();
    //unsigned long my_time = current_time - prev_time;
    //prev_time = current_time;
    //Serial.print(my_time);
    //Serial.print(" ms, ");
    // Send pulses and frequency to serial, separated by a comma
    Serial.println(frq, 2);   // 2 decimal places for float
    
    frq_updated = false;
  }
}

//********
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR(TIMER2_COMPA_vect) {
  SensingLoop();
}
