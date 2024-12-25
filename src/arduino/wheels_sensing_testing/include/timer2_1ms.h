#ifndef TIMER2_1MS_H
#define TIMER2_1MS_H

int timer2TickPeriod = 1; // ms

void Timer2Reset() {
  TCCR2A = 0;
  TCCR2B = 0;
  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs.
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit(WGM21); // CTC mode
  OCR2A = 124;         // count up to 156  (zero relative!!!!)

  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit(OCIE2A); // enable Timer2 Interrupt

  TCNT2 = 0;
  // start Timer 2
  TCCR2B = bit(CS20) | bit(CS22); // prescaler of 128
} // end of TIMER2_COMPA_vect



#endif // TIMER2_1MS_H
