// Timer and Counter example
// Author: Nick Gammon
// Date: 17th January 2012

#ifndef WHEELS_SENSING_H
#define WHEELS_SENSING_H

// Input: Pin D5
// Note from Shaheer: I do not understand a good portion of this code,
// especially the work with interrupts, since I borrowed this from online to
// save time and effort. Perhaps I will set aside some time to understand and
// explain this, but for now, consider this as a black box that takes a typical
// digital signal (where 0-1 V is low and 4-5 V is high) and figures out its
// frequency. The larger the sampling time, the more accurate our frequency...
// but the less frequently we will be updated. This is not to say that low
// sampling time kills the accuracy, just that we will have to consider the
// average of the output data (for example, we are sending 50 Hz and the output
// constantly alternates between 0 and 100 Hz equally, averaging to 50 Hz).
// Trial and error is a good idea to find the sweet spot.
int sensingTickPeriod = 1; // by default we assume ticks to occur every 1 ms
const float pi = 3.14159265358979323846;
float last_frq = 0.0;
// these are checked for in the main program
volatile unsigned long
    timerCounts; // apparently the volatile keyword is to let the compiler know
                 // this can have very sharp changes? other than that, this is
                 // self explanatory
float frq;

// internal to counting routine
unsigned long overflowCount; // keeps track of overflows for Timer 1. will make
                             // more sense later
unsigned int timerTicks;
unsigned int timerPeriod;
bool frq_updated = false;

void startCounting(unsigned int ms) {
  timerPeriod = ms;     // how many 1 ms counts to do
  timerTicks = 0;       // reset interrupt counter
  overflowCount = 0;    // no overflows yet

  // reset Timer 1 and Timer 2
  TCCR1A = 0;
  TCCR1B = 0;

  // Timer 1 - counts events on pin D5
  TIMSK1 = bit(TOIE1); // interrupt on Timer 1 overflow

  TCNT1 = 0; // Both counters to zero

  // Reset prescalers
  GTCCR = bit(PSRASY); // reset prescaler now
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B = bit(CS10) | bit(CS11) | bit(CS12);
} // end of startCounting

ISR(TIMER1_OVF_vect) {
  ++overflowCount; // count number of Counter1 overflows
} // end of TIMER1_OVF_vect

void WheelsSensingSetup(int tickPeriod) {
  sensingTickPeriod = tickPeriod;
  startCounting(200); // Start counting for 10 ms (or another period)
} // end of setup

float CalculateAndThresholdFrequency() {
  // adjust counts by counting interval to give frequency in Hz
  frq = (timerCounts * 1000.0) /
        timerPeriod; // frequency is obtained by dividing the number of detected
                     // pulses by the period, then multiplying by 1000 because
                     // the period is in ms
  if (abs(last_frq - frq) > 900) {
    frq = last_frq;
  } else {
    last_frq = frq;
  }
  frq_updated = true;
  return frq;
}

void SensingLoop() {
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue =
      TCNT1; // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;

  // see if we have reached timing period
  timerTicks += sensingTickPeriod;
  if (timerTicks < timerPeriod)
    return; // not yet

  // if just missed an overflow
  if ((TIFR1 & bit(TOV1)) && timer1CounterValue < 256)
    overflowCopy++;

  // end of gate time, measurement ready

  TCCR1A = 0; // stop timer 1
  TCCR1B = 0;
  TIMSK1 = 0; // disable Timer1 Interrupt

  // calculate total count
  timerCounts =
      (overflowCopy << 16) + timer1CounterValue; // each overflow is 65536 more
  frq = CalculateAndThresholdFrequency();
  // restart counting
  startCounting(200); // Continue counting for 10 ms (or another period)
}

#endif // WHEELS_SENSING_H
