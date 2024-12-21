// Timer and Counter example
// Author: Nick Gammon
// Date: 17th January 2012

#ifndef WHEELS_SENSING_ROBUST_H
#define WHEELS_SENSING_ROBUST_H

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
long tempCounter = 0;
const uint16_t TIMER_THRESHOLD =
    500; // ~0.5ms seconds at 16MHz with prescaler 1024
unsigned int SENSING_LOOP_PERIOD = 50; // ms
float frq;

unsigned int timerTicks;
unsigned int timerPeriod;
bool frq_updated = false;

void startCounting(unsigned int ms) {
  timerPeriod = ms; // how many 1 ms counts to do
  timerTicks = 0;   // reset interrupt counter
  tempCounter = 0;
} // end of startCounting

void Timer1Reset() {
  TCNT1 = 0;             // Reset the timer counter
  TCCR1B |= (1 << CS11); // Restart the timer
}

void ai1() { Timer1Reset(); }

//
// Timer1 Compare Match Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  TCCR1B &= ~(1 << CS11); // Stop the timer by clearing the prescaler bits
  tempCounter++;
}

void WheelsSensingSetup(int tickPeriod) {
  sensingTickPeriod = tickPeriod;
  startCounting(SENSING_LOOP_PERIOD); // Start counting for 10 ms (or another period)
  pinMode(3, INPUT);  // internalเป็น pullup input pin 3
  attachInterrupt(1, ai1, RISING);
  // Configure Timer1
  cli();                   // Disable interrupts during configuration
  TCCR1A = 0;              // Normal mode, no PWM
  TCCR1B = 0;              // Reset Timer1 configuration
  TCNT1 = 0;               // Reset the counter
  OCR1A = TIMER_THRESHOLD; // Set the compare match value (threshold)
  TCCR1B |= (1 << WGM12);  // Enable CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);   // Set prescaler to 1024

  // Enable Timer1 compare match interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // Enable global
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
  // see if we have reached timing period
  timerTicks += sensingTickPeriod;
  if (timerTicks < timerPeriod)
    return; // not yet

  // calculate total count
  timerCounts = tempCounter;
  // ======
  frq = CalculateAndThresholdFrequency();
  // restart counting
  startCounting(SENSING_LOOP_PERIOD); // Continue counting for 10 ms (or another period)
}

#endif // WHEELS_SENSING_ROBUST_H
