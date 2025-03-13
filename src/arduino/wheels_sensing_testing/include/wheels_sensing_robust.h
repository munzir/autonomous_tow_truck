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
volatile unsigned long timerCounts;
long tempCounter = 0;
const uint16_t TIMER_THRESHOLD = 500;
unsigned int SENSING_LOOP_PERIOD = 50;
float frq;

unsigned int timerTicks;
unsigned int timerPeriod;
bool frq_updated = false;

void startCounting(unsigned int ms) {
  timerPeriod = ms;
  timerTicks = 0;
  tempCounter = 0;
}

void Timer1Reset() {
  TCNT1 = 0;
  TCCR1B |= (1 << CS11);
}

void ai1() { Timer1Reset(); }

ISR(TIMER1_COMPA_vect) {
  TCCR1B &= ~(1 << CS11);
  tempCounter++;
}

void WheelsSensingSetup(int tickPeriod) {
  sensingTickPeriod = tickPeriod;
  startCounting(SENSING_LOOP_PERIOD);
  pinMode(3, INPUT);
  attachInterrupt(1, ai1, RISING);
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = TIMER_THRESHOLD;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

float CalculateAndThresholdFrequency() {
  // Overriding frequency with dummy data
  frq = 0.0;
  frq_updated = true;
  return frq;
}

void SensingLoop() {
  timerTicks += sensingTickPeriod;
  if (timerTicks < timerPeriod)
    return;

  timerCounts = tempCounter;
  frq = CalculateAndThresholdFrequency();
  startCounting(SENSING_LOOP_PERIOD);
}

#endif // WHEELS_SENSING_ROBUST_H
