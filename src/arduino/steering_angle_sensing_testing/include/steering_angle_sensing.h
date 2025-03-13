#ifndef STEERING_ANGLE_SENSING_H
#define STEERING_ANGLE_SENSING_H

#include <avr/interrupt.h>

const uint16_t TIMER_THRESHOLD = 1500; // ~1ms seconds at 16MHz with prescaler 1024
volatile bool timerExpired = false;
bool ai0_triggered = false;
bool ai1_triggered = false;
volatile long steering_angle = 0;

volatile long edges_0, interesting_edges_0, clockwise_edges_0, anti_clockwise_edges_0 = 0;
volatile long edges_1, interesting_edges_1, clockwise_edges_1, anti_clockwise_edges_1 = 0;

void Timer1Reset() {
  TCNT1 = 0;
  timerExpired = false;
  TCCR1B |= (1 << CS11);
}

void ai0_handler() {
  edges_0++;
  if (digitalRead(3) == LOW) {
    interesting_edges_0++;
    if (digitalRead(2) == HIGH) {
      anti_clockwise_edges_0++;
      if (steering_angle < 55) steering_angle++;
    } else {
      clockwise_edges_0++;
      if (steering_angle > -55) steering_angle--;
    }
  }
}

void ai1_handler() {
  edges_1++;
  if (digitalRead(2) == LOW) {
    interesting_edges_1++;
    if (digitalRead(3) == HIGH) {
      clockwise_edges_1++;
      if (steering_angle > -55) steering_angle--;
    } else {
      anti_clockwise_edges_1++;
      if (steering_angle < 55) steering_angle++;
    }
  }
}

void ai0() {
  Timer1Reset();
  ai0_triggered = true;
}

void ai1() {
  Timer1Reset();
  ai1_triggered = true;
}

ISR(TIMER1_COMPA_vect) {
  timerExpired = true;
  TCCR1B &= ~(1 << CS11);

  if (ai0_triggered) {
    ai0_handler();
    ai0_triggered = false;
  }

  if (ai1_triggered) {
    ai1_handler();
    ai1_triggered = false;
  }
}

void steering_angle_sensing_setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(0, ai0, CHANGE);
  attachInterrupt(1, ai1, CHANGE);

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

#endif // STEERING_ANGLE_SENSING_H
