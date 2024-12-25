// Control of EPS motor 
#ifndef STEERING_ANGLE_SENSING_H
#define STEERING_ANGLE_SENSING_H

#include <avr/interrupt.h>

// Timer threshold in clock ticks (16-bit max is 65535)
const uint16_t TIMER_THRESHOLD = 1500; // ~1ms seconds at 16MHz with prescaler 1024
volatile bool timerExpired = false;     // Flag to indicate timer interrupt occurred
bool ai0_triggered = false;
bool ai1_triggered = false;
volatile long steering_angle = 0; //This variable will increase or decrease depending on the rotation of encoder

volatile long edges_0, interesting_edges_0, clockwise_edges_0, anti_clockwise_edges_0 = 0;
volatile long edges_1, interesting_edges_1, clockwise_edges_1, anti_clockwise_edges_1 = 0;



void Timer1Reset() {
  TCNT1 = 0;                 // Reset the timer counter
  timerExpired = false;      // Reset the flag
  TCCR1B |= (1 << CS11);     // Restart the timer
}

void ai0_handler() {
  edges_0++;
  if (digitalRead(3) == LOW) {
    interesting_edges_0++;
      if (digitalRead(2) == HIGH) {
        anti_clockwise_edges_0++;
        steering_angle++;
      } else {
        clockwise_edges_0++;
        steering_angle--;
      }
  }
}

void ai1_handler() {
  edges_1++;
  if (digitalRead(2) == LOW) {
    interesting_edges_1++;
      if (digitalRead(3) == HIGH) {
        clockwise_edges_1++;
        steering_angle--;
      } else {
        anti_clockwise_edges_1++;
        steering_angle++;
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

// Timer1 Compare Match Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  timerExpired = true;           // Set the flag to indicate timer expiration
  TCCR1B &= ~(1 << CS11);        // Stop the timer by clearing the prescaler bits

  if (ai0_triggered) {
    ai0_handler();
    ai0_triggered = false;
  }

  if (ai1_triggered) {
    ai1_handler();
    ai1_triggered = false;
  }
  
}

void steering_angle_sensing_setup()
{
  pinMode(2, INPUT); // internal pullup input pin 2 
  
  pinMode(3, INPUT); // internalเป็น pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, CHANGE);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, CHANGE);

  // Configure Timer1
  cli();                  // Disable interrupts during configuration
  TCCR1A = 0;             // Normal mode, no PWM
  TCCR1B = 0;             // Reset Timer1 configuration
  TCNT1 = 0;              // Reset the counter
  OCR1A = TIMER_THRESHOLD; // Set the compare match value (threshold)
  TCCR1B |= (1 << WGM12); // Enable CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);  // Set prescaler to 1024

  // Enable Timer1 compare match interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();                  // Enable global 

}

#endif // STEERING_ANGLE_SENSING_H
