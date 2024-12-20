#include "include/wheels_sensing.h"
// Global variable to count interrupts
volatile bool timerFlag = false;
unsigned long current_time, myTime, prev_time = 0;

void Timer2Reset() {
  // Set Timer2 in CTC mode
  TCCR2A = (1 << WGM21);   // Configure Timer2 in CTC mode
  TCCR2B = (1 << CS22) | (1 << CS20); // Set prescaler to 128

  // Calculate OCR2A for 40ms interrupt
  // Formula: OCR2A = (Timer Frequency * Desired Time) / (Prescaler) - 1
  // Timer Frequency for Arduino Uno = 16MHz
  // OCR2A = (16,000,000 * 0.04) / 128 - 1 ≈ 124
  OCR2A = 186;  // Compare Match Value for 40ms

  // Enable Timer2 Compare Match Interrupt
  TIMSK2 = (1 << OCIE2A);

  // Enable global interrupts
  sei();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Frequency Counter"); // to show that we have initialized successfully
  int tickPeriod = 40;
  WheelsSensingSetup(tickPeriod);
  Timer2Reset();
}

void loop() {
  if (timerFlag) {
    timerFlag = false;  // Reset the flag

    current_time = millis();
    myTime = current_time - prev_time;
    prev_time = current_time;
    
    // Place your repeated code here
    Serial.println(" ");
    Serial.print(myTime);
    Serial.println(" ");
  }
}

// Timer2 Compare Match A ISR
ISR(TIMER2_COMPA_vect) {
  timerFlag = true;  // Set flag when the timer expires
  Timer2Reset();
}
