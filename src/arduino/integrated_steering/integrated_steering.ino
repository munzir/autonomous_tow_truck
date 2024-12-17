#include "include/steering_angle_sensing.h"
#include "include/steering_control.h"
#include "include/joystick.h"

void setup()
{ 
  steering_control_setup();  
  joystick_setup();

  // Set Timer2 in CTC mode
  TCCR2A = (1 << WGM21);   // Configure Timer2 in CTC mode
  TCCR2B = (1 << CS22) | (1 << CS20); // Set prescaler to 128

  // Calculate OCR2A for 40ms interrupt
  // Formula: OCR2A = (Timer Frequency * Desired Time) / (Prescaler) - 1
  // Timer Frequency for Arduino Uno = 16MHz
  // OCR2A = (16,000,000 * 0.04) / 128 - 1 ≈ 124
  OCR2A = 124;  // Compare Match Value for 40ms

  // Enable Timer2 Compare Match Interrupt
  TIMSK2 = (1 << OCIE2A);

  // Enable global interrupts
  sei();
}


void loop()
{
  JoystickLoop();  

  manual_mode = (mode == "Manual");

  // Send the value of steering_angle
  Serial.println (steering_angle);
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


ISR(TIMER2_COMPA_vect) {
  // control logic
  ControlLoop(ref_steering_angle);
}
