// Control of EPS motor 

#include <avr/interrupt.h>

// Timer threshold in clock ticks (16-bit max is 65535)
const uint16_t TIMER_THRESHOLD = 1500; // ~1ms seconds at 16MHz with prescaler 1024
volatile bool eventOccurred = false;    // Flag to simulate an event in the loop
volatile bool timerExpired = false;     // Flag to indicate timer interrupt occurred
bool ai0_triggered = false;
bool ai1_triggered = false;

// Input is Duty cycle which varies from -255 to 255, beyond those values it stops. 0 is unavailable for use
int RPWM_Output = 6; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 5; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile long edges_0, interesting_edges_0, clockwise_edges_0, anti_clockwise_edges_0 = 0;
volatile long edges_1, interesting_edges_1, clockwise_edges_1, anti_clockwise_edges_1 = 0;
unsigned long myTime;
float setpoint = 0;
int dutyCycle = 0;
unsigned long lastDebounceTime0 = 0; 
unsigned long lastDebounceTime1 = 0; 

const unsigned long debounceDelay = 0; // 5 ms debounce time

// Define PID control constants
const float Kp = -21.3846124157446; // Proportional gain
const float Ki = -18.8004976592663; // Integral gain
const float Kd = -1.76724865606813;  // Derivative gain
// const float N = 14.908;    // Filter coefficient

// Time variables
unsigned long lastTime = 0;
float sampleTime = 0.001; // 10 ms (adjust as per your system)

// PID variables
float prevError = 0;
float integral = 0;
float prevDerivative = 0;
float output = 0;
float pidOutput = 0;
// 
// Integral clamping limits
const float integratorMin = -abs(255.0/Ki); // Minimum integrator limit (adjust as needed)
const float integratorMax = abs(255.0/Ki);  // Maximum integrator limit (adjust as needed)

// Function to compute the PID control
float computePID(float setpoint, float angle) {
  // Compute the error
  float error = setpoint - angle;

  // Compute time difference
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // in seconds
  // Serial.print("delta: ");
  // Serial.print(deltaTime);
  // Serial.print(",");
  if (deltaTime >= sampleTime) {
    // Proportional term
    float Pout = Kp * error;

    // Integral term
    integral += error * deltaTime;
    // Clamp the integral to prevent wind-up
    integral = constrain(integral, integratorMin, integratorMax);
    
    float Iout = Ki * integral;

    // Derivative term with filter213 
    float derivative = (error - prevError) / deltaTime;
    // float derivativeFiltered = (N*derivative + (prevDerivative)) / (1 + N * deltaTime);
    // float Dout = Kd * derivativeFiltered;
    float Dout = Kd*derivative;

    // Total output
    // Serial.print("Pout: ");
    // Serial.print(Pout);
    // Serial.print(",");
    // Serial.print("Iout: ");
    // Serial.print(Iout);
    // Serial.print(",");
    // Serial.print("Dout: ");
    // Serial.print(Dout);
    // Serial.print(",");
    output = Pout + Iout + Dout;

    // Update previous variables
    prevError = error;
    prevDerivative = derivative;
    // prevDerivative = derivativeFiltered;
    lastTime = currentTime;
  }
  return output;
}

  
void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
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

  lastTime = millis(); 
}

void loop()
{
  myTime = millis();

  // pick input 

  // value is in the range -255 to 255
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (Serial.available() > 0)
  {
    dutyCycle = Serial.parseFloat();
    Serial.println(dutyCycle);
  }
  // Compute the PID output (duty cycle)


  // Compute the PID output (duty cycle)
 

  
  if (dutyCycle  == 0)
  {
    // Serial.println("STOP");
    digitalWrite(LPWM_Output, 0);
    digitalWrite(RPWM_Output, 0);
  }
  else if (dutyCycle < 0)
  {
    // reverse rotation
    // Serial.println("reverse");
    int reversePWM = -dutyCycle;
    digitalWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
  }
  else
  {
    // forward rotation
    // Serial.println("frwrd");
    int forwardPWM = dutyCycle;
    digitalWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, forwardPWM);
  }

  Serial.print(counter);
  Serial.print(", e0: ");
  Serial.print(edges_0);
  Serial.print(", ie0: ");
  Serial.print(interesting_edges_0);
  Serial.print(", cwe0: ");
  Serial.print(clockwise_edges_0);
  Serial.print(", acwe0: ");
  Serial.print(anti_clockwise_edges_0);
  Serial.print(", e1: ");
  Serial.print(edges_1);
  Serial.print(", ie1: ");
  Serial.print(interesting_edges_1);
  Serial.print(", cwe1: ");
  Serial.print(clockwise_edges_1);
  Serial.print(", acwe1: ");
  Serial.println(anti_clockwise_edges_1);
}

void Timer1Reset() {
  TCNT1 = 0;                 // Reset the timer counter
  timerExpired = false;      // Reset the flag
  TCCR1B |= (1 << CS11);     // Restart the timer
}

void ai0_handler() {
  edges_0++;
  if (digitalRead(3) == LOW) {
    interesting_edges_0++;
    //if (millis() - lastDebounceTime0 > debounceDelay) {
      if (digitalRead(2) == HIGH) {
        anti_clockwise_edges_0++;
        counter++;
      } else {
        clockwise_edges_0++;
        counter--;
      }
      //lastDebounceTime0 = millis();
    //}
  }
}

void ai1_handler() {
  edges_1++;
  if (digitalRead(2) == LOW) {
    interesting_edges_1++;
    //if (millis() - lastDebounceTime1 > debounceDelay) {
      if (digitalRead(3) == HIGH) {
        clockwise_edges_1++;
        counter--;
      } else {
        anti_clockwise_edges_1++;
        counter++;
      }
      //lastDebounceTime1 = millis();
    //}
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
