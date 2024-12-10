// Control of EPS motor 

// Timer threshold in clock ticks (16-bit max is 65535)
const uint16_t TIMER_THRESHOLD = 2000; // ~1ms seconds at 16MHz with prescaler 1024
volatile bool eventOccurred = false;    // Flag to simulate an event in the loop
volatile bool timerExpired = false;     // Flag to indicate timer interrupt occurred
bool ai0_triggered = false;
bool ai1_triggered = false;
volatile long edges_0, interesting_edges_0, clockwise_edges_0, anti_clockwise_edges_0 = 0;
volatile long edges_1, interesting_edges_1, clockwise_edges_1, anti_clockwise_edges_1 = 0;

// Input is Duty cycle which varies from -255 to 255, beyond those values it stops. 0 is unavailable for use
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
unsigned long myTime;
float setpoint = 0;
int dutyCycle = 0;
unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 5; // 5 ms debounce time
// PID constants
float alpha = 0.1;      // Smoothing factor for the derivative (0 < alpha < 1)

// Variables for PID calculation
float previousError = 0.0;
float integral = 0.0;
float smoothedDerivative = 0.0;
unsigned long lastTime = 0;
bool reinitialize = false;
String mode = "";
bool brake = false;
String direction = "";
String debug = "";
float receive;
int steering_angle = 0;


// Define PID control constants
const float Kp = -21.3846124157446; // Proportional gain
const float Ki = -18.8004976592663; // Integral gain
const float Kd = -1.76724865606813;  // Derivative gain
// const float N = 14.908;    // Filter coefficient

// Time variables
float sampleTime = 0.001; // 10 ms (adjust as per your system)

// PID variables
float prevError = 0;
float prevDerivative = 0;
float output = 0;
float pidOutput = 0;
// 
// Integral clamping limits
const float integratorMin = -abs(255.0/Ki); // Minimum integrator limit (adjust as needed)
const float integratorMax = abs(255.0/Ki);  // Maximum integrator limit (adjust as needed)


float PID(float setpoint, float measuredValue) {
    // Calculate time elapsed
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    // Calculate the error term
    float error = setpoint - measuredValue;

    // Proportional term
    float Pout = Kp * error;

    // Integral term with clamping
    integral += error * elapsedTime;
    if (integral > 0.4) integral = 0.4;         // Upper clamp
    else if (integral < -0.4) integral = -0.4;  // Lower clamp
    float Iout = Ki * integral;

    // Derivative term with smoothing
    float rawDerivative = (error - previousError) / elapsedTime;
    smoothedDerivative = alpha * rawDerivative + (1 - alpha) * smoothedDerivative;
    float Dout = Kd * smoothedDerivative;
    previousError = error;

    // Calculate the PID output
    float output = Pout + Iout + Dout;
    return output;
}

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
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
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
  // myTime = millis(); 
  if( counter != temp ){
  Serial.println (counter);
//  Serial.print(", e0: ");
//  Serial.print(edges_0);
//  Serial.print(", ie0: ");
//  Serial.print(interesting_edges_0);
//  Serial.print(", cwe0: ");
//  Serial.print(clockwise_edges_0);
//  Serial.print(", acwe0: ");
//  Serial.print(anti_clockwise_edges_0);
//  Serial.print(", e1: ");
//  Serial.print(edges_1);
//  Serial.print(", ie1: ");
//  Serial.print(interesting_edges_1);
//  Serial.print(", cwe1: ");
//  Serial.print(clockwise_edges_1);
//  Serial.print(", acwe1: ");
//  Serial.println(anti_clockwise_edges_1);
  temp = counter;
  }

  // // pick input 

  // // value is in the range -255 to 255
  // // the lower half of it we use for reverse rotation; the upper half for forward rotation
  // if (1000<myTime && myTime<7000)
  // {
  //   setpoint = 20;
  // }
  // else if (7000<=myTime && myTime<14000)
  // {
  //   setpoint = -20;
  // }
  // else
  // {
  //   setpoint = 0;
  // }
  if (Serial.available() > 0)
  {
   // Take in Serial input
    String input = Serial.readStringUntil('\n'); // Read the incoming string until a newline character

    // Split the input into individual parameters
    int lastIndex = 0;
    while (lastIndex < input.length()) {
      // Find the next comma
      int nextComma = input.indexOf(',', lastIndex);
      if (nextComma == -1) nextComma = input.length(); // Handle the last parameter

      // Extract the key-value pair
      String pair = input.substring(lastIndex, nextComma);
      pair.trim(); // Remove any leading/trailing whitespace

      // Parse the key-value pair
      if (pair.startsWith("Reinitialize = ")) {
        reinitialize = pair.substring(15).equalsIgnoreCase("True");
      } else if (pair.startsWith("Mode = ")) {
        mode = pair.substring(7);
      } else if (pair.startsWith("Brake = ")) {
        brake = pair.substring(8).equalsIgnoreCase("True");
      } else if (pair.startsWith("Direction = ")) {
        direction = pair.substring(12);
      } else if (pair.startsWith("Speed = ")) {
        receive = pair.substring(8).toFloat();
      } else if (pair.startsWith("Steering Angle = ")) {
        steering_angle = pair.substring(17).toInt();
      } else if (pair.startsWith("DEBUG = ")){
        debug = pair.substring(8);        
      }

      // Move to the next key-value pair
      lastIndex = nextComma + 1;
    }
    if (debug == "ON"){
    Serial.print("Reinitialize: "); Serial.print(reinitialize);
    Serial.print(", Mode: "); Serial.print(mode);
    Serial.print(", Brake: "); Serial.print(brake);
    Serial.print(", Direction: "); Serial.print(direction);
    Serial.print(", Speed: "); Serial.print(receive);
    Serial.print(", Steering Angle: "); Serial.println(steering_angle);                                                         
    }
    setpoint = steering_angle;
  }
  // Compute the PID output (duty cycle)
  if (setpoint< -56)
  {
    setpoint = -56;
  }
  if (setpoint>56)
  {
    setpoint = 56;
  }
  if (setpoint>0)
  {
    dutyCycle = map(setpoint,0,56,170,255);
  }
  if (setpoint<0)
  {
    dutyCycle = map(setpoint,-56,0,-255,-170);
  }
  if (setpoint==0)
  {
    dutyCycle = 0;
  }
  // pidOutput = -computePID(setpoint, counter);

  // // Ensure the output (duty cycle) is between 0 and 255 for PWM
  // dutyCycle = constrain((int)pidOutput, -255, 255);

  
  if ((dutyCycle < -255) || (dutyCycle > 255))
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
  // if (myTime<15000)
  // {
  // Serial.print("Time: ");
  // Serial.print(myTime);
  // Serial.print(", ");
  // Serial.print("Angle: ");
  // Serial.print(counter);
  // Serial.print(", ");
  // Serial.print("DutyCycle: ");
  // Serial.print(dutyCycle);
  // Serial.print(", ");
  // Serial.print("pidOUTPUT: ");
  // Serial.println(pidOutput);
  // }
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
    if (digitalRead(2) == HIGH) {
      anti_clockwise_edges_0++;
      counter++;
    } else {
      clockwise_edges_0++;
      counter--;
    }
  }
}

void ai1_handler() {
  edges_1++;
  if (digitalRead(2) == LOW) {
    interesting_edges_1++;
    if (digitalRead(3) == HIGH) {
      clockwise_edges_1++;
      counter--;
    } else {
      anti_clockwise_edges_1++;
      counter++;
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
