// Control of EPS motor 
#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H
#include "steering_angle_sensing.h"
#include <avr/interrupt.h>

bool manual_mode = false;

// Input is Duty cycle which varies from -255 to 255, beyond those values it stops. 0 is unavailable for use
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
unsigned long myTime;
float setpoint = 0;
int dutyCycle = 0;
// Define PID control constants


const float Kp = -8.68368161935867; // Proportional gain
// const float Ki = -3.8004976592663; // Integral gain
const float Ki = 0; // Integral gain
const float Kd = -0.178015473196853; // Derivative gain
// const float N = 14.908;    // Filter coefficient

// Time variables
unsigned long lastTime = 0;
float sampleTime = 0.0041; // 10 ms (adjust as per your system)

// PID variables
float prevError = 0;
float integral = 0;
float prevDerivative = 0;
float output = 0;
float pidOutput = 0;
// 
// Integral clamping limits
const float integratorMin = -abs(7/Ki); // Minimum integrator limit (adjust as needed)
const float integratorMax = abs(7/Ki);  // Maximum integrator limit (adjust as needed)

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

  
void steering_control_setup()
{
  Serial.println("restarting");
  steering_angle_sensing_setup();
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  lastTime = millis(); 
}

int clampSetpoint(float setpoint)
{
  // Compute the PID output (duty cycle)
  if (setpoint< -56)
  {
    setpoint = -56;
  }
  if (setpoint>56)
  {
    setpoint = 56;
  }
  return setpoint; 
}

int clampDutyCycle(int dutyCycle)
{
 dutyCycle = constrain(dutyCycle, -255, 255);
 if ((dutyCycle < -17) && (dutyCycle >-177))
  {
    dutyCycle = -177;
  }
  else if ((dutyCycle > 17) && (dutyCycle < 177))
  {
    dutyCycle = 177;
  }
  return dutyCycle;
}

void pwm_output(int dutyCycle)
{
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
  
  return;

}

int ControlLoop(float setpoint)
{
  if(manual_mode) {
    dutyCycle = 0;
  }
  else {
    //Clamp Setpoint
    setpoint = clampSetpoint(setpoint);
  
    // Compute the PID output (duty cycle)
    pidOutput = computePID(setpoint, steering_angle);
  
  
    // Clamp Duty Cycle
    dutyCycle = clampDutyCycle((int)pidOutput);
  }
  // Serial.println(dutyCycle);
 
  // Serial.print("dutyCycle = ");Serial.println(dutyCycle);
  // Serial.print(",timer = ");Serial.println(myTime - prevTime);
  // prevTime= myTime;
  // value is in the range -255 to 255
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  // dutyCycle = inputV;
  // dutyCycle = inputV;
  
  // Writing PWM
  pwm_output(dutyCycle);
  return dutyCycle;

 
}


#endif // STEERING_CONTROl_H
