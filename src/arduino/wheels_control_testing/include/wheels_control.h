
#ifndef WHEELS_CONTROL_H
#define WHEELS_CONTROL_H

#include "wheels_sensing.h"
#include <Adafruit_MCP4725.h>
#include <Wire.h>

int control_tick_period = 1; // by default the tick period is assumed to be 1ms
int control_timer_ticks = 0;
int control_timer_period = 60;
bool manual_mode = false;

Adafruit_MCP4725 dac; // Create an MCP4725 object
float Kp = 0.0050;    // Proportional gain
float Ki = 0.000628;  // Integral gain
float Kd = 0.00003;   // Derivative gain
float alpha = 0.1;    // Smooth
float errorCap = 60;
int idle_pin = 11;
int teleop_pin = 10;
float wheel_speed = 0.0; 
// these are checked for in the main program
float Pout;
float Iout;
float Dout;
float output;
float speed = 0;
float pidOutput = 0;
float vvv = 0;
int dacValue = 0;

// Variables for PID calculation
float previousError = 0.0;
float integral = 0.0;
float smoothedDerivative = 0.0;
unsigned long lastTime = 0;

float PID(float setpoint, float measuredValue) {
  // Calculate time elapsed
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  // Calculate the error term
  float error = setpoint - measuredValue;

  // Proportional term
  Pout = Kp * error;

  // Integral term with clamping
  integral += error * elapsedTime;
  if (integral > 1.47 / Ki)
    integral = 1.47 / Ki; // Upper clamp
  else if (integral < -1.47 / Ki)
    integral = -1.47 / Ki; // Lower clamp
  Iout = Ki * integral;

  // Derivative term with smoothing
  float rawDerivative = (error - previousError) / elapsedTime;
  smoothedDerivative = alpha * rawDerivative + (1 - alpha) * smoothedDerivative;
  Dout = Kd * smoothedDerivative;
  previousError = error;
  // Calculate the PID output
  output = Pout + Iout + Dout;
  return output;
}

void WheelsControlSetup(int tick_period) {
  control_tick_period = tick_period;
  dac.begin(0x60);
  // if (dac.begin(0x60))   // Initialize MCP4725 DAC at default address (0x60)
  // {
  //   Serial.println("MCP4725 Initialized Successfully.");
  // }
  // else
  // {
  //   Serial.println("Failed.");
  // }

  pinMode(idle_pin, OUTPUT);
  pinMode(teleop_pin, OUTPUT);
  dac.setVoltage(0, false); // Set DAC output to 0V
  // Serial.println("Frequency Counter"); // to show that we have initialized
  // successfully
}

void ControlLoop() {
  control_timer_ticks += control_tick_period;
  if (control_timer_ticks < control_timer_period)
    return;

  if (manual_mode) {
    digitalWrite(teleop_pin, LOW);
    digitalWrite(idle_pin, LOW);
    dacValue = 0;
    //dac.setVoltage(0, false);
  } else {
    digitalWrite(teleop_pin, HIGH);
    if (wheel_speed == 0) {
      digitalWrite(idle_pin, LOW);
      dacValue = 0;
      //dac.setVoltage(0, false);

    } else {
      digitalWrite(idle_pin, HIGH);
      // dac.setVoltage(wheel_speed*4096/5, false);
      pidOutput = PID(wheel_speed, frq) + 2.6;
      if (pidOutput < 1.1) {
        vvv = 2.5;
      } else if (pidOutput < 2.6) {
        vvv = 2.6;
      } else if (pidOutput > 4.4) {
        vvv = 4.4;
      } else {
        vvv = pidOutput;
      }

      dacValue =
          int(4095 / 5.0 * vvv); // Convert 0-5V range to 0-4095 range for DAC
      // Serial.println(dacValue);                   // Print the DAC value for
      // debugging
    }
  }
  TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
  dac.setVoltage(dacValue, false); // Send the corresponding voltage to DAC
  control_timer_ticks = 0;
}

#endif // WHEELS_CONTROL_H
