#ifndef BRAKES_H
#define BRAKES_H

#include <Arduino.h>

//Struct to encapsulate brake data and functionality
struct Pins
{
  //Pin Definiations
  const int dir;  //Direction pin (yellow)
  const int pul;  //Pulse pin  (grey)
  const int ena;  //Enable pin (blue)
  //const int relay;  //Relay pin (to turn on the the relay) (green)
};

struct Brake
{
  //Variables
  long currentPosition = 0; //Current position in steps
  long targetPosition = 0;  //Target position in steps
  unsigned long stepInterval = 12; //Step interval in microseconds
  unsigned long lastStepTime = 0; //Last time a step was taken
  //bool relayFlag = false; //Goes true when relay is on
  //int inputs[2] = {0, 1};  //Allowed inputs for validation
};



//Function prototypes
void brake_setup(struct Pins* pins);
//bool check_input(char element, struct Brake* brake);  
//void relay_control_on(struct Brake* brake, struct Pins* pins);
//void relay_control_off(struct Brake* brake, struct Pins* pins);
void press_brake(struct Brake* brake, struct Pins* pins);
void release_brake(struct Brake* brake, struct Pins* pins);
void move_to_position(struct Brake* brake, struct Pins* pins, long target);
void run(struct Brake* brake, struct Pins* pins);


#endif  // BRAKE_H