#include "include/brakes.h"

//Function to initialize the brake system
void brake_setup(struct Pins* pins) {
  pinMode(pins->dir, OUTPUT);
  pinMode(pins->pul, OUTPUT);
  pinMode(pins->ena, OUTPUT);
  //pinMode(pins->relay, OUTPUT);

  //digitalWrite(pins->relay, LOW);  //LOW means relay is off, HIGH means relay is on
  //digitalWrite(pins->ena, HIGH); //High enable pin means motor is disengaged

}

//Function to check if input is valid
// bool check_input(char element, struct Brake* brake) {
//   //Function iterates through the list of valid inputs to check if recieved input is valid
//   for (int i = 0; i < 2; i++) 
//   {
//     if ((char)(brake->inputs[i] + '0') == element) 
//     {
//       return true;  //Returns true if input is valid
//     }
//   }
//   return false; //Returns false if input is invalid
// }

//Function to turn relay on
// void relay_control_on(struct Brake* brake, struct Pins* pins) {
//   brake->relayFlag = true;
//   digitalWrite(pins->relay, HIGH);
// }

//Function to turn relay off
// void relay_control_off(struct Brake* brake, struct Pins* pins) {
//   brake->relayFlag = false;
//   digitalWrite(pins->relay, LOW);
// }

//Function to press brake
void press_brake(struct Brake* brake, struct Pins* pins) {
  digitalWrite(pins->dir, HIGH);  
  brake->targetPosition = 28800;
  digitalWrite(pins->ena, LOW);
  move_to_position(brake, pins, brake->targetPosition);
  brake->currentPosition = 0;
  digitalWrite(pins->ena, HIGH);
}

//Function to release brake
void release_brake(struct Brake* brake, struct Pins* pins) {
  digitalWrite(pins->dir, LOW);  
  brake->targetPosition = 20000;
  digitalWrite(pins->ena, LOW);
  move_to_position(brake, pins, brake->targetPosition);
  brake->currentPosition = 0;
  digitalWrite(pins->ena, HIGH);
}

//Function to move the motor to specified position
void move_to_position(struct Brake* brake, struct Pins* pins, long target) {
  brake->targetPosition = target;
  while (brake->currentPosition != brake->targetPosition) 
  {
    run(brake, pins);
  } 
}

//Function to rotate the motor
void run(struct Brake* brake, struct Pins* pins) {
  unsigned long currentTime = micros();
  if (currentTime - brake->lastStepTime >= brake->stepInterval) 
  {
    brake->lastStepTime = currentTime;
    if (brake->currentPosition < brake->targetPosition) 
    {
      brake->currentPosition++;
    } else if (brake->currentPosition > brake->targetPosition) 
    {
      brake->currentPosition--;
    }
    digitalWrite(pins->pul, HIGH);
    delayMicroseconds(9);
    digitalWrite(pins->pul, LOW);
    delayMicroseconds(3);
  }
}