#include "brakes.h"

//Global brake instance
struct Pins pins = {
  .dir = 9,
  .pul = 8,
  .ena = 10,
  //.relay = 10,
};

Brake brakes;


void setup() 
{
  Serial.begin(9600);
  brake_setup(&pins);  
}

void loop() 
{
  int input = Serial.read();  //To store input recieved from bluetooth

  //Determine direction of rotation
  if (input == '0') //Releasing Brake
  {
    release_brake(&brakes, &pins);
  }
  else if (input == '1') //Pressing Brake
  {
    press_brake(&brakes, &pins);
  }

  // if (Serial.available() > 0) 
  // {
  //   int input = Serial.read();  //To store input recieved from bluetooth
    
  //   // Filter input
  //   char val = (char)input;
  //   if (val == '\r' || val == '\n') 
  //   {
  //     return;
  //   }

  //   if (input == 'a') //Turn relay off
  //   {
  //     Serial.println("OFF");
  //     relay_control_off(&brake, &pins);
  //   }
  //   else if (input == 'b')  //Turn relay on
  //   {
  //     Serial.println("ON");
  //     relay_control_on(&brake, &pins);
  //   }

  //   if (check_input(val, &brake)) //Check if the input is valid
  //   {
  //     //When system is in TeleOp mode Electronically
  //     if (brake.relayFlag) 
  //     {
  //       //Determine direction of rotation
  //       if (input == '0') //Releasing Brake
  //       {
  //         release_brake(&brake, &pins);
  //       }
  //       else if (input == '1') //Pressing Brake
  //       {
  //         press_brake(&brake, &pins);
  //       }
  //     }
  //   }
  // }
}