#ifndef JOYSTICK_H
#define JOYSTICK_H
#include "SerialTransfer.h"

SerialTransfer myTransfer;
bool reinitialize = false;
bool manual = false;
bool brake = false;
bool reverse = false;
bool debug = false;
float ref_speed;
long ref_steering_angle = 0;
char arr[13];

bool serial_transfer_debug = false;

void joystick_setup() {
  Serial.begin(115200);
  myTransfer.begin(Serial, false); // false for silencing debug messages
}

void PrintHex(char myString[], int length) {
  // Print the character array in hexadecimal
  for (int i = 0; i < length; i++) { // Exclude null terminator
    Serial.print("0x");              // Prefix for hex
    if ((uint8_t)myString[i] < 16) {
      Serial.print("0"); // Add leading zero for single-digit hex values
    }
    Serial.print((uint8_t)myString[i],
                 HEX); // Print the ASCII value in hexadecimal
    Serial.print(" "); // Add a space between hex values
  }
}

bool JoystickLoop() {
  // Serial.println(steering_angle);
  if (myTransfer.available()) {
    // uint16_t recSize = myTransfer.rxObj(arr, 0, 13);
    // PrintHex(arr, 13);
    // Serial.println("");
    uint16_t recSize = 0;
    char ch;
    char str[1];
    char *str1;
    recSize = myTransfer.rxObj(ch, recSize);
    reinitialize = (ch == 1);
    if (serial_transfer_debug) {
      Serial.print("reinit: ");
      Serial.print(recSize);
      Serial.print(", ");
      str[0] = ch;
      PrintHex(str, 1);
      Serial.print(", ");
      Serial.println(reinitialize);
    }

    recSize = myTransfer.rxObj(ch, recSize);
    manual = (ch == 1);
    if (serial_transfer_debug) {
      Serial.print("manual: ");
      Serial.print(recSize);
      Serial.print(", ");
      str[0] = ch;
      PrintHex(str, 1);
      Serial.print(", ");
      Serial.println(manual);
    }

    recSize = myTransfer.rxObj(ch, recSize);
    brake = (ch == 1);
    if (serial_transfer_debug) {
      Serial.print("brake: ");
      Serial.print(recSize);
      Serial.print(", ");
      str[0] = ch;
      PrintHex(str, 1);
      Serial.print(", ");
      Serial.println(brake);
    }

    recSize = myTransfer.rxObj(ch, recSize);
    reverse = (ch == 1);
    if (serial_transfer_debug) {
      Serial.print("reverse: ");
      Serial.print(recSize);
      Serial.print(", ");
      str[0] = ch;
      PrintHex(str, 1);
      Serial.print(", ");
      Serial.println(reverse);
    }

    recSize = myTransfer.rxObj(ref_speed, recSize);
    if (serial_transfer_debug) {
      Serial.print("ref speed: ");
      Serial.print(recSize);
      Serial.print(", ");
      str1 = (char *)&ref_speed;
      PrintHex(str1, 4);
      Serial.print(", ");
      Serial.println(ref_speed);
    }

    recSize = myTransfer.rxObj(ref_steering_angle, recSize);
    if (serial_transfer_debug) {
      Serial.print("ref steering angle: ");
      Serial.print(recSize);
      Serial.print(", ");
      str1 = (char *)&ref_steering_angle;
      PrintHex(str1, 4);
      Serial.print(", ");
      Serial.println(ref_steering_angle);
    }

    recSize = myTransfer.rxObj(ch, recSize);
    debug = (ch == 1);
    if (serial_transfer_debug) {
      Serial.print("debug: ");
      Serial.print(recSize);
      Serial.print(", ");
      str[0] = ch;
      PrintHex(str, 1);
      Serial.print(", ");
      Serial.println(debug);
    }
    return true;
  }
  return false;
}

#endif // JOYSTICK_H

