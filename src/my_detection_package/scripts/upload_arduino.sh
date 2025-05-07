#!/bin/bash
BOARD="arduino:avr:uno"
PORT="/dev/ttyACM0"
SKETCH="firmware/lighting_control.ino"

arduino-cli compile --fqbn $BOARD $SKETCH
arduino-cli upload -p $PORT --fqbn $BOARD $SKETCH
