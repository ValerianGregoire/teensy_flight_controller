#ifndef CONTROL
#define CONTROL

#include "common.h"

/*
Initialize peripherals to send controllers commands.
Returns true if successful.
*/
bool init_controllers();

/*
Controller task. Reads data from *_data_new structs to compute the drone's current state and publish it to drone_state. Converts all *_data_new to *_data_old and updates the motors_input struct.
*/
void controller(void *parameters);

/*
ESC task. Reads data from motors_input and sends it to the ESCs through PWM.
*/
void esc(void *parameters);

/*
LED task. Reads data from led_state and sends it to the LED.
*/
void led(void *parameters);

/*
Telemetry task. Receives data through UART and updates ground_station_data. Reads data from ground_station_data and sends it through UART. Updates led_state based on context.
*/
void telemetry(void *parameters);

#endif