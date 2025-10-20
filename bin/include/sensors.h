#ifndef SENSORS
#define SENSORS

#include "common.h"

/*
Initialize peripherals to get sensors data.
Returns true if successful.
*/
bool init_sensors();

/*
Optical flow sensor task. Reads data through SPI and sends it to optical_flow_data_new.
Sets a flag in all_updated_reg when called.
*/
void optical_flow(void *parameters);

/*
IMU task. Reads data through SPI and sends it to imu_data_new.
Sets a flag in all_updated_reg when called.
*/
void imu(void *parameters);

/*
Barometer task. Reads data through SPI and sends it to barometer_data_new.
Sets a flag in all_updated_reg when called.
*/
void barometer(void *parameters);

/*
Lidar task. Reads data through UART and sends it to lidar_data_new.
Sets a flag in all_updated_reg when called.
*/
void lidar(void *parameters);

#endif