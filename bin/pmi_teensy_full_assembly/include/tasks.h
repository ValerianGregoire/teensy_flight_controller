#pragma once

// Gets relative/absolute altitude through SPI1
void barometer(void *params);

// Gets movement along x/y through SPI
void ofs(void *params);

// Reads distance measurements through UART2
void lidar(void *params);

// Gets accelerations/attitude/rates through SPI1
void imu(void *params);

// Extended Kalman Filter to fuse sensor data
void ekf(void *params);

// Manages optic fiber communication through UART3
void fiber(void *params);

// Sends PWM commands to ESCs
void esc(void *params);

// Prints tasks data through UART
void logger(void *params);

// Measures the performance of each task
void perfMonitor(void *params);
