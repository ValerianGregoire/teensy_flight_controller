#pragma once

// Gets relative/absolute altitude through VSPI
void barometer(void *params);

// Gets movement along x/y through SPI
void ofs(void *params);

// Reads distance measurements through UART2
void lidar(void *params);

// Gets accelerations/tilt/yaw through VSPI
void imu(void *params);

// Sends dshot commands to ESCs
void esc(void *params);

// Prints updated data
void logger(void *params);
