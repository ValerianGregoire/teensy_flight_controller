#ifndef COMMON
#define COMMON

#include "Arduino.h"
#include "SPI.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdexcept>

// DEBUG PARAMETERS
#define __DEBUG // Comment out to enter debug mode
#define __DEBUG_BAUD 115200

// SPI PARAMETERS
#define SPI_BUFFER_SIZE 128

// UART PARAMETERS
#define TELEM_UART_RX 
#define TELEM_UART_TX 
#define TELEM_UART_BAUD 

// PWM PARAMETERS
#define ESC1_PWM_PIN 
#define ESC2_PWM_PIN 
#define ESC3_PWM_PIN 
#define ESC4_PWM_PIN 

// === SENSORS STRUCTS DEFINITION ===
struct optical_flow_data
{
    char *data;               // Raw data
    uint8_t right_movement = 0;   // Right movement speed
    uint8_t forward_movement = 0; // Forward movement speed
};

struct imu_data
{
    char *data; // Raw data
    /*
    To fill
    */
};

struct barometer_data
{
    char *data; // Raw data
    /*
    To fill
    */
};

struct lidar_data
{
    char *data; // Raw data
    /*
    To fill
    */
};

extern uint8_t all_updated_reg;

// === MOVEMENT STRUCTS DEFINITION ===
struct drone_position
{
    // Current position
    double y = 0.0;     // Y position from take off in meters
    double z = 0.0;     // Z position from take off in meters
    double x = 0.0;     // X position from take off in meters

    // Destination
    double tgt_x = 0.0;   // Target X position from take off
    double tgt_y = 0.0;   // Target Y position from take off
    double tgt_z = 0.0;   // Target Z position from take off
    double tgt_yaw = 0.0; // Target yaw
};

struct wire_position
{
    double *x_points; // X position of each waypoint in meters
    double *y_points; // X position of each waypoint in meters
    double *z_points; // X position of each waypoint in meters
};

struct map
{
    double *x_points; // X position of each wall in meters
    double *y_points; // X position of each wall in meters
    double *z_points; // X position of each wall in meters
};


// === CONTROL STRUCTS DEFINITION ===
struct ground_station_data
{
    char *data; // Raw data
    /*
    To fill
    */
};

enum led_state
{
    OFF,       // Off state
    ON,        // On state
    BLINK_1,   // 1 blink and 1s break (2 Hz - 1s pause)
    BLINK_2,   // 2 blinks and 1s break (2 Hz - 1s pause)
    BLINK_3,   // 3 blinks and 1s break (2 Hz - 1s pause)             
    BREATHING, // Continuous breathing  (0.5 Hz)
    ALARM      // Continuous blinking   (2 Hz)
};

struct motors_input
{
    uint16_t esc1_pwm = 0;  // PWM value for motor 1 (0 - 1023)
    uint16_t esc2_pwm = 0;  // PWM value for motor 2 (0 - 1023)
    uint16_t esc3_pwm = 0;  // PWM value for motor 3 (0 - 1023)
    uint16_t esc4_pwm = 0;  // PWM value for motor 4 (0 - 1023)
};

struct drone_state
{
    double pitch = 0.0;   // Pitch in rad
    double roll = 0.0;    // Roll in rad
    double yaw = 0.0;     // Yaw in rad

    double ax = 0.0;      // Acceleration along x in m.s^-2
    double ay = 0.0;      // Acceleration along y in m.s^-2
    double az = 0.0;      // Acceleration along z in m.s^-2

    double vx = 0.0;      // Velocity along x in m.s^-1
    double vy = 0.0;      // Velocity along y in m.s^-1
    double vz = 0.0;      // Velocity along z in m.s^-1
};

// === MUTEXES DECLARATION ===
extern SemaphoreHandle_t sensors_data_old_mutex;
extern SemaphoreHandle_t optical_flow_data_new_mutex;
extern SemaphoreHandle_t imu_data_new_mutex;
extern SemaphoreHandle_t barometer_data_new_mutex;
extern SemaphoreHandle_t lidar_data_new_mutex;
extern SemaphoreHandle_t drone_position_mutex;
extern SemaphoreHandle_t wire_position_mutex;
extern SemaphoreHandle_t map_mutex;
extern SemaphoreHandle_t motors_input_mutex;
extern SemaphoreHandle_t ground_station_data_mutex;
extern SemaphoreHandle_t led_state_mutex;

// === HANDLES DECLARATION ===
// Controller handles
extern TaskHandle_t controller_handle;
extern TaskHandle_t esc_handle;
extern TaskHandle_t led_handle;
extern TaskHandle_t telemetry_handle;

// Sensors handles
extern TaskHandle_t optical_flow_handle;
extern TaskHandle_t imu_handle;
extern TaskHandle_t barometer_handle;
extern TaskHandle_t lidar_handle;

// Movement handles
extern TaskHandle_t navigation_handle;
extern TaskHandle_t mapping_handle;
extern TaskHandle_t wire_handle;

#endif