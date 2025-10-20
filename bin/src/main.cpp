#include "common.h"
#include "control.h"
#include "movement.h"
#include "sensors.h"

// Controller handles
TaskHandle_t controller_handle = NULL;
TaskHandle_t esc_handle = NULL;
TaskHandle_t led_handle = NULL;
TaskHandle_t telemetry_handle = NULL;

// Sensors handles
TaskHandle_t optical_flow_handle = NULL;
TaskHandle_t imu_handle = NULL;
TaskHandle_t barometer_handle = NULL;
TaskHandle_t lidar_handle = NULL;

// Movement handles
TaskHandle_t navigation_handle = NULL;
TaskHandle_t mapping_handle = NULL;
TaskHandle_t wire_handle = NULL;

// Control register
uint8_t all_updated_reg = 0x00;

void setup()
{
#ifdef __DEBUG
    // Init debug
    Serial.begin(__DEBUG_BAUD);
#endif

    // Init controllers
    if (!init_controllers())
    {
        // throw std::runtime_error("Failed to init controllers");
        while (1)
            ;
    }
#ifdef __DEBUG
    else
    {
        Serial.println("Successfully initialized controllers");
    }
#endif

    // Init sensors
    if (!init_sensors())
    {
        // throw std::runtime_error("Failed to init sensors");
        while (1)
            ;
    }
#ifdef __DEBUG
    else
    {
        Serial.println("Successfully initialized sensors");
    }
#endif

    // Init mutexes
    SemaphoreHandle_t sensors_data_old_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t optical_flow_data_new_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t imu_data_new_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t barometer_data_new_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t lidar_data_new_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t drone_position_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t wire_position_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t map_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t motors_input_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t ground_station_data_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t led_state_mutex = xSemaphoreCreateMutex();

    // Init tasks
    xTaskCreate( // Optical flow task
        optical_flow,
        "optical_flow",
        2048,
        NULL,
        4,
        &optical_flow_handle);

    xTaskCreate( // IMU task
        imu,
        "imu",
        2048,
        NULL,
        4,
        &imu_handle);

    xTaskCreate( // Barometer task
        barometer,
        "barometer",
        2048,
        NULL,
        4,
        &barometer_handle);

    xTaskCreate( // Lidar task
        lidar,
        "lidar",
        2048,
        NULL,
        4,
        &lidar_handle);

    xTaskCreate( // Controller task
        controller,
        "controller",
        2048,
        NULL,
        5,
        &controller_handle);

    xTaskCreate( // ESC task
        esc,
        "esc",
        2048,
        NULL,
        5,
        &esc_handle);

    xTaskCreate( // Telemetry task
        telemetry,
        "telemetry",
        2048,
        NULL,
        2,
        &telemetry_handle);

    xTaskCreate( // LED task
        led,
        "led",
        2048,
        NULL,
        1,
        &led_handle);

    xTaskCreate( // Mapping task
        mapping,
        "mapping",
        2048,
        NULL,
        3,
        &mapping_handle);

    xTaskCreate( // Wire task
        wire,
        "wire",
        2048,
        NULL,
        3,
        &wire_handle);

    xTaskCreate( // Navigation task
        navigation,
        "navigation",
        2048,
        NULL,
        3,
        &navigation_handle);
}

void loop()
{
    // Don't write anything here
}