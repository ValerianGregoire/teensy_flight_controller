#pragma once

#ifdef B1
#undef B1
#endif
#ifdef B0
#undef B0
#endif

#include <Eigen/Dense>
#include <Arduino.h>
using arduino::HIGH;
using arduino::LOW;
using arduino::LSBFIRST;
using arduino::MSBFIRST;
using arduino::OUTPUT;
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_BNO08x.h>
#include "Bitcraze_PMW3901.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>
#include <algorithm>

// Performance measurement macros
// Place at start of loop
#define PERF_START(metric)                                         \
    uint32_t _pStart = ARM_DWT_CYCCNT;                             \
    uint32_t _totalCycles = 0;                                     \
    if (metric.lastStart > 0)                                      \
    {                                                              \
        _totalCycles = _pStart - metric.lastStart;                 \
        /* Frequency = ClockSpeed / CyclesDiff */                  \
        if (_totalCycles > 0)                                      \
            metric.frequency = 600000000.0f / (float)_totalCycles; \
    }                                                              \
    metric.lastStart = _pStart;

#define PERF_END(metric)                                                      \
    uint32_t _pEnd = ARM_DWT_CYCCNT;                                          \
    uint32_t _execCycles = _pEnd - _pStart;                                   \
    metric.execTimeUs = _execCycles / 600;                                    \
    /* CPU Load = (ExecutionCycles / TotalPeriodCycles) * 100 */              \
    if (_totalCycles > 0)                                                     \
    {                                                                         \
        metric.cpuLoad = ((float)_execCycles / (float)_totalCycles) * 100.0f; \
    }

// Serial config
#define SERIAL_BAUD 115200
#define SERIAL2_BAUD 115200
#define SERIAL3_BAUD 115200

// SPI config
#define SPI1_MOSI 26
#define SPI1_MISO 39
#define SPI1_SCK 27

// Pin config
#define DPS310_CS 10
#define PMW3901_CS 20

#define BNO08X_CS 19
#define BNO08X_RESET 18
#define BNO08X_INT 17

#define ESC1_PIN 2
#define ESC2_PIN 3
#define ESC3_PIN 4
#define ESC4_PIN 5

/* PIN SETUP
DPS310 -> Teensy 4.1:
    - VIN -> 3V (needs to be alone on the port)
    - 3Vo -> N/C
    - GND -> GND (needs to be alone on the port)
    - SCK -> 27
    - SDO -> 39
    - SDI -> 26
    - CS  -> 40

BNO085 -> Teensy 4.1:
    - VIN -> 3V
    - 3Vo -> N/C
    - GND -> GND
    - SCL -> 27
    - SDA -> 39
    - INT -> 17
    - BT  -> N/C
    - P0  -> 3V
    - P1  -> 3V
    - RST -> 18
    - DI  -> 26
    - CS  -> 19

PMW3901 -> Teensy 4.1:
    - 3-5V -> 3V
    - CS   -> 20
    - SCK  -> 13
    - MOSI -> 11
    - MISO -> 12
    - INT  -> N/C
    - GND  -> GND

TF-Luna -> Teensy 4.1:
    - Red Wire -> 3V
    - Blue Wire (2) -> 8
    - Blue Wire (3) -> 7
    - Black Wire    -> GND

TEL0153 -> Teensy 4.1:
    - + -> 3V
    - - -> GND
    - R -> 14
    - T -> 15

ESC -> Teensy 4.1:
    - 1 -> 2
    - 1 -> 3
    - 1 -> 4
    - 1 -> 5
*/

// Performance metrics structs
struct TaskPerf
{
    float frequency;     // Measured Hz
    float cpuLoad;       // % of CPU time used
    uint32_t execTimeUs; // Time taken to execute logic
    uint32_t lastStart;  // Timestamp for freq calc
};

struct SensorHealth
{
    uint32_t updateCount; // Increments on every valid read
    float actualRate;     // Hz (calculated by monitor)
    uint32_t lastCount;   // For calculation
};

struct SystemMetrics
{
    TaskPerf barometer, ofs, lidar, imu, ekf, fiber, esc, logger;
    SensorHealth barometerSensor, ofsSensor, lidarSensor, imuSensor;
};

// Data structs
struct BarometerData
{
    float temperature;      // In *C
    float pressure;         // In hPa
    float absoluteAltitude; // In m
    float relativeAltitude; // In m
};

struct OFSData
{
    int16_t deltaX;
    int16_t deltaY;
    float distanceX; // In m
    float distanceY; // In m
};

struct LidarData
{
    int distance; // In mm
    int strength;
    float temperature; // In *C
};

struct IMUData
{
    float ax;
    float ay;
    float az;
    float p;
    float q;
    float r;
    float roll;
    float pitch;
    float yaw;
};

struct StateEstimate
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct FiberData
{ // User inputs
    float vx;
    float vy;
    float vz;
    float yaw_rate;
};

struct ESCData
{
    float m1; // 0.0 to 1.0
    float m2; // 0.0 to 1.0
    float m3; // 0.0 to 1.0
    float m4; // 0.0 to 1.0
};

// Let tasks access these structs before declaration in main
extern BarometerData barometerData;
extern OFSData ofsData;
extern LidarData lidarData;
extern IMUData imuData;
extern StateEstimate stateEstimate;
extern FiberData fiberData;
extern ESCData escData;
extern SystemMetrics sysMetrics; // Global metrics

// Mutexes to manage data sharing
extern SemaphoreHandle_t barometerMutex;
extern SemaphoreHandle_t ofsMutex;
extern SemaphoreHandle_t lidarMutex;
extern SemaphoreHandle_t imuMutex;
extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t spi1Mutex;
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t fiberMutex;
extern SemaphoreHandle_t escMutex;
extern SemaphoreHandle_t serialMutex;

// Runtime variables
extern SPIClass &spi;
extern SPIClass &spi1;
extern Adafruit_DPS310 dps;
extern Bitcraze_PMW3901 flow;
extern Adafruit_BNO08x bno08x;
extern Servo m1, m2, m3, m4;