#pragma once

#ifdef B1
#undef B1
#endif
#ifdef B0
#undef B0
#endif

#include <Eigen/Dense>

#include <Arduino.h>
using arduino::LSBFIRST;
using arduino::MSBFIRST;
using arduino::OUTPUT;
using arduino::HIGH;
using arduino::LOW;
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_BNO08x.h>
#include "Bitcraze_PMW3901.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

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

/* PIN SETUP
DPS310 -> Teensy 4.1:
    - VIN -> 3V (needs to be alone on the port)
    - 3Vo -> N/C
    - GND -> GND (needs to be alone on the port)
    - SCK -> 27
    - SDO -> 39
    - SDI -> 26
    - CS  -> 10

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
*/

// Data structs
struct BarometerData
{
    float temperature; // In *C
    float pressure; // In hPa
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

// State estimate struct
struct StateEstimate {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

extern StateEstimate stateEstimate;


// Let tasks access these structs before declaration in main
extern BarometerData barometerData;
extern OFSData ofsData;
extern LidarData lidarData;
extern IMUData imuData;

// Mutexes to manage data sharing
extern SemaphoreHandle_t barometerMutex;
extern SemaphoreHandle_t ofsMutex;
extern SemaphoreHandle_t lidarMutex;
extern SemaphoreHandle_t imuMutex;
extern SemaphoreHandle_t spiMutex; // for VSPI peripheral sharing
extern SemaphoreHandle_t spi1Mutex; // for HSPI peripheral sharing
extern SemaphoreHandle_t stateMutex;

// Runtime variables
extern SPIClass &spi;
extern SPIClass &spi1;
extern Adafruit_DPS310 dps;
extern Bitcraze_PMW3901 flow;
extern Adafruit_BNO08x bno08x;