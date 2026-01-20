#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include "Bitcraze_PMW3901.h"
#include <SPI.h>

// Serial config
#define SERIAL_BAUD 115200
#define SERIAL2_BAUD 115200
#define SERIAL2_RX 17
#define SERIAL2_TX 4

// Pin config
#define DPS310_CS 22
#define PMW3901_CS 25
#define BNO310_CS


#define HSPI_MOSI 13
#define HSPI_MISO 14
#define HSPI_SCK 26

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
    float r;
    float p;
    float s;
};

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
extern SemaphoreHandle_t vspiMutex; // for VSPI peripheral sharing

// Runtime variables
extern SPIClass vspi;
extern Adafruit_DPS310 dps;
extern Bitcraze_PMW3901 flow;