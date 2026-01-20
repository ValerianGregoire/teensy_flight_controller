#include <Arduino.h>
#include <SPI.h>
#include "Bitcraze_PMW3901.h"

// Define SPI Pins for PMW3901
#define OFS_SPI_CS 25

// 2. Pass the CS pin to the library
Bitcraze_PMW3901 flow(OFS_SPI_CS);

// Initialize movement parameters
uint8_t resolution = 30;
uint8_t pixelsCount = 35;
double fov = 42.0;

double height; // in mm
double scaleFactor;
float distanceX, distanceY;

void setup()
{
    Serial.begin(115200);

    // Init SPI peripheral
    SPI.begin();

    // Initialize the sensor
    if (!flow.begin())
    {
        Serial.println("Initialization of the flow sensor failed!");
        while (1); // Halt if sensor not found
    }

    // Initalize processed variables
    height = 300;

    Serial.println("PMW3901 initialized successfully.");
}

void loop()
{
    // Read delta X and delta Y
    int16_t deltaX, deltaY;
    flow.readMotionCount(&deltaX, &deltaY);
    
    // Compute movement
    scaleFactor = (2 * height * tan(fov/2.0))/(pixelsCount * resolution);
    distanceX = deltaX * scaleFactor;
    distanceY = deltaY * scaleFactor;
    
    // Only print if there is significant movement
    if (deltaX != 0 || deltaY != 0)
    {
        Serial.print(">ofs_delta_x:");
        Serial.println(deltaX);
        Serial.print(">ofs_delta_y:");
        Serial.println(deltaY);
        Serial.print(">ofs_distance_x:");
        Serial.println(distanceX);
        Serial.print(">ofs_distance_y:");
        Serial.println(distanceY);
    }
}