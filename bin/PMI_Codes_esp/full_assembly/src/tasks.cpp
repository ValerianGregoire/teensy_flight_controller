#include "tasks.h"
#include "common.h"
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <SPI.h>

void barometer(void *params)
{
    // Init
    float temperature;             // In *C
    float pressure;                // In hPa
    float pastPressure = 1013.25f; // In hPa
    float absoluteAltitude;        // In m
    float relativeAltitude;        // In m

    // Loop
    while (true)
    {
        if (xSemaphoreTake(vspiMutex, portMAX_DELAY))
        {
            sensors_event_t temp_event, pressure_event;
            if (dps.temperatureAvailable() && dps.pressureAvailable())
            {
                dps.getEvents(&temp_event, &pressure_event);
                temperature = temp_event.temperature;
                pressure = pressure_event.pressure;

                float temp = dps.readAltitude(pastPressure);
                if (abs(temp) < 5)
                {
                    relativeAltitude = temp;
                }

                absoluteAltitude = dps.readAltitude(1013.25f);
                pastPressure = pressure_event.pressure;
            }
            xSemaphoreGive(vspiMutex);
        }

        if (xSemaphoreTake(barometerMutex, portMAX_DELAY))
        {
            barometerData.temperature = temperature;
            barometerData.pressure = pressure;
            barometerData.absoluteAltitude = absoluteAltitude;
            barometerData.relativeAltitude = relativeAltitude;
            xSemaphoreGive(barometerMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(nullptr);
}

void ofs(void *params)
{
    // Init
    uint8_t resolution = 30;
    uint8_t pixelsCount = 35;
    double fov = 42.0;

    int16_t deltaX;
    int16_t deltaY;
    float distanceX; // In m
    float distanceY; // In m
    double scaleFactor;
    int height;

    if (xSemaphoreTake(lidarMutex, portMAX_DELAY))
    {
        lidarData.distance = 0;
        xSemaphoreGive(lidarMutex);
    }

    // Loop
    while (true)
    {
        // Read delta X and delta Y
        flow.readMotionCount(&deltaX, &deltaY);

        // Read lidar distance
        if (xSemaphoreTake(lidarMutex, portMAX_DELAY))
        {
            height = lidarData.distance;
            xSemaphoreGive(lidarMutex);
        }

        // Compute movement
        scaleFactor = (2 * height * tan(fov / 2.0)) / (pixelsCount * resolution);
        distanceX = deltaX * scaleFactor;
        distanceY = deltaY * scaleFactor;

        // Update data
        if (xSemaphoreTake(ofsMutex, portMAX_DELAY))
        {
            ofsData.deltaX = deltaX;
            ofsData.deltaY = deltaY;
            ofsData.distanceX = distanceX;
            ofsData.distanceY = distanceY;
            xSemaphoreGive(ofsMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(nullptr);
}

void lidar(void *params)
{
    // Init
    int distance; // In mm
    int strength;
    float temperature; // In *C

    // Loop
    while (true)
    {
        if (Serial2.available() >= 9)
        {
            if (Serial2.read() == 0x59)
            {
                if (Serial2.read() == 0x59)
                {
                    uint8_t buffer[7];
                    Serial2.readBytes(buffer, 7);

                    // Verify Checksum
                    uint8_t checksum = 0x59 + 0x59;
                    for (int i = 0; i < 6; i++)
                    {
                        checksum += buffer[i];
                    }

                    if (checksum == buffer[6])
                    {
                        distance = (buffer[0] + (buffer[1] << 8)) * 10;
                        strength = buffer[2] + (buffer[3] << 8);
                        temperature = (buffer[4] + (buffer[5] << 8)) / 8.0 - 256.0;

                        if (xSemaphoreTake(lidarMutex, portMAX_DELAY))
                        {
                            lidarData.distance = distance;
                            lidarData.strength = strength;
                            lidarData.temperature = temperature;
                            xSemaphoreGive(lidarMutex);
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void logger(void *params)
{
    // Barometer variables
    float barometer_temperature;      // In *C
    float barometer_pressure;         // In hPa
    float barometer_absoluteAltitude; // In m
    float barometer_relativeAltitude; // In m

    // OFS variables
    int ofs_deltaX;
    int ofs_deltaY;
    float ofs_distanceX; // In m
    float ofs_distanceY; // In m

    // Lidar variables
    int lidar_distance; // In mm
    int lidar_strength;
    float lidar_temperature; // In *C

    // IMU variables
    float ax;
    float ay;
    float az;
    float r;
    float p;
    float s;

    while (true)
    {
        // Get barometer data
        if (xSemaphoreTake(barometerMutex, portMAX_DELAY))
        {
            barometer_temperature = barometerData.temperature;
            barometer_pressure = barometerData.pressure;
            barometer_absoluteAltitude = barometerData.absoluteAltitude;
            barometer_relativeAltitude = barometerData.relativeAltitude;

            xSemaphoreGive(barometerMutex);
        }

        // Get ofs data
        if (xSemaphoreTake(ofsMutex, portMAX_DELAY))
        {
            ofs_deltaX = ofsData.deltaX;
            ofs_deltaY = ofsData.deltaY;
            ofs_distanceX = ofsData.distanceX;
            ofs_distanceY = ofsData.distanceY;

            xSemaphoreGive(ofsMutex);
        }
        
        // Get lidar data
        if (xSemaphoreTake(lidarMutex, portMAX_DELAY))
        {
            lidar_distance = lidarData.distance;
            lidar_strength = lidarData.strength;
            lidar_temperature = lidarData.temperature;

            xSemaphoreGive(lidarMutex);
        }

        // Print all data
        Serial.print(">barometer_temperature:");
        Serial.println(barometer_temperature);
        Serial.print(">barometer_pressure:");
        Serial.println(barometer_pressure);
        Serial.print(">barometer_absoluteAltitude:");
        Serial.println(barometer_absoluteAltitude);
        Serial.print(">barometer_relativeAltitude:");
        Serial.println(barometer_relativeAltitude);
        Serial.print(">ofs_deltaX:");
        Serial.println(ofs_deltaX);
        Serial.print(">ofs_deltaY:");
        Serial.println(ofs_deltaY);
        Serial.print(">ofs_distanceX:");
        Serial.println(ofs_distanceX);
        Serial.print(">ofs_distanceY:");
        Serial.println(ofs_distanceY);
        Serial.print(">lidar_distance:");
        Serial.println(lidar_distance);
        Serial.print(">lidar_strength:");
        Serial.println(lidar_strength);
        Serial.print(">lidar_temperature:");
        Serial.println(lidar_temperature);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(nullptr);
}