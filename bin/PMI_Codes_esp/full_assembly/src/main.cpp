#include <Arduino.h>
#include "common.h"
#include "tasks.h"

SPIClass &hspi = SPI1; // SPI1 = HSPI
SPIClass &vspi = SPI;  // VSPI

Adafruit_DPS310 dps;
Bitcraze_PMW3901 flow(PMW3901_CS);
Adafruit_BNO08x bno08x(BNO08X_RESET);

// Structs
BarometerData barometerData;
OFSData ofsData;
LidarData lidarData;
IMUData imuData;

// Mutexes
SemaphoreHandle_t barometerMutex;
SemaphoreHandle_t ofsMutex;
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t vspiMutex;
SemaphoreHandle_t hspiMutex;


void setup() {

    // Init UART peripherals
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial2.begin(115200);
    while (!Serial2) delay(10);

    // Init mutexes
    barometerMutex = xSemaphoreCreateMutex();
    ofsMutex = xSemaphoreCreateMutex();
    lidarMutex = xSemaphoreCreateMutex();
    imuMutex = xSemaphoreCreateMutex();
    vspiMutex = xSemaphoreCreateMutex();
    hspiMutex = xSemaphoreCreateMutex();

    
    // Setup SPI slave CS pins
    pinMode(DPS310_CS, OUTPUT);
    digitalWrite(DPS310_CS, HIGH);

    pinMode(PMW3901_CS, OUTPUT);
    digitalWrite(PMW3901_CS, HIGH);

    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(BNO08X_CS, HIGH);

    // Start SPI peripherals
    vspi.begin(); // Init VSPI
    hspi.begin(); // Init HSPI

    // Init flags
    bool baroInit = false;
    bool ofsInit = false;
    bool imuInit = false;
    
    // Init sensors
    if (!dps.begin_SPI(DPS310_CS, &hspi)) // Barometer
    {
        Serial.println("Initialization of the barometer failed.");
    }
    else
    {
        dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
        dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
        baroInit = true;
        Serial.println("Initialization of the barometer successful.");
    }

    
    if (!flow.begin()) // OFS
    {
        Serial.println("Initialization of the ofs failed.");
        while(true)
        ;
    }
    else
    {
        Serial.println("Initialization of the ofs successful.");
        ofsInit = true;
    }

    if (!bno08x.begin_SPI(BNO08X_CS,-1, &hspi)) // IMU
    {
        Serial.println("Initialization of the imu failed.");
    }
    else
    {
        // bno08x.enableReport(SH2_LINEAR_ACCELERATION);
        // bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
        // bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
        imuInit = true;
        Serial.println("Initialization of the imu successful.");
    }

    // Create tasks
    if (baroInit)
    {
        xTaskCreate(barometer, "barometer", 4096, nullptr, 2, nullptr);
    }

    if (ofsInit)
    {
        xTaskCreate(ofs, "ofs", 4096, nullptr, 2, nullptr);
    }
    
    if (imuInit)
    {
        xTaskCreate(imu, "imu", 512, nullptr, 1, nullptr);
    }
    
    xTaskCreate(lidar, "lidar", 2048, nullptr, 2, nullptr);
    xTaskCreate(logger, "logger", 2048, nullptr, 2, nullptr);

    // Run tasks
    vTaskStartScheduler();
}

void loop() {
}