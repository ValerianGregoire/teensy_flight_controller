#include <Arduino.h>
#include "common.h"
#include "tasks.h"

// Objects
SPIClass &spi = SPI;
SPIClass &spi1 = SPI1;

Adafruit_DPS310 dps;
Bitcraze_PMW3901 flow(PMW3901_CS);
Adafruit_BNO08x bno08x(BNO08X_RESET);

Servo m1, m2, m3, m4;

// Structs
BarometerData barometerData;
OFSData ofsData;
LidarData lidarData;
IMUData imuData;
StateEstimate stateEstimate;
FiberData fiberData;
ESCData escData;

// Mutexes
SemaphoreHandle_t barometerMutex;
SemaphoreHandle_t ofsMutex;
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t spi1Mutex;
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t fiberMutex;
SemaphoreHandle_t escMutex;

void setup() {

    // Init UART peripherals
    Serial.begin(115200); // Logs
    while (!Serial) delay(10);
    
    Serial2.begin(115200); // Lidar
    while (!Serial2) delay(10);
    
    Serial3.begin(115200); // Optic Fiber
    while (!Serial3) delay(10);

    // Init mutexes
    barometerMutex = xSemaphoreCreateMutex();
    ofsMutex = xSemaphoreCreateMutex();
    lidarMutex = xSemaphoreCreateMutex();
    imuMutex = xSemaphoreCreateMutex();
    spiMutex = xSemaphoreCreateMutex();
    spi1Mutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    fiberMutex = xSemaphoreCreateMutex();

    // Setup SPI slave CS pins
    pinMode(DPS310_CS, OUTPUT);
    digitalWrite(DPS310_CS, HIGH);

    pinMode(PMW3901_CS, OUTPUT);
    digitalWrite(PMW3901_CS, HIGH);

    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(BNO08X_CS, HIGH);

    pinMode(BNO08X_INT, OUTPUT);
    digitalWrite(BNO08X_INT, HIGH);

    pinMode(BNO08X_RESET, OUTPUT);
    digitalWrite(BNO08X_RESET, HIGH);

    // Start SPI peripherals
    spi.begin(); // Init SPI

    spi1.setMOSI(SPI1_MOSI);
    spi1.setMISO(SPI1_MISO);
    spi1.setSCK(SPI1_SCK);
    spi1.begin();

    // Init flags
    bool baroInit = false;
    bool ofsInit = false;
    bool imuInit = false;
    
    // Init sensors
    if (!dps.begin_SPI(DPS310_CS, &spi1)) // Barometer
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
    }
    else
    {
        Serial.println("Initialization of the ofs successful.");
        ofsInit = true;
    }

    
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &spi1)) // IMU
    {
        Serial.println("Initialization of the imu failed.");
    }
    else
    {
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
        xTaskCreate(imu, "imu", 4096, nullptr, 1, nullptr);
    }
    
    xTaskCreate(lidar, "lidar", 2048, nullptr, 2, nullptr);
    xTaskCreate(fiber, "fiber", 2048, nullptr, 2, nullptr);
    xTaskCreate(fiber, "esc", 2048, nullptr, 2, nullptr);
    xTaskCreate(logger, "logger", 2048, nullptr, 2, nullptr);

    // Run tasks
    vTaskStartScheduler();
}

void loop() {
}