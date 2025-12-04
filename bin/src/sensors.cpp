#include "common.h"
#include "sensors.h"

// IMU object
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t imuSensorValue;

bool init_sensors()
{

    // IMU initialization
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT))
    {
        Serial.println("Failed to find BNO08x chip");
        return false;
    }
    delay(100);
    if (!setReports())
    {
        Serial.println("Failed to set reports");
        return false;
    }
    
    return true;
}

// === OPTICAL FLOW FUNCTIONS ===
void optical_flow(void *parameters)
{
}

void FASTRUN optical_flow_isr()
{
    BaseType_t TaskWoken = pdFALSE;

    // Notify the task that the IRQ was sent
    vTaskNotifyGiveFromISR(imu_handle, &TaskWoken);
    
    // Force context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(TaskWoken);
}

// === IMU FUNCTIONS ===
bool setReports()
{
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        return false;
    }
    return true;
}

void FASTRUN imu_isr()
{
    BaseType_t TaskWoken = pdFALSE;

    // Notify the task that the IRQ was sent
    vTaskNotifyGiveFromISR(imu_handle, &TaskWoken);
    
    // Force context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(TaskWoken);
}

void imu(void *parameters)
{
    // Make sure to have correct reports
    if (bno08x.wasReset())
    {
        Serial.print("sensor was reset ");
        setReports();
    }

    // Read sensor data
    if (bno08x.getSensorEvent(&imuSensorValue))
    {
        switch (imuSensorValue.sensorId)
        {
        case SH2_GAME_ROTATION_VECTOR:
            Serial.print("Game Rotation Vector - r: ");
            Serial.print(imuSensorValue.un.gameRotationVector.real);
            Serial.print(" i: ");
            Serial.print(imuSensorValue.un.gameRotationVector.i);
            Serial.print(" j: ");
            Serial.print(imuSensorValue.un.gameRotationVector.j);
            Serial.print(" k: ");
            Serial.println(imuSensorValue.un.gameRotationVector.k);
            break;
        }
    }
}

// === BAROMETER FUNCTIONS ===
void barometer(void *parameters)
{
}

void FASTRUN barometer_isr()
{
    BaseType_t TaskWoken = pdFALSE;

    // Notify the task that the IRQ was sent
    vTaskNotifyGiveFromISR(barometer_handle, &TaskWoken);

    // Force context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(TaskWoken);
}

// === LIDAR FUNCTIONS ===
void lidar(void *parameters)
{
}