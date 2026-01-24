#include "tasks.h"
#include "common.h"

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
        PERF_START(sysMetrics.barometer);

        if (xSemaphoreTake(spiMutex, portMAX_DELAY))
        {
            sensors_event_t temp_event, pressure_event;
            if (dps.temperatureAvailable() && dps.pressureAvailable())
            {
                dps.getEvents(&temp_event, &pressure_event);
                temperature = temp_event.temperature;
                pressure = pressure_event.pressure;

                float temp = dps.readAltitude(pastPressure);
                if (abs(temp) < 5) relativeAltitude = temp;

                absoluteAltitude = dps.readAltitude(1013.25f);
                pastPressure = pressure_event.pressure;

                sysMetrics.barometerSensor.updateCount++;
            }
            xSemaphoreGive(spiMutex);
        }

        if (xSemaphoreTake(barometerMutex, portMAX_DELAY))
        {
            barometerData.temperature = temperature;
            barometerData.pressure = pressure;
            barometerData.absoluteAltitude = absoluteAltitude;
            barometerData.relativeAltitude = relativeAltitude;
            xSemaphoreGive(barometerMutex);
        }

        PERF_END(sysMetrics.barometer);
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
        PERF_START(sysMetrics.ofs);

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

        if(deltaX != 0 || deltaY != 0) sysMetrics.ofsSensor.updateCount++;

        PERF_END(sysMetrics.ofs);
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
        PERF_START(sysMetrics.lidar);

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

                        if (xSemaphoreTake(lidarMutex, portMAX_DELAY) && distance < 10000)
                        {
                            lidarData.distance = distance;
                            lidarData.strength = strength;
                            lidarData.temperature = temperature;
                            xSemaphoreGive(lidarMutex);
                        }
                        sysMetrics.lidarSensor.updateCount++;
                    }
                }
            }
        }
        PERF_END(sysMetrics.lidar);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void imu(void *params)
{

    auto setReports = [](void)
    {
        if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
            Serial.println("IMU Fail: Accel");
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
            Serial.println("IMU Fail: RotVec");
        if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
            Serial.println("IMU Fail: Gyro");
    };

    // Init
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    sh2_SensorValue_t sensorValue;

    // Enable reports once
    setReports();

    while (true)
    {
        PERF_START(sysMetrics.imu);

        // Handle sensor reset
        if (bno08x.wasReset()) {
            setReports();
        }

        // Lock SPI
        if (xSemaphoreTake(spi1Mutex, portMAX_DELAY))
        {
            if (bno08x.getSensorEvent(&sensorValue))
            {
                xSemaphoreGive(spi1Mutex);

                if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
                    sysMetrics.imuSensor.updateCount++;

                // Decode event
                switch (sensorValue.sensorId)
                {
                case SH2_LINEAR_ACCELERATION:
                    ax = sensorValue.un.linearAcceleration.x;
                    ay = sensorValue.un.linearAcceleration.y;
                    az = sensorValue.un.linearAcceleration.z;
                    break;

                case SH2_GYROSCOPE_CALIBRATED:
                    wx = sensorValue.un.gyroscope.x;
                    wy = sensorValue.un.gyroscope.y;
                    wz = sensorValue.un.gyroscope.z;
                    break;

                case SH2_GAME_ROTATION_VECTOR:
                    qw = sensorValue.un.gameRotationVector.real;
                    qx = sensorValue.un.gameRotationVector.i;
                    qy = sensorValue.un.gameRotationVector.j;
                    qz = sensorValue.un.gameRotationVector.k;
                    break;
                }
            }
            else
            {
                xSemaphoreGive(spi1Mutex);
            }
        }

        // Publish IMU data
        if (xSemaphoreTake(imuMutex, portMAX_DELAY))
        {
            imuData.ax = ax;
            imuData.ay = ay;
            imuData.az = az;

            imuData.p = wx;
            imuData.q = wy;
            imuData.r = wz;

            // Convert quaternion to Euler angles (in degrees)
            imuData.roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / PI);
            imuData.pitch = asinf(2.0f * (qw * qy - qz * qx)) * (180.0f / PI);
            imuData.yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / PI);

            xSemaphoreGive(imuMutex);
        }

        PERF_END(sysMetrics.imu);
        vTaskDelay(pdMS_TO_TICKS(5)); // 200 Hz
    }
    vTaskDelete(nullptr);
}

void ekf(void *pvParameters)
{    
    // Frequency of the IMU
    float T = 1.0f / 100.0f;  // 100 Hz
    Eigen::Matrix<double, 3,1> acc;
    Eigen::Matrix<double, 3,1> attitude;
    Eigen::Matrix<double, 3,3> R; // Rotation matrix

    // Identity matrix definition 
    Eigen::Matrix<double, 3, 3> I3 = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 4, 4> I4 = Eigen::Matrix<double, 4, 4>::Identity();

    Eigen::Matrix<double, 6, 1> Xkk_1 = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> Xkk;
    Eigen::Matrix<double, 6, 6> Pk_1k_1;
    Eigen::Matrix<double, 6, 6> Pkk_1;
    Eigen::Matrix<double, 6, 6> Pkk;

    // Prediction model matrices definition 
    Eigen::Matrix<double, 6, 6> Fk;
    Eigen::Matrix<double, 6, 6> Gk = Eigen::Matrix<double, 6, 6>::Identity()*T;
    Eigen::Matrix<double, 6, 6> Hk = Eigen::Matrix<double, 6, 6>::Identity()*0.1*T;

    // OFS Model 
    Eigen::Matrix<double, 2, 6> C_OFS = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Matrix<double, 2, 2> L_OFS = Eigen::Matrix<double, 2, 2>::Identity()*0.1;
    Eigen::Matrix<double, 2,2> Py_OFS;
    Eigen::Matrix<double, 6,2> K_OFS;
    Eigen::Matrix<double, 2,1> ymeas_OFS;
    Eigen::Matrix<double, 2,1> ypred_OFS;

    // Barometer model 
    Eigen::Matrix<double, 1, 6> C_baro = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Matrix<double, 1, 1> L_baro = Eigen::Matrix<double, 1, 1>::Identity()*0.1;
    Eigen::Matrix<double, 1,1> Py_baro;
    Eigen::Matrix<double, 6,1> K_baro;
    double ymeas_baro = 0.0;
    double ypred_baro = 0.0;

    Xkk_1.setZero();
    Xkk.setZero();
    Pk_1k_1.setIdentity();
    Pkk = Pk_1k_1;

    C_OFS.setZero();
    C_OFS(0,3) = 1;
    C_OFS(1,4) = 1;

    C_baro.setZero();
    C_baro(0,2) = 1;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    IMUData imu;
    BarometerData baro;
    OFSData ofs;

    for (;;)
    {
        PERF_START(sysMetrics.ekf);
        
        if (xSemaphoreTake(imuMutex, portMAX_DELAY)) {
            imu = imuData;
            xSemaphoreGive(imuMutex);
        }

        if (xSemaphoreTake(barometerMutex, 0)) {
            baro = barometerData;
            xSemaphoreGive(barometerMutex);
        }
        
        if (xSemaphoreTake(ofsMutex, 0)) {
            ofs = ofsData;
            xSemaphoreGive(ofsMutex);
        }

        // EKF steps
        Fk.setIdentity();
        Fk(0,3) = T;
        Fk(1,4) = T;
        Fk(2,5) = T;
        acc << imu.ax, imu.ay, imu.az;
        attitude << imu.roll * PI / 180.0f, imu.pitch * PI / 180.0f, imu.yaw * PI / 180.0f;

        // Rotation matrix from euler angles
        R = Eigen::AngleAxisd(attitude(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(attitude(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(attitude(0), Eigen::Vector3d::UnitX());
        
        Eigen::Vector3d g(0, 0, 9.81);
        
        Xkk_1.block<3,1>(0,0) += T * Xkk_1.block<3,1>(3,0) + 0.5 * T * T * ( R * (acc )-g); // position
        Xkk_1.block<3,1>(3,0) += T * ( R * (acc)-g );             // velocity
        Pkk_1 = Fk * Pk_1k_1 * Fk.transpose() + Hk;
        
        ypred_baro = C_baro * Xkk_1;
        ymeas_baro = baro.absoluteAltitude;
        Py_baro = C_baro * Pkk_1 * C_baro.transpose() + L_baro;
        K_baro = Pkk_1 * C_baro.transpose() * Py_baro.inverse();
        Xkk = Xkk_1 + K_baro * ( ymeas_baro - ypred_baro );
        Pkk = Pkk_1 - K_baro * C_baro * Pkk_1;
        
        ypred_OFS = C_OFS * Xkk;
        // TO BE CHECKED: VALUE SHOULD BE A SPEED
        ymeas_OFS << ofs.distanceX, ofs.distanceY;
        Py_OFS    = C_OFS * Pkk * C_OFS.transpose() + L_OFS;
        K_OFS     = Pkk * C_OFS.transpose() * Py_OFS.inverse();
        Xkk       = Xkk + K_OFS * (ymeas_OFS - ypred_OFS);
        Pkk       = Pkk - K_OFS * C_OFS * Pkk;
        
        
        Pk_1k_1 = Pkk;
        Xkk_1 = Xkk;
        
        if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
            stateEstimate.position = Xkk.block<3,1>(0,0);
            stateEstimate.velocity = Xkk.block<3,1>(3,0);
            xSemaphoreGive(stateMutex);
        }

        PERF_END(sysMetrics.ekf);
        vTaskDelayUntil(&last, period);
    }
}

void fiber(void *params)
{
    const TickType_t xDelay = 40 / portTICK_PERIOD_MS; // 25 Hz frequency
    const uint32_t FAILSAFE_TIMEOUT_MS = 500;          // 0.5s: Stop moving
    const uint32_t LANDING_TIMEOUT_MS = 3000;          // 3.0s: Start landing

    uint32_t lastPacketTime = millis();
    uint8_t latestCmd = 0;

    while (true)
    {
        PERF_START(sysMetrics.fiber);

        bool newPacketFound = false;

        // 1. Drain the buffer to find the most recent packet
        while (Serial3.available() >= 3)
        {
            if (Serial3.read() == 0xFE)
            {
                uint8_t cmd = Serial3.read();
                uint8_t footer = Serial3.read();
                if (footer == 0xFF)
                {
                    latestCmd = cmd;
                    newPacketFound = true;
                }
            }
        }

        if (xSemaphoreTake(fiberMutex, portMAX_DELAY))
        {
            if (newPacketFound)
            {
                lastPacketTime = millis(); // Reset heartbeat

                // Normal Command Processing
                fiberData.vx = 0;
                fiberData.vy = 0;
                fiberData.vz = 0;
                fiberData.yaw_rate = 0;

                if (latestCmd & (1 << 0))
                    fiberData.vx += 0.5; // Z
                if (latestCmd & (1 << 1))
                    fiberData.vy += 0.5; // Q
                if (latestCmd & (1 << 2))
                    fiberData.vx -= 0.5; // S
                if (latestCmd & (1 << 3))
                    fiberData.vy -= 0.5; // D
                if (latestCmd & (1 << 4))
                    fiberData.vz += 0.4; // I (Up)
                if (latestCmd & (1 << 5))
                    fiberData.vz -= 0.4; // K (Down)
                if (latestCmd & (1 << 6))
                    fiberData.yaw_rate += 30.0; // J
                if (latestCmd & (1 << 7))
                    fiberData.yaw_rate -= 30.0; // L
            }
            else
            {
                // 2. Failsafe Logic (No packet received)
                uint32_t elapsed = millis() - lastPacketTime;

                if (elapsed > LANDING_TIMEOUT_MS)
                {
                    // Stage 2: Sustained Loss -> Auto-Land
                    fiberData.vx = 0;
                    fiberData.vy = 0;
                    fiberData.yaw_rate = 0;
                    fiberData.vz = -0.3; // Gentle descent (approx 30cm/s)
                }
                else if (elapsed > FAILSAFE_TIMEOUT_MS)
                {
                    // Stage 1: Momentary Loss -> Hover Stationary
                    fiberData.vx = 0;
                    fiberData.vy = 0;
                    fiberData.vz = 0;
                    fiberData.yaw_rate = 0;
                }
            }
            xSemaphoreGive(fiberMutex);
        }

        PERF_END(sysMetrics.fiber);
        vTaskDelay(xDelay);
    }
    vTaskDelete(nullptr);
}

void esc(void *params)
{
    // 1. Attach Motors to Pins
    m1.attach(ESC1_PIN);
    m2.attach(ESC2_PIN);
    m3.attach(ESC3_PIN);
    m4.attach(ESC4_PIN);

    // 2. ARMING SEQUENCE
    // Most ESCs require 1000us for ~2 seconds to initialize
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    const TickType_t xDelay = 5 / portTICK_PERIOD_MS; // 200Hz Update rate

    while (true)
    {
        PERF_START(sysMetrics.esc);

        int pwm[4] = {1000, 1000, 1000, 1000};

        if (xSemaphoreTake(escMutex, portMAX_DELAY))
        {
            // Map 0.0-1.0 to 1000-2000 microseconds
            pwm[0] = 1000 + (int)(std::clamp(escData.m1, 0.0f, 1.0f) * 1000);
            pwm[1] = 1000 + (int)(std::clamp(escData.m2, 0.0f, 1.0f) * 1000);
            pwm[2] = 1000 + (int)(std::clamp(escData.m3, 0.0f, 1.0f) * 1000);
            pwm[3] = 1000 + (int)(std::clamp(escData.m4, 0.0f, 1.0f) * 1000);
            xSemaphoreGive(escMutex);
        }

        // 3. Output PWM signals
        m1.writeMicroseconds(pwm[0]);
        m2.writeMicroseconds(pwm[1]);
        m3.writeMicroseconds(pwm[2]);
        m4.writeMicroseconds(pwm[3]);

        PERF_END(sysMetrics.esc);
        vTaskDelay(xDelay);
    }
    vTaskDelete(nullptr);
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
    float imu_ax;
    float imu_ay;
    float imu_az;
    float imu_p;
    float imu_q;
    float imu_r;
    float imu_roll;
    float imu_pitch;
    float imu_yaw;

    while (true)
    {
        PERF_START(sysMetrics.logger);

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

        // Get imu data
        if (xSemaphoreTake(imuMutex, portMAX_DELAY))
        {
            imu_ax = imuData.ax;
            imu_ay = imuData.ay;
            imu_az = imuData.az;
            imu_p = imuData.p;
            imu_q = imuData.q;
            imu_r = imuData.r;
            imu_roll = imuData.roll;
            imu_pitch = imuData.pitch;
            imu_yaw = imuData.yaw;
            xSemaphoreGive(imuMutex);
        }

        // Print all data
        if (xSemaphoreTake(serialMutex, portMAX_DELAY))
        {
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
            Serial.print(">imu_ax:");
            Serial.println(imu_ax);
            Serial.print(">imu_ay:");
            Serial.println(imu_ay);
            Serial.print(">imu_az:");
            Serial.println(imu_az);
            Serial.print(">imu_p:");
            Serial.println(imu_p);
            Serial.print(">imu_q:");
            Serial.println(imu_q);
            Serial.print(">imu_r:");
            Serial.println(imu_r);
            Serial.print(">imu_roll:");
            Serial.println(imu_roll);
            Serial.print(">imu_pitch:");
            Serial.println(imu_pitch);
            Serial.print(">imu_yaw:");
            Serial.println(imu_yaw);
            xSemaphoreGive(serialMutex);
        }
        PERF_END(sysMetrics.logger);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(nullptr);
}

void perfMonitor(void *params) {
    char buffer[128];
    const TickType_t delay = pdMS_TO_TICKS(1000); // 1Hz update

    while(true) {
        // Calculate Sensor Actual Rates
        sysMetrics.barometerSensor.actualRate = (sysMetrics.barometerSensor.updateCount - sysMetrics.barometerSensor.lastCount);
        sysMetrics.barometerSensor.lastCount = sysMetrics.barometerSensor.updateCount;

        sysMetrics.imuSensor.actualRate = (sysMetrics.imuSensor.updateCount - sysMetrics.imuSensor.lastCount);
        sysMetrics.imuSensor.lastCount = sysMetrics.imuSensor.updateCount;

        sysMetrics.lidarSensor.actualRate = (sysMetrics.lidarSensor.updateCount - sysMetrics.lidarSensor.lastCount);
        sysMetrics.lidarSensor.lastCount = sysMetrics.lidarSensor.updateCount;

        // Print Table
        if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
            Serial.println("\n--- SYSTEM PERFORMANCE (1s) ---");
            Serial.println("TASK    | FREQ(Hz) | EXEC(us) | LOAD(%) | DATA(Hz)");
            
            snprintf(buffer, sizeof(buffer), "BARO    | %6.1f   | %6lu   | %5.2f   | %5.1f", 
                sysMetrics.barometer.frequency, sysMetrics.barometer.execTimeUs, sysMetrics.barometer.cpuLoad, sysMetrics.barometerSensor.actualRate);
            Serial.println(buffer);
            
            snprintf(buffer, sizeof(buffer), "OFS    | %6.1f   | %6lu   | %5.2f   | %5.1f", 
                sysMetrics.ofs.frequency, sysMetrics.ofs.execTimeUs, sysMetrics.ofs.cpuLoad, sysMetrics.ofsSensor.actualRate);
            Serial.println(buffer);

            snprintf(buffer, sizeof(buffer), "IMU     | %6.1f   | %6lu   | %5.2f   | %5.1f", 
                sysMetrics.imu.frequency, sysMetrics.imu.execTimeUs, sysMetrics.imu.cpuLoad, sysMetrics.imuSensor.actualRate);
            Serial.println(buffer);

            snprintf(buffer, sizeof(buffer), "EKF     | %6.1f   | %6lu   | %5.2f   | N/A", 
                sysMetrics.ekf.frequency, sysMetrics.ekf.execTimeUs, sysMetrics.ekf.cpuLoad);
            Serial.println(buffer);

            snprintf(buffer, sizeof(buffer), "ESC     | %6.1f   | %6lu   | %5.2f   | N/A", 
                sysMetrics.esc.frequency, sysMetrics.esc.execTimeUs, sysMetrics.esc.cpuLoad);
            Serial.println(buffer);
            
            snprintf(buffer, sizeof(buffer), "LIDAR   | %6.1f   | %6lu   | %5.2f   | %5.1f", 
                sysMetrics.lidar.frequency, sysMetrics.lidar.execTimeUs, sysMetrics.lidar.cpuLoad, sysMetrics.lidarSensor.actualRate);
            Serial.println(buffer);

            snprintf(buffer, sizeof(buffer), "FIBER   | %6.1f   | %6lu   | %5.2f   | N/A", 
                sysMetrics.fiber.frequency, sysMetrics.fiber.execTimeUs, sysMetrics.fiber.cpuLoad);
            Serial.println(buffer);

            Serial.println("------------------------------------");
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(delay);
    }
}
