#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <SPI.h>

Adafruit_DPS310 dps;
#define DPS310_CS 22
#define DPS310_MOSI 13
#define DPS310_MISO 14
#define DPS310_SCK 26

SPIClass vspi(VSPI);

float rel_altitude = 0;
float past_pressure = 1013.25f;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("DPS310");

    pinMode(DPS310_CS, OUTPUT);
    digitalWrite(DPS310_CS, HIGH);

    vspi.begin(DPS310_SCK, DPS310_MISO, DPS310_MOSI, DPS310_CS);

    if (!dps.begin_SPI(DPS310_CS, &vspi)) {
        Serial.println("Failed to find DPS");
        while (1)
            yield();
    }
    Serial.println("DPS OK!");
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}
void loop()
{
    sensors_event_t temp_event, pressure_event;
    while (!dps.temperatureAvailable() || !dps.pressureAvailable())
    {
        return; // wait until there's something to read
    }
    dps.getEvents(&temp_event, &pressure_event);
    Serial.print(F(">temperature:"));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.print(F(">pressure:"));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");
    Serial.println();

    float temp = dps.readAltitude(past_pressure);
    if (abs(temp) < 5)
    {
        rel_altitude = temp;
    }
    Serial.print(">relative_altitude:");
    Serial.println(double(rel_altitude));

    Serial.print(">altitude:");
    Serial.println(dps.readAltitude(1013.25f));
    Serial.println();
    past_pressure = pressure_event.pressure;
}
