#include <Arduino.h>

void setup()
{
    Serial.begin(115200);  // To PC via USB
    Serial2.begin(115200, 134217756U, 17, 4); // To TF-Luna via Pins 17(RX) and 4(TX)
    Serial.println("Lidar");
}

void loop()
{
    // Check if at least one 9-byte packet is in the buffer
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
                    int distance = (buffer[0] + (buffer[1] << 8)) * 10;
                    int strength = buffer[2] + (buffer[3] << 8);
                    float temp = (buffer[4] + (buffer[5] << 8)) / 8.0 - 256.0;

                    Serial.print(F(">lidar_distance:"));
                    Serial.print(distance);
                    Serial.println(" mm");
                    
                    Serial.print(F(">lidar_strength:"));
                    Serial.println(strength);

                    Serial.print(F(">lidar_temperature:"));
                    Serial.print(temp);
                    Serial.println(" *C");
                }
            }
        }
    }
}