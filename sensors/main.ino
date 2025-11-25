#include <Arduino.h>
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    mpu.begin();
}

void loop() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.readAccel(ax, ay, az);
    mpu.readGyro(gx, gy, gz);

    Serial.printf("Accel: %d %d %d | Gyro: %d %d %d\n",
                  ax, ay, az, gx, gy, gz);

    delay(200);
}