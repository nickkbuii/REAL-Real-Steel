/**
 * @brief ESP32 main loop for dual-IMU orientation tracking.
 *
 * Reads two MPU6050 IMUs, runs Madgwick fusion on each, and sends both
 * resulting quaternions over BLE.
 */

#include <Arduino.h>
#include "MPU6050.h"
// #include "Madgwick.h"
// #include "QuatMath.h"
// #include "BLEManager.h"

MPU6050 imuUpper(0x68);
MPU6050 imuForearm(0x69);

// MadgwickFilter upperFilter;
// MadgwickFilter forearmFilter;

// unsigned long lastMicros = 0;

void setup() {
    Serial.begin(115200);
    delay(200);

    bool okUpper   = imuUpper.begin(11, 12);
    bool okForearm = imuForearm.begin(11, 12);

    Serial.println("Dual MPU6050 + Madgwick + BLE (ESP32)");
    Serial.printf("Upper IMU:   %s\n", okUpper   ? "OK" : "FAIL");
    Serial.printf("Forearm IMU: %s\n", okForearm ? "OK" : "FAIL");

    initBLE("NanoESP32");
    lastMicros = micros();
}

void loop() {
    // unsigned long now = micros();
    // float dt = (now - lastMicros) / 1e6f;
    // if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
    // lastMicros = now;

    float ax1, ay1, az1, gx1, gy1, gz1;
    float ax2, ay2, az2, gx2, gy2, gz2;

    imuUpper.readAccelG(ax1, ay1, az1);
    imuUpper.readGyroDPS(gx1, gy1, gz1);

    imuForearm.readAccelG(ax2, ay2, az2);
    imuForearm.readGyroDPS(gx2, gy2, gz2);

    // upperFilter.update(gx1, gy1, gz1, ax1, ay1, az1, dt);
    // forearmFilter.update(gx2, gy2, gz2, ax2, ay2, az2, dt);

    // updateBLEQuats(
    //     upperFilter.q0, upperFilter.q1, upperFilter.q2, upperFilter.q3,
    //     forearmFilter.q0, forearmFilter.q1, forearmFilter.q2, forearmFilter.q3
    // );

    delay(20);
}