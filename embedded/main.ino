#include <Arduino.h>
#include "MPU6050.h"
#include "Madgwick.h"
#include "QuatMath.h"
#include "BLEManager.h"

MPU6050 imuUpper(0x68);    // Bicep IMU
MPU6050 imuForearm(0x69);  // Forearm IMU

MadgwickFilter upperFilter;
MadgwickFilter forearmFilter;

unsigned long lastMicros = 0;

void setup() {
    Serial.begin(115200);
    delay(200);

    bool okUpper   = imuUpper.begin(21, 22);
    bool okForearm = imuForearm.begin(21, 22);

    Serial.println();
    Serial.println("Dual MPU6050 + Madgwick + BLE (ESP32)");
    Serial.print("Upper IMU init:   "); Serial.println(okUpper   ? "OK" : "FAIL");
    Serial.print("Forearm IMU init: "); Serial.println(okForearm ? "OK" : "FAIL");

    initBLE("NanoESP32");
    Serial.println("BLE is running...");

    lastMicros = micros();
}

void loop() {
    // Compute dt
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1e6f;
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.01f;
    }
    lastMicros = now;

    // Read scaled IMU data (g and deg/s)
    float ax1, ay1, az1, gx1, gy1, gz1;
    float ax2, ay2, az2, gx2, gy2, gz2;

    imuUpper.readAccelG(ax1, ay1, az1);
    imuUpper.readGyroDPS(gx1, gy1, gz1);

    imuForearm.readAccelG(ax2, ay2, az2);
    imuForearm.readGyroDPS(gx2, gy2, gz2);

    // Update orientation filters
    upperFilter.update(gx1, gy1, gz1, ax1, ay1, az1, dt);
    forearmFilter.update(gx2, gy2, gz2, ax2, ay2, az2, dt);

    // Compute forward vectors for each segment
    float ufx, ufy, ufz;
    float ffx, ffy, ffz;

    quatRotateVector(upperFilter.q0, upperFilter.q1, upperFilter.q2, upperFilter.q3,
                     1.0f, 0.0f, 0.0f, ufx, ufy, ufz);
    quatRotateVector(forearmFilter.q0, forearmFilter.q1, forearmFilter.q2, forearmFilter.q3,
                     1.0f, 0.0f, 0.0f, ffx, ffy, ffz);

    // Normalize vectors
    float ulen = sqrtf(ufx*ufx + ufy*ufy + ufz*ufz);
    float flen = sqrtf(ffx*ffx + ffy*ffy + ffz*ffz);

    if (ulen > 0.0f && flen > 0.0f) {
        ufx /= ulen; ufy /= ulen; ufz /= ulen;
        ffx /= flen; ffy /= flen; ffz /= flen;

        // Elbow angle (angle between upper and forearm forward vectors)
        float dot = ufx*ffx + ufy*ffy + ufz*ffz;
        if (dot >  1.0f) dot =  1.0f;
        if (dot < -1.0f) dot = -1.0f;

        float elbowDeg = acosf(dot) * 57.2957795f;

        Serial.println("----");
        Serial.printf("Upper q:   %.3f %.3f %.3f %.3f\n",
                      upperFilter.q0, upperFilter.q1, upperFilter.q2, upperFilter.q3);
        Serial.printf("Forearm q: %.3f %.3f %.3f %.3f\n",
                      forearmFilter.q0, forearmFilter.q1, forearmFilter.q2, forearmFilter.q3);
        Serial.printf("Elbow angle [deg]: %.1f\n", elbowDeg);
        Serial.printf("Punch dir (forearm): [%.2f, %.2f, %.2f]\n", ffx, ffy, ffz);
    }

    // Send forearm IMU data over BLE (scaled)
    updateBLE(ax2, ay2, az2, gx2, gy2, gz2);

    delay(20); // ~50 Hz loop
}