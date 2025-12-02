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

#define upper_ax_bias 0.04
#define upper_ay_bias 0.005
#define upper_az_bias 0.130
#define upper_gx_bias 0.244
#define upper_gy_bias 0.649
#define upper_gz_bias 1.008

#define fore_ax_bias -0.026
#define fore_ay_bias 0.004
#define fore_az_bias -0.008
#define fore_gx_bias 1.824
#define fore_gy_bias 0.794
#define fore_gz_bias 0.107

MPU6050 imuUpper(0x68);
MPU6050 imuForearm(0x69);

// MadgwickFilter upperFilter;
// MadgwickFilter forearmFilter;

// unsigned long lastMicros = 0;

void setup() {
    Serial.begin(115200);
    delay(200);

    bool okUpper   = imuUpper.begin(A4, A5);
    bool okForearm = imuForearm.begin(A4, A5);

    Serial.println("Dual MPU6050 + Madgwick + BLE (ESP32)");
    Serial.printf("Upper IMU:   %s\n", okUpper   ? "OK" : "FAIL");
    Serial.printf("Forearm IMU: %s\n", okForearm ? "OK" : "FAIL");
    

    // initBLE("NanoESP32");
    // lastMicros = micros();



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


    ax1 -= upper_ax_bias;
    ay1 -= upper_ay_bias;
    az1 -= upper_az_bias;

    gx1 -= upper_gx_bias;
    gy1 -= upper_gy_bias;
    gz1 -= upper_gz_bias;

    ax2 -= fore_ax_bias;
    ay2 -= fore_ay_bias;
    az2 -= fore_az_bias;

    gx2 -= fore_gx_bias;
    gy2 -= fore_gy_bias;
    gz2 -= fore_gz_bias;

    // Serial.println("==== Upper IMU ====");
    // Serial.print("Accel (G): ");
    // Serial.print("ax1="); Serial.print(ax1, 3); Serial.print("  ");
    // Serial.print("ay1="); Serial.print(ay1, 3); Serial.print("  ");
    // Serial.print("az1="); Serial.println(az1, 3);

    // Serial.print("Gyro (DPS): ");
    // Serial.print("gx1="); Serial.print(gx1, 3); Serial.print("  ");
    // Serial.print("gy1="); Serial.print(gy1, 3); Serial.print("  ");
    // Serial.print("gz1="); Serial.println(gz1, 3);


    // Serial.println("==== Forearm IMU ====");
    // Serial.print("Accel (G): ");
    // Serial.print("ax2="); Serial.print(ax2, 3); Serial.print("  ");
    // Serial.print("ay2="); Serial.print(ay2, 3); Serial.print("  ");
    // Serial.print("az2="); Serial.println(az2, 3);

    // Serial.print("Gyro (DPS): ");
    // Serial.print("gx2="); Serial.print(gx2, 3); Serial.print("  ");
    // Serial.print("gy2="); Serial.print(gy2, 3); Serial.print("  ");
    // Serial.print("gz2="); Serial.println(gz2, 3);

    // Serial.println();

    Serial.print("ax1:"); Serial.print(ax1);
    Serial.print(" ay1:"); Serial.print(ay1);
    Serial.print(" az1:"); Serial.print(az1);

    Serial.print(" gx1:"); Serial.print(gx1);
    Serial.print(" gy1:"); Serial.print(gy1);
    Serial.print(" gz1:"); Serial.print(gz1);

    Serial.print(" ax2:"); Serial.print(ax2);
    Serial.print(" ay2:"); Serial.print(ay2);
    Serial.print(" az2:"); Serial.print(az2);

    Serial.print(" gx2:"); Serial.print(gx2);
    Serial.print(" gy2:"); Serial.print(gy2);
    Serial.print(" gz2:"); Serial.println(gz2);


    

    // upperFilter.update(gx1, gy1, gz1, ax1, ay1, az1, dt);
    // forearmFilter.update(gx2, gy2, gz2, ax2, ay2, az2, dt);

    // updateBLEQuats(
    //     upperFilter.q0, upperFilter.q1, upperFilter.q2, upperFilter.q3,
    //     forearmFilter.q0, forearmFilter.q1, forearmFilter.q2, forearmFilter.q3
    // );

    delay(20);
}