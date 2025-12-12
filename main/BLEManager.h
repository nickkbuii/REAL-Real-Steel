#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

/**
 * @brief Initialize BLE with the given device name.
 * @return true if initialization was successful, false otherwise.
 */
bool initBLE(const char* deviceName = "NanoESP32");

/**
 * @brief Send two IMU measurements over BLE.
 *
 * Payload format (CSV, UTF-8):
 *   ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2
 *
 * Units:
 *   accel: m/s^2
 *   gyro:  rad/s
 */
void updateBLEIMU(float ax1, float ay1, float az1,
                  float gx1, float gy1, float gz1,
                  float ax2, float ay2, float az2,
                  float gx2, float gy2, float gz2);

#endif