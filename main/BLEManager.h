#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

/**
 * @brief Initialize BLE services and start advertising.
 *
 * @param deviceName  Name of the BLE peripheral.
 * @return void
 */
void initBLE(const char* deviceName = "NanoESP32");

/**
 * @brief Send two quaternion measurements over BLE.
 *
 * @param uq0 Upper-arm quaternion w
 * @param uq1 Upper-arm quaternion x
 * @param uq2 Upper-arm quaternion y
 * @param uq3 Upper-arm quaternion z
 * @param fq0 Forearm quaternion w
 * @param fq1 Forearm quaternion x
 * @param fq2 Forearm quaternion y
 * @param fq3 Forearm quaternion z
 *
 * @return void
 */
void updateBLEQuats(float uq0, float uq1, float uq2, float uq3,
                    float fq0, float fq1, float fq2, float fq3);

#endif