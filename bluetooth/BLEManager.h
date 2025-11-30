#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

/**
 * @brief Initialize BLE stack, service, and characteristic.
 *
 * Creates a BLE server, service (UUID 0x180A), and a notify-capable
 * characteristic (UUID 0x2A57). Starts advertising automatically.
 *
 * @param deviceName Name of the BLE peripheral (e.g. "NanoESP32").
 */
void initBLE(const char* deviceName = "NanoESP32");

/**
 * @brief Send IMU data over BLE if a client is connected.
 *
 * Data is sent as a comma-separated ASCII string:
 * "ax,ay,az,gx,gy,gz" with 3 decimal places.
 *
 * @param ax Acceleration X [g or normalized].
 * @param ay Acceleration Y [g or normalized].
 * @param az Acceleration Z [g or normalized].
 * @param gx Angular rate X [deg/s or normalized].
 * @param gy Angular rate Y [deg/s or normalized].
 * @param gz Angular rate Z [deg/s or normalized].
 */
void updateBLE(float ax, float ay, float az,
               float gx, float gy, float gz);

#endif