#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    MPU6050(uint8_t addr = 0x68);
    bool begin(int sda = 21, int scl = 22);

    void readAccel(int16_t &ax, int16_t &ay, int16_t &az);
    void readGyro(int16_t &gx, int16_t &gy, int16_t &gz);

    /**
     * @brief Read accelerometer values scaled to 'g'
     */
    void readAccelG(float &ax, float &ay, float &az);

    /**
     * @brief Read gyroscope values scaled to deg/s
     */
    void readGyroDPS(float &gx, float &gy, float &gz);

private:
    uint8_t _addr;
};

#endif