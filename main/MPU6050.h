#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    /**
    * @brief Construct a new MPU6050 object
    */
    MPU6050(uint8_t addr = 0x68);

    /**
    * @brief Initialize the MPU6050 sensor
    * @return true if initialization is successful, false otherwise
    */
    bool begin(int sda = 21, int scl = 22);

    /**
     * @brief Read raw accelerometer values
     */
    void readAccel(int16_t &ax, int16_t &ay, int16_t &az);


    /**
     * @brief Read raw gyroscope values
     */
    void readGyro(int16_t &gx, int16_t &gy, int16_t &gz);

    /**
     * @brief Read accelerometer values scaled to 'g'
     */
    void readAccelG(float &ax, float &ay, float &az);

    /**
     * @brief Read gyroscope values scaled to deg/s
     */
    void readGyroDPS(float &gx, float &gy, float &gz);

    /**
     * @brief Read accelerometer values scaled to m/s²
     */
    void readAccelms2(float &ax, float &ay, float &az);

    /**
     * @brief Read gyroscope values scaled to rad/s
     */
    void readGyroRads(float &gx, float &gy, float &gz);


    /**
     * @brief Calibration function to determine and set biases for accelerometer and gyroscope
     */
    void calibrate(int samples = 1000);

private:
    uint8_t _addr;
    float ax_bias = 0.0f;
    float ay_bias = 0.0f;
    float az_bias = 0.0f;
    float gx_bias = 0.0f;
    float gy_bias = 0.0f;
    float gz_bias = 0.0f;
};

#endif