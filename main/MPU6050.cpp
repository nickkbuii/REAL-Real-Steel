#include "MPU6050.h"

static constexpr float ACC_SENS = 16384.0f;
static constexpr float GYR_SENS = 131.0f;

MPU6050::MPU6050(uint8_t addr) : _addr(addr) {}

bool MPU6050::begin(int sda, int scl) {
    static bool wireInitialized = false;
    if (!wireInitialized) {
        Wire.begin(sda, scl);
        wireInitialized = true;
    }

    Wire.beginTransmission(_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    return (Wire.endTransmission() == 0);
}

void MPU6050::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom((int)_addr, 6, true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
}

void MPU6050::readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);

    Wire.requestFrom((int)_addr, 6, true);

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

void MPU6050::readAccelG(float &ax, float &ay, float &az) {
    int16_t rx, ry, rz;
    readAccel(rx, ry, rz);

    ax = (static_cast<float>(rx) - ax_bias) / ACC_SENS;
    ay = (static_cast<float>(ry) - ay_bias) / ACC_SENS;
    az = (static_cast<float>(rz) - az_bias) / ACC_SENS;
}

void MPU6050::readGyroDPS(float &gx, float &gy, float &gz) {
    int16_t rx, ry, rz;
    readGyro(rx, ry, rz);

    gx = (static_cast<float>(rx) - gx_bias) / GYR_SENS;
    gy = (static_cast<float>(ry) - gy_bias) / GYR_SENS;
    gz = (static_cast<float>(rz) - gz_bias) / GYR_SENS;
}

void MPU6050::calibrate(int samples) {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; ++i) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        readAccel(ax, ay, az);
        readGyro(gx, gy, gz);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(2);
    }

    float ax_avg = static_cast<float>(ax_sum) / samples;
    float ay_avg = static_cast<float>(ay_sum) / samples;
    float az_avg = static_cast<float>(az_sum) / samples;

    float gx_avg = static_cast<float>(gx_sum) / samples;
    float gy_avg = static_cast<float>(gy_sum) / samples;
    float gz_avg = static_cast<float>(gz_sum) / samples;

    ax_bias = ax_avg;
    ay_bias = ay_avg;
    az_bias = az_avg - ACC_SENS;

    gx_bias = gx_avg;
    gy_bias = gy_avg;
    gz_bias = gz_avg;
}