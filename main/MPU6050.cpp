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
    ax = rx / ACC_SENS;
    ay = ry / ACC_SENS;
    az = rz / ACC_SENS;
}

void MPU6050::readGyroDPS(float &gx, float &gy, float &gz) {
    int16_t rx, ry, rz;
    readGyro(rx, ry, rz);
    gx = rx / GYR_SENS;
    gy = ry / GYR_SENS;
    gz = rz / GYR_SENS;
}