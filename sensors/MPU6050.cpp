#include "MPU6050.h"

MPU6050::MPU6050(uint8_t addr) : _addr(addr) {}

bool MPU6050::begin(int sda, int scl) {
    Wire.begin(sda, scl);

    // Wake up MPU6050
    Wire.beginTransmission(_addr);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00);
    return (Wire.endTransmission() == 0);
}

void MPU6050::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);

    Wire.requestFrom(_addr, 6, true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
}

void MPU6050::readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(_addr);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);

    Wire.requestFrom(_addr, 6, true);

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}