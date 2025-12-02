#include <Arduino.h>
#include <math.h>
#include "Madgwick.h"

MadgwickFilter::MadgwickFilter(float betaVal) {
    beta = betaVal;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void MadgwickFilter::update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    const float deg2rad = 0.017453292519943295f;
    gx *= deg2rad;
    gy *= deg2rad;
    gz *= deg2rad;

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        float f1 = _2q1 * q3 - _2q0 * q2 - ax;
        float f2 = _2q0 * q1 + _2q2 * q3 - ay;
        float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;

        s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
        s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - _4q1 * f3;
        s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - _4q2 * f3;
        s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}