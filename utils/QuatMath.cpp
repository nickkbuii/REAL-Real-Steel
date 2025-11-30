#include "QuatMath.h"

void quatConjugate(float q0, float q1, float q2, float q3,
                   float &qc0, float &qc1, float &qc2, float &qc3) {
    qc0 = q0;
    qc1 = -q1;
    qc2 = -q2;
    qc3 = -q3;
}

void quatMultiply(float a0, float a1, float a2, float a3,
                  float b0, float b1, float b2, float b3,
                  float &q0, float &q1, float &q2, float &q3) {
    q0 = a0*b0 - a1*b1 - a2*b2 - a3*b3;
    q1 = a0*b1 + a1*b0 + a2*b3 - a3*b2;
    q2 = a0*b2 - a1*b3 + a2*b0 + a3*b1;
    q3 = a0*b3 + a1*b2 - a2*b1 + a3*b0;
}

void quatRotateVector(float q0, float q1, float q2, float q3,
                      float vx, float vy, float vz,
                      float &ox, float &oy, float &oz) {

    float qc0 = q0;
    float qc1 = -q1;
    float qc2 = -q2;
    float qc3 = -q3;

    float w0 = -q1*vx - q2*vy - q3*vz;
    float w1 =  q0*vx + q2*vz - q3*vy;
    float w2 =  q0*vy - q1*vz + q3*vx;
    float w3 =  q0*vz + q1*vy - q2*vx;

    float r1 = w0*qc1 + w1*qc0 + w2*qc3 - w3*qc2;
    float r2 = w0*qc2 - w1*qc3 + w2*qc0 + w3*qc1;
    float r3 = w0*qc3 + w1*qc2 - w2*qc1 + w3*qc0;

    ox = r1;
    oy = r2;
    oz = r3;
}