#ifndef QUAT_MATH_H
#define QUAT_MATH_H

/**
 * @brief Compute quaternion conjugate.
 * @param q0 Input quaternion w
 * @param q1 Input quaternion x
 * @param q2 Input quaternion y
 * @param q3 Input quaternion z
 * @param qc0 Output conjugate w
 * @param qc1 Output conjugate x
 * @param qc2 Output conjugate y
 * @param qc3 Output conjugate z
 */
void quatConjugate(float q0, float q1, float q2, float q3,
                   float &qc0, float &qc1, float &qc2, float &qc3);

/**
 * @brief Multiply two quaternions: q = a * b
 * @param a0,a1,a2,a3 Quaternion a
 * @param b0,b1,b2,b3 Quaternion b
 * @param q0,q1,q2,q3 Output quaternion product
 */
void quatMultiply(float a0, float a1, float a2, float a3,
                  float b0, float b1, float b2, float b3,
                  float &q0, float &q1, float &q2, float &q3);

/**
 * @brief Rotate vector by quaternion.
 * @param q0,q1,q2,q3 Quaternion
 * @param vx,vy,vz Input vector
 * @param ox,oy,oz Output rotated vector
 */
void quatRotateVector(float q0, float q1, float q2, float q3,
                      float vx, float vy, float vz,
                      float &ox, float &oy, float &oz);

#endif