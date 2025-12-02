#ifndef MADGWICK_H
#define MADGWICK_H

/**
 * @brief Madgwick AHRS filter for IMU orientation estimation.
 *
 * Uses gyroscope and accelerometer to estimate orientation as a unit quaternion.
 */
class MadgwickFilter {
public:
    float beta; // Algorithm gain.
    float q0, q1, q2, q3; // Orientation quaternion (w, x, y, z).

    /**
     * @brief Construct the filter with an initial gain and identity quaternion.
     * @param betaVal Algorithm gain controlling correction vs. responsiveness.
     */
    MadgwickFilter(float betaVal = 0.2f);

    /**
     * @brief Update filter state from IMU data.
     * @param gx Gyro X [deg/s].
     * @param gy Gyro Y [deg/s].
     * @param gz Gyro Z [deg/s].
     * @param ax Accel X [g] or normalized units.
     * @param ay Accel Y [g] or normalized units.
     * @param az Accel Z [g] or normalized units.
     * @param dt Time step [s] since last update.
     */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt);
};

#endif