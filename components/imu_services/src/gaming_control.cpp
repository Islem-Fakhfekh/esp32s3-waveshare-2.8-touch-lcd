/**
 * @file gaming_control.cpp
 * @brief Gaming Control Service (Tilt detection)
 */

#include <cmath>

#include <cmath>
#include "esp_log.h"
#include "imu_services.hpp"

constexpr static const char *TAG = "GamingControl";

namespace ImuServices {

TiltAngles getTiltAngles(qmi8658_dev_t *imu) {
    TiltAngles angles = {0.0, 0.0};

    if (imu == nullptr) {
        return angles;
    }

    float accelX = NAN;
    float accelY = NAN;
    float accelZ = NAN;
    if (qmi8658_read_accel(imu, &accelX, &accelY, &accelZ) != ESP_OK) {
        return angles;
    }

    // Calculate pitch and roll in degrees
    angles.pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
    angles.roll = atan2(accelY, accelZ) * 180.0 / M_PI;

    return angles;
}

float getTiltX(qmi8658_dev_t *imu, float maxTiltDeg) {
    TiltAngles angles = getTiltAngles(imu);

    // Roll: -180 to +180°
    // Normalize to -1.0 (left) to +1.0 (right)
    float normalized = angles.roll / maxTiltDeg;

    // Clamp to [-1.0, 1.0]
    if (normalized < -1.0) {
        normalized = -1.0;
    }
    if (normalized > 1.0) {
        normalized = 1.0;
    }

    return normalized;
}

float getTiltY(qmi8658_dev_t *imu, float maxTiltDeg) {
    TiltAngles angles = getTiltAngles(imu);

    // Pitch: -90 to +90°
    // Normalize to -1.0 (backward) to +1.0 (forward)
    float normalized = angles.pitch / maxTiltDeg;

    // Clamp to [-1.0, 1.0]
    if (normalized < -1.0) {
        normalized = -1.0;
    }
    if (normalized > 1.0) {
        normalized = 1.0;
    }

    return normalized;
}

}  // namespace ImuServices
