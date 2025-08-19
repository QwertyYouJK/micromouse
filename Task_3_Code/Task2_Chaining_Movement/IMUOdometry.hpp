#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry() : x(0), y(0), z(0), vx(0), vy(0), vz(0), yaw_angle(0), lastUpdateTime(millis()) {}
        
        void update() {
            mpu.update();
        }

        void update_xyz(float accel_x, float accel_y, float accel_z) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime);  // Convert to seconds
            lastUpdateTime = currentTime;

            // Differentiate acceleration to get velocity
            vx += accel_x * dt/1000;
            vy += accel_y * dt/1000;
            vz += accel_z * dt/1000;

            // Differentiate velocity to get position
            x += vx * dt/1000;
            y += vy * dt/1000;
            z += vz * dt/1000;
        }
        
        float getX() const { return x; }
        float getY() const { return y; }
        float getZ() const { return z; }

        float get_accX() { return mpu.getAccX(); }
        float get_accY() { return mpu.getAccY(); }
        float get_accZ() { return mpu.getAccZ(); }

        float get_yaw() { return mpu.getAngleZ(); }
        
    private:
        float x, y, z;
        float vx, vy, vz;
        float yaw_angle;
        unsigned long lastUpdateTime;
    };
}

#endif // IMU_ODOMETRY_HPP