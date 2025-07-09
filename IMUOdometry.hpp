#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry() : x(0), y(0), vx(0), vy(0), lastUpdateTime(millis()) {}

        void update(float accel_x, float accel_y) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime);  // Convert to seconds
            lastUpdateTime = currentTime;

            // Differentiate acceleration to get velocity
            vx += accel_x * dt/1000;
            vy += accel_y * dt/1000;

            // TODO: Differentiate velocity to get position
            x += vx * dt/1000;
            y += vy * dt/1000;
        }

        float getX() const { return x; }
        float getY() const { return y; }

    private:
        float x, y;
        float vx, vy;
        unsigned long lastUpdateTime;
    };
}

#endif // IMU_ODOMETRY_HPP
