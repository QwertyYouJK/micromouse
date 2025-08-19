#pragma once

#include <Arduino.h>
#include "../Task_2_Code/Task2_LiDAR/motor.hpp"
#include "../Task_2_Code/Task2_LiDAR/PIDcontroller.hpp"
#include "../Task_2_Code/Task2_LiDAR/LIDAR.hpp"

#define BUFFERDIST 10 // placeholder, mm

//  drive correction

// check if robot is vearing towards a wall when driving straight
    // if lidar values are decreasing by X value and distance is less than Y value
        // turn slightly opposite direction
    
    // if lidar values are increasing by -X value and distance is greater than Z value
        // turn slightly opposite direction


namespace mtrn3100 {
    
class DriveCorrection {
public: 
    DriveCorrection(Motor* leftM, Motor* rightM, Lidar& leftLidar, Lidar& rightLidar):
    left_motor(leftM),
    right_motor(rightM),
    left_lidar(leftLidar), 
    right_lidar(rightLidar) 
    {}

    void at_movement_start {
        uint16_t left_prev_dist = left_lidar.readMillimetres();
        uint16_t right_prev_dist = right_lidar.readMillimetres();
    }

    void deltaLidar {
        uint16_t left_curr_dist = left_lidar.readMillimetres();
        uint16_t right_curr_dist = right_lidar.readMillimetres();

        if ((left_curr_dist - left_prev_dist) < BUFFERDIST) {

        }


    }

    void closeCorr {
        //a
    }

    void farCorr {
        //b
    }



private:
    Motor* empty; // does nothing, code isnt working without it though
    Lidar& left_lidar;
    Lidar& right_lidar;
    Motor* left_motor;
    Motor* right_motor;
};
}
