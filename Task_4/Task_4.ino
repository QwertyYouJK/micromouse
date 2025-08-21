#include "Task_4_motor.hpp"
#include "Task_4_DualEncoder.hpp"
#include "Task_4_EncoderOdometry.hpp"
#include "Task_4_PID_controller.hpp"
#include "Task_4_controller.hpp"
#include "Task_4_LIDAR.hpp"
#include "Task_4_IMUOdometry.hpp"
#include "Task_4_maze_map.hpp"
#include "Task_4_oled.hpp"
#include "Task_4_oled_mapping.hpp"


#include "Wire.h"
#include <MPU6050_light.h>

#define EN_1_A 2  //These are the pins for the PCB encoder
#define EN_1_B 7  //These are the pins for the PCB encoder
#define EN_2_A 3  //These are the pins for the PCB encoder
#define EN_2_B 8  //These are the pins for the PCB encoder

// define pins
#define MOTLPWM 11
#define MOTLDIR 12

#define MOTRPWM 9  // PIN 9 is a PWM pin
#define MOTRDIR 10

// define any other constants
// motor constants
#define MAXPWM 255 // max PWM
#define MOTPWM 30 // example PWM
#define ACPTPWM 50 // acceptable PWM
#define MOTOFF 0 // off
#define LEFTADJ -1 // adjustment values
#define RIGHTADJ 1 // adjustment values

// robot constants
#define WHRAD 22.5 // Wheel radius
#define AXLEN 103 // Axle length

// motor PID controller constants
#define MKP 8 // proportional gain
#define MKI 0.2 // integral gain
#define MKD 0.9 // derivative gain
#define MBOUND 2 // error, millimetres

// for turning:
#define TKP 50
#define TKI 1
#define TKD 2
#define ABOUND 0.01 // error, radians

#define SENSOR_LEFT A0
#define SENSOR_FRONT  A1
#define SENSOR_RIGHT A2

#define ADDR_FRONT 0x56
#define ADDR_LEFT  0x54
#define ADDR_RIGHT 0x55

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset = */ U8X8_PIN_NONE);

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHRAD, AXLEN);
mtrn3100::IMUOdometry IMU;

mtrn3100::PIDController turnPid(TKP, TKI, TKD);
mtrn3100::PIDController leftPid(MKP, MKI, MKD);
mtrn3100::PIDController rightPid(MKP, MKI, MKD);

mtrn3100::Lidar leftLidar(SENSOR_LEFT);
mtrn3100::Lidar frontLidar (SENSOR_FRONT);
mtrn3100::Lidar rightLidar(SENSOR_RIGHT);

mtrn3100::Controller controller(
  &encoder, 
  &encoder_odometry, 
  &leftMotor, 
  &rightMotor, 
  &leftPid, 
  &rightPid, 
  &turnPid, 
  frontLidar,
  leftLidar,
  rightLidar,
  &IMU
  );

mtrn3100::maze_map maze;
enum direction : int { north = 0, east = 1, south = 2, west = 3 };

int original_yaw;

// Initial robot state
int start_row = 0, start_col = 0;
int heading = 0; 


void setup() {
  Serial.begin(9600);
  Serial.println("Starting up!");

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clear();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "BallerinaArduina");


  //////////////////// Lidar setup ////////////////////
  frontLidar.begin(ADDR_FRONT);
  Serial.print("Front LiDAR ready @0x");
  Serial.println(ADDR_FRONT, HEX);

  leftLidar.begin(ADDR_LEFT);
  Serial.print("Left LiDAR ready @0x");
  Serial.println(ADDR_LEFT, HEX);

  rightLidar.begin(ADDR_RIGHT);
  Serial.print("Right LiDAR ready @0x");
  Serial.println(ADDR_RIGHT, HEX);

  //////////////////// IMU setup ////////////////////
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
  delay(1000);
  Serial.println("Done");
  IMU.update();
  original_yaw = IMU.get_yaw(); // Store original heading yaw
  Serial.print("IMU ready, facing: ");
  Serial.println(original_yaw);

  delay(100);

  mapping::commitCellAndRedraw(u8x8, maze, frontLidar, leftLidar, rightLidar,
                               start_row, start_col, heading);

  Serial.print(F("Start pose r=")); Serial.print(start_row);
  Serial.print(F(" c="));           Serial.print(start_col);
  Serial.print(F(" heading="));     Serial.println(heading);
}

void loop() {

  // Define goal cell 
  const int goal_r = 2;
  const int goal_c = 2;
  
  // Perform one autonomous mapping/navigation step
  autonom_map(start_row, start_col, goal_r, goal_c, heading);

  delay(10000);
}


// Flood fill to create a graph to map the maze to the goal while tracking where it is
void autonom_map(int start_r, int start_c, int goal_r, int goal_c, int heading) {
  int row = start_r;
  int col = start_c;

  while (!(row == goal_r && col == goal_c)) {

    // Update map with sensors for the *current cell*
    maze.update(row, col, heading,
                frontLidar.readMillimetres(),
                leftLidar.readMillimetres(),
                rightLidar.readMillimetres());

    // // Optional if you still keep flood fill for other logic
    // maze.flood_fill(goal_r, goal_c);

    // Choose a step that is strictly closer than the current cell
    int dir = maze.choose_best_dir(row, col, goal_r, goal_c);
    if (dir == -1) {
      // No strictly-closer neighbour → either at goal or boxed in
      break;
    }

    Serial.print("Moving from: ");
    Serial.print(row);
    Serial.print(" ");
    Serial.print(col);
    Serial.print("Heading ");
    Serial.println(heading);

    // Command the move (ideally blocking until cell center reached)
    controller.move_direction(dir);

    // Update discrete pose
    row += row_step[dir];
    col += col_step[dir];
    heading = dir;  // absolute headings must match N,E,S,W = 0..3
  }

  Serial.println("Reached goal!");
  leftMotor.setPWM(MOTOFF);
  rightMotor.setPWM(MOTOFF);
  delay(10000);
}
