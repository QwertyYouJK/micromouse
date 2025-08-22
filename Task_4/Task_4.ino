#include "Task_4_motor.hpp"
#include "Task_4_DualEncoder.hpp"
#include "Task_4_EncoderOdometry.hpp"
#include "Task_4_PID_controller.hpp"
#include "Task_4_controller.hpp"
#include "Task_4_LIDAR.hpp"
#include "Task_4_IMUOdometry.hpp"

#include "Task_4_oled_new.hpp"
// #include "Task_4_autonom.hpp"
// #include "Task_4_oled.hpp"
// #include "Task_4_oled_mapping.hpp"
// #include "Task_4_maze_map.hpp"

#include "Stack.h"

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
#define ACPTPWM 100 // acceptable PWM
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

// // motor PID controller constants
// #define MKP 100 // proportional gain
// #define MKI 1 // integral gain
// #define MKD 5 // derivative gain
// #define MBOUND 2//2 // error, millimetres

// // for turning:
// #define TKP 100
// #define TKI 0
// #define TKD 0

// #define TIKP 0
// #define TIKI 0
// #define TIKD 0

#define ABOUND 0.01 // error, radians

#define SENSOR_LEFT A0
#define SENSOR_FRONT  A1
#define SENSOR_RIGHT A2

#define ADDR_FRONT 0x56
#define ADDR_LEFT  0x54
#define ADDR_RIGHT 0x55

// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset = */ U8X8_PIN_NONE);

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

mtrn3100::Coord start = {0, 0};
mtrn3100::Coord goal  = {1, 1};

mtrn3100::Maze maze;
mtrn3100::Navigator nav(maze, controller, start, goal, 0);

int original_yaw;
String token;

String seq41 = "flfrffffffrflfrffffs";
char seq42[] = 
// "F178;F178;F182;F182;F178;F178;T90;F178;T-90;F182;T90;F178;T90;F182;T-90;F178;F178;F178;F178;T90;F178;F178;T86;f325;T-37;f118;T-24;f290;T16;f166;T37;f252;T-78;F178;T-90;F178;T90;F182;T-90;F178;F178;F178;T-90;F182;T90;F178;T-90;F178;T90;F178;T-90;F178;F182;T-90;F178;T-90;F182";
"F178;T-90;F178;T-90;F178;T85;f296;T-128;f172;T-18;f290;T18;f209;T41;f216;T-84;F178;T-90;F178;T90;F182;T-90;F178;T-90;F182;";

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up!");

  //////////////////// OLED setup ////////////////////
  // u8x8.begin();
  // u8x8.setPowerSave(0);
  // u8x8.clear();
  // u8x8.setFont(u8x8_font_chroma48medium8_r);
  // u8x8.drawString(0, 0, "BallerinaArduina");

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
}

void loop() {
  //////////////////// TASK 4.1 ////////////////////
  // char sequence_1[] = "ffrr";
  // controller.sequence_move(sequence_1);

  //////////////////// TASK 4.2 ////////////////////
  // char sequence_2 = [];
  // controller.execute_sequence(sequence_2);

  //////////////////// TASK 4.3 ////////////////////
  autonomous_map();
  delay(100000);
}


void autonomous_map() {
  // mapping::commitCellAndRedraw(u8x8, maze, frontLidar, leftLidar, rightLidar, 0, 0, 0);

  // Step 1: Explore maze until reaching goal
  // nav.explore();
  explore();

  // // Step 2: Compute shortest path
  // mtrn3100::Coord path[MAZE_ROWS * MAZE_COLS];
  // int pathLen = 0;
  // nav.shortestPath(start, goal, path, pathLen);

  // // Step 3: Go back to start
  // mtrn3100::Coord backPath[MAZE_ROWS * MAZE_COLS];
  // int backLen = 0;
  // nav.shortestPath(nav.pos, start, backPath, backLen);
  // nav.followPath(backPath, backLen);

  // // Step 4: Follow shortest path to goal
  // nav.followPath(path, pathLen);

  Serial.println("Navigation complete.");
}

void explore() {
  // Simple DFS exploration until reaching goal
  dfs(nav.pos);
}

void dfs(mtrn3100::Coord c) {
  Serial.print(c.y);
  Serial.print(" ");
  Serial.println(c.x);

  maze.grid[c.y][c.x].visited = true;
  if (c == goal) {
    delay(100000000);
  }

  // Check walls using lidar
  nav.updateWalls();
  for (int d = 0; d < 4; d++) {
    mtrn3100::Coord next = maze.neighbor(c, (direction)d);
    if (maze.inBounds(next) &&
      !maze.grid[next.y][next.x].visited &&
      !maze.grid[c.y][c.x].walls[d]) 
    {
      controller.move_direction((direction)d);
        // mtrn3100::drawMaze(
        //   u8x8,
        //   maze,
        //   c.y,      
        //   c.x,     
        //   (direction)d,
        //   mtrn3100::countVisitedCells(maze),
        //   MAZE_ROWS * MAZE_COLS - NUM_CORNER_CELLS,
        //   mtrn3100::percentVisited(maze)
        // );
      nav.heading = (direction)d;
      nav.pos = next;

      dfs(next);

      // Backtrack
      controller.move_direction((direction)((d + 2) % 4));
      nav.heading = (direction)d;
      nav.pos = c;
      controller.move_direction((direction)d); // restore orientation
      nav.heading = (direction)d;
    }
  }
}

void explore_stack() {
  mtrn3100::Coord stack[MAZE_ROWS * MAZE_COLS];
  int top = 0;
  stack[top++] = nav.pos;

  while (top > 0) {
    mtrn3100::Coord c = stack[--top];
    maze.grid[c.y][c.x].visited = true;

    if (c == goal) return;

    nav.updateWalls();

    for (int d = 0; d < 4; d++) {
      mtrn3100::Coord next = maze.neighbor(c, (direction)d);
      if (maze.inBounds(next) &&
          !maze.grid[next.y][next.x].visited &&
          !maze.grid[c.y][c.x].walls[d]) {
        
        controller.move_direction((direction)d);
        nav.heading = (direction)d;
        nav.pos = next;

        // mtrn3100::drawMaze(
        //   u8x8, maze,
        //   nav.pos.y, nav.pos.x, nav.heading,
        //   mtrn3100::countVisitedCells(maze),
        //   MAZE_ROWS * MAZE_COLS - NUM_CORNER_CELLS,
        //   mtrn3100::percentVisited(maze)
        // );

        stack[top++] = next;
      }
    }
  }
}


