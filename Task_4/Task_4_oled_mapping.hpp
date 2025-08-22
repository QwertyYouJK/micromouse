#pragma once
#include <U8x8lib.h>
#include "Task_4_autonom.hpp"
#include "Task_4_lidar.hpp"
#include "Task_4_OLED.hpp"   // must provide: void drawMaze(U8X8& u8x8, ...)

namespace mapping {

// Count visited (used for OLED stats)
inline int countVisitedCells(mtrn3100::maze_map& maze) {
  int v = 0;
  for (int r = 0; r < MAZE_ROWS; ++r)
    for (int c = 0; c < MAZE_COLS; ++c) {
      auto* cell = maze.get_cell(r,c);
      if (cell && cell->isVisited()) ++v;
    }
  return v;
}

// Advance grid pose by 1 cell in the current heading
inline void advancePoseOneCell(int& robot_r, int& robot_c, int heading) {
  if      (heading == 0) --robot_r;   // N
  else if (heading == 1) ++robot_c;   // E
  else if (heading == 2) ++robot_r;   // S
  else                   --robot_c;   // W

  if (robot_r < 0) robot_r = 0;
  if (robot_r >= MAZE_ROWS) robot_r = MAZE_ROWS - 1;
  if (robot_c < 0) robot_c = 0;
  if (robot_c >= MAZE_COLS) robot_c = MAZE_COLS - 1;
}

// Sense -> update map -> redraw OLED (U8x8)
inline void commitCellAndRedraw(
  U8X8& u8x8,
  mtrn3100::maze_map& maze,
  mtrn3100::Lidar& frontLidar,
  mtrn3100::Lidar& leftLidar,
  mtrn3100::Lidar& rightLidar,
  int robot_r, int robot_c, int heading
) {
  maze.get_node(robot_r, robot_c); // no-op for preallocated map; safe otherwise
  Serial.print("Entering cell: ");
  Serial.print(robot_r);
  Serial.println(robot_c);
  const uint16_t f = frontLidar.readMillimetres();
  const uint16_t l = leftLidar.readMillimetres();
  const uint16_t r = rightLidar.readMillimetres();

  maze.update(robot_r, robot_c, heading, f, l, r);

  const int visited = countVisitedCells(maze);
  const int total   = MAZE_ROWS * MAZE_COLS;
  const int percent = maze.percent();

  drawMaze(u8x8, maze, robot_r, robot_c, heading, visited, total, percent);
}

} // namespace mapping
