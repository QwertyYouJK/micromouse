#pragma once
#include <U8x8lib.h>
#include "Task_4_autonom.hpp" 

namespace mtrn3100 {

inline int countVisitedCells(const Maze& maze) {
  int v = 0;
  for (int r = 0; r < MAZE_ROWS; ++r)
    for (int c = 0; c < MAZE_COLS; ++c)
      if (maze.grid[r][c].visited) ++v;
  return v;
}

inline int percentVisited(const Maze& maze) {
  const int total = MAZE_ROWS * MAZE_COLS;
  if (total == 0) return 0;
  return (100 * countVisitedCells(maze)) / total;
}

// Safe cell pointer (returns nullptr if OOB)
inline const Cell* getCellConst(const Maze& maze, int row, int col) {
  // Maze::inBounds expects Coord{x=col, y=row}
  if (maze.inBounds({col, row})) return &maze.grid[row][col];
  return nullptr;
}
inline Cell* getCell(Maze& maze, int row, int col) {
  if (maze.inBounds({col, row})) return &maze.grid[row][col];
  return nullptr;
}

// ---------- Build an 8x8 tile for one cell ----------
static inline void makeCellTile(
  const Cell* cell,
  bool isRobot, int heading,
  uint8_t out[8]
) {
  for (int i = 0; i < 8; ++i) out[i] = 0x00;

  if (cell) {
    // Walls: solid edges (N=0, E=1, S=2, W=3)
    if (cell->walls[0]) out[0] = 0xFF;     // North
    if (cell->walls[2]) out[7] = 0xFF;     // South
    if (cell->walls[3]) {                  // West (leftmost column of pixels)
      for (int y = 0; y < 8; ++y) out[y] |= 0x01;
    }
    if (cell->walls[1]) {                  // East (rightmost column of pixels)
      for (int y = 0; y < 8; ++y) out[y] |= 0x80;
    }

    // Visited indicator: center block
    if (cell->visited) {
      for (int y = 2; y <= 5; ++y) out[y] |= 0b00111100;
    }
  }

  // Robot marker + heading
  if (isRobot) {
    out[3] |= 0b00011000;
    out[4] |= 0b00011000;
    switch (heading & 0x3) {
      case 0:  out[1] |= 0b00011000; break;                                  // North
      case 1:  out[3] |= 0b00100000; out[4] |= 0b00100000; break;            // East
      case 2:  out[6] |= 0b00011000; break;                                  // South
      case 3:  out[3] |= 0b00000100; out[4] |= 0b00000100; break;            // West
    }
  }
}

// ---------- Main draw function ----------
inline void drawMaze(
  U8X8 &u8x8,
  Maze &maze,
  // IMPORTANT: pass robot_r = row (y), robot_c = col (x)
  int robot_r, int robot_c, int heading,
  // You can pass precomputed stats or -1 to compute here
  int visited_count = -1, int total_cells = -1, int percent = -1
) {
  if (visited_count < 0) visited_count = countVisitedCells(maze);
  if (total_cells   < 0) total_cells   = MAZE_ROWS * MAZE_COLS;
  if (percent       < 0) percent       = (total_cells > 0) ? (100 * visited_count) / total_cells : 0;

  // 8x8 window centered on robot when possible
  int r0 = robot_r - 4;
  int c0 = robot_c - 4;
  if (r0 < 0) r0 = 0;
  if (c0 < 0) c0 = 0;
  if (r0 > MAZE_ROWS - 8) r0 = MAZE_ROWS - 8;
  if (c0 > MAZE_COLS - 8) c0 = MAZE_COLS - 8;

  u8x8.clear();

  uint8_t tile[8];
  for (int tr = 0; tr < 8; ++tr) {
    for (int tc = 0; tc < 8; ++tc) {
      const int mr = r0 + tr;  // maze row
      const int mc = c0 + tc;  // maze col
      const Cell* cell = getCell(maze, mr, mc);
      const bool isRobot = (mr == robot_r && mc == robot_c);
      makeCellTile(cell, isRobot, heading, tile);
      u8x8.drawTile(tc, tr, 1, tile);  // (xTile, yTile, count, data)
    }
  }

  // Right text panel
  u8x8.setFont(u8x8_font_5x7_f);
  char buf[16];

  u8x8.drawString(9, 0, "Visited:");
  snprintf(buf, sizeof(buf), "%d/%d", visited_count, total_cells);
  u8x8.drawString(9, 1, buf);

  u8x8.drawString(9, 3, "Done:");
  snprintf(buf, sizeof(buf), "%d%%", percent);
  u8x8.drawString(9, 4, buf);
}

} // namespace mtrn3100