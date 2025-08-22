#pragma once
#include <U8x8lib.h>
#include "Task_4_autonom.hpp"

// Build a single 8x8 tile for one cell based on walls/visited/robot.
// NOTE: If left/right walls show mirrored, swap 0x01 and 0x80 below (bit order differs on some modules).
static inline void makeCellTile(
  const mtrn3100::maze_cell* cell,
  bool isRobot, int heading,
  uint8_t out[8]
) {
  for (int i = 0; i < 8; ++i) out[i] = 0x00;

  if (cell) {
    // Walls: draw solid edges
    if (cell->isWall(0)) out[0] = 0xFF;     // North
    if (cell->isWall(2)) out[7] = 0xFF;     // South
    if (cell->isWall(3)) {                  // West (left column)
      for (int y = 0; y < 8; ++y) out[y] |= 0x01;   // if mirrored, change to 0x80
    }
    if (cell->isWall(1)) {                  // East (right column)
      for (int y = 0; y < 8; ++y) out[y] |= 0x80;   // if mirrored, change to 0x01
    }

    // Visited: fill a small square inside
    if (cell->isVisited()) {
      for (int y = 2; y <= 5; ++y) out[y] |= 0b00111100;  // center block
    }
  }

  // Robot marker + heading tick (overlays on top)
  if (isRobot) {
    // center pixel (approx): (x=3..4, y=3..4)
    out[3] |= 0b00011000;
    out[4] |= 0b00011000;
    // heading tick
    if (heading == 0) out[1] |= 0b00011000;       // North
    else if (heading == 1) { out[3] |= 0b00100000; out[4] |= 0b00100000; } // East
    else if (heading == 2) out[6] |= 0b00011000;  // South
    else if (heading == 3) { out[3] |= 0b00000100; out[4] |= 0b00000100; } // West
  }
}

// Draw an 8x8-tile window around the robot + text panel on the right.
// Uses only U8x8 tile/text API (tiny RAM).
inline void drawMaze(
  U8X8 &u8x8,
  mtrn3100::maze_map &m,
  int robot_r, int robot_c, int heading,
  int visited_count, int total_cells, int percent
) {
  // Compute 8x8 window origin, centered on robot when possible
  int r0 = robot_r - 4;
  int c0 = robot_c - 4;
  if (r0 < 0) r0 = 0;
  if (c0 < 0) c0 = 0;
  if (r0 > MAZE_ROWS - 8) r0 = MAZE_ROWS - 8;
  if (c0 > MAZE_COLS - 8) c0 = MAZE_COLS - 8;

  // Clear whole screen quickly (tile mode)
  u8x8.clear();

  // Draw 8x8 tiles for the window in tile columns [0..7], rows [0..7]
  uint8_t tile[8];
  for (int tr = 0; tr < 8; ++tr) {
    for (int tc = 0; tc < 8; ++tc) {
      int mr = r0 + tr;
      int mc = c0 + tc;
      mtrn3100::maze_cell* cell = m.get_node(mr, mc);
      bool isRobot = (mr == robot_r && mc == robot_c);
      makeCellTile(cell, isRobot, heading & 0x3, tile);
      // drawTile(xTile, yTile, count, data) — count=1 (one tile)
      u8x8.drawTile(tc, tr, 1, tile);
    }
  }

  // Right-hand text panel (tile columns 9..15 leave a gutter at col 8)
  u8x8.setFont(u8x8_font_5x7_f);   // small, readable
  char buf[16];

  // u8x8.drawString(9, 0, "Visited:");
  // snprintf(buf, sizeof(buf), "%d/%d", visited_count, total_cells);
  // u8x8.drawString(9, 1, buf);

  u8x8.drawString(9, 3, "Done:");
  snprintf(buf, sizeof(buf), "%d%%", percent);
  u8x8.drawString(9, 4, buf);

  // Show window origin for debugging (optional)
  // snprintf(buf, sizeof(buf), "r0:%d c0:%d", r0, c0);
  // u8x8.drawString(9, 6, buf);
}


