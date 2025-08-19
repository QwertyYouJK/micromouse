#pragma once
#include <Arduino.h>

#define MAZE_ROWS 9
#define MAZE_COLS 9
#define CELL_SIZE 180        // mm per cell
#define WALL_THRESHOLD 120   // mm: < threshold = wall, >= threshold = open

namespace mtrn3100 {

// Directions and wall states
enum direction:int {north=0, east=1, south=2, west=3};
enum wall_state:int {wall_unknown=-1, wall_open=0, wall_present=1};

// One grid cell
struct cell {
  int8_t wall_dir[4];  // N,E,S,W => -1 unknown, 0 open, 1 wall
  uint8_t visited;
};

// offsets for N,E,S,W (row increases downward, col increases right)
static const int row_step[4] = {-1, 0, +1, 0};
static const int col_step[4] = {0, +1,  0, -1};

class maze_map {
public:
  maze_map() { 
    setup(); 
  }

  // read-only queries for drawing
  bool is_visited(int r, int c) const {
    return (r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS) ? grid[r][c].visited : false;
  }
  int wall_at(int r, int c, int dir) const {
    if (r < 0 || r >= MAZE_ROWS || c < 0 || c >= MAZE_COLS) return wall_unknown;
    return grid[r][c].wall_dir[dir];
  }

  int percent() const {
  return (visited_count * 100) / (MAZE_ROWS * MAZE_COLS);
}

  // all cells ->  unknown + not visited
  void setup() {
    for (int r = 0; r < MAZE_ROWS; r++) {
      for (int c = 0; c < MAZE_COLS; c++) {
        grid[r][c].visited = false;
        grid[r][c].wall_dir[north] = wall_unknown;
        grid[r][c].wall_dir[east]  = wall_unknown;
        grid[r][c].wall_dir[south] = wall_unknown;
        grid[r][c].wall_dir[west]  = wall_unknown;
      }
    }
    visited_count = 0;
  }

  // Mark current cell visited and set walls from LiDAR (front/left/right).
  // Call ONCE when the robot is centred in a cell.
  void update(int row, int col, int heading, int front_mm, int left_mm, int right_mm) {
    if (!in_bounds(row, col)) 
    return;

    if (!grid[row][col].visited) {
      grid[row][col].visited = true;
      visited_count++;
    }

    // Convert robot-relative to absolute directions, then set/mirror walls
    set_wall_from_distance(row, col, rel_to_abs(heading, 0), front_mm); // front
    set_wall_from_distance(row, col, rel_to_abs(heading, 3), left_mm);  // left
    set_wall_from_distance(row, col, rel_to_abs(heading, 1), right_mm); // right
  }

private:
  static bool in_bounds(int r, int c) { 
    return (r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS); 
  }

  static int  opposite_dir(int d) { 
    return (d + 2) & 0x3; 
  }
  static int  rel_to_abs(int h, int rel){ 
    return (h + rel) & 0x3; 
  } // rel: 0=front,1=right,2=back,3=left

  // Set one side and mirror into the neighbour
  void set_wall_from_distance(int row, int col, int abs_dir, int dist_mm) {
    if (dist_mm == 0) return; // no reading
    
    int v;
    if (dist_mm < WALL_THRESHOLD) {
      v = wall_present;
    } else {
      v = wall_open;
    }

    grid[row][col].wall_dir[abs_dir] = v;

    int nr = row + row_step[abs_dir];
    int nc = col + col_step[abs_dir];
    if (in_bounds(nr, nc)) {
      grid[nr][nc].wall_dir[opposite_dir(abs_dir)] = v;
    }
  }

private:
  cell grid[MAZE_ROWS][MAZE_COLS];
  int visited_count = 0;
};

} 