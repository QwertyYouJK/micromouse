#pragma once
#include <Arduino.h>
#include "simple_queue.hpp"

#define MAZE_ROWS 9
#define MAZE_COLS 9
#define CELL_SIZE 180        // mm per cell
#define WALL_THRESHOLD 120   // mm: < threshold = wall, >= threshold = open
#define NUM_CORNER_CELLS 12
#define FLOOD_FILL_INF 255

// offsets for N,E,S,W (row increases downward, col increases right)
static const int row_step[4] = {-1, 0, +1, 0};
static const int col_step[4] = {0, +1,  0, -1};

namespace mtrn3100 {

class maze_cell {
public:
  // Default constructor
  maze_cell() : ff_dist(FLOOD_FILL_INF), visited(false) {
    for (int i = 0; i < 4; i++) {
      wall_dir[i] = wall_unknown;
    }
  }

  // Create a cell with unknown walls and unvisited
  maze_cell(int r, int c, int ff_dist)
  : row(r), col(c), ff_dist(ff_dist), visited(false) {
    for (int i = 0; i < 4; i++) {
      wall_dir[i] = wall_unknown;
    }
  }

  int getRow() const { return row; }
  int getCol() const { return col; }

  bool isVisited() const { return visited; }
  void setVisited(bool v) { visited = v; }

  // Wall access
  int getWall(int dir) const { return wall_dir[dir]; }
  bool isWall(int dir) const { return wall_dir[dir] == wall_present; }
  bool isOpen(int dir) const { return wall_dir[dir] == wall_open; }

  void setWallState(int dir, int dist_mm) {
    if (dist_mm < WALL_THRESHOLD) {
      wall_dir[dir] = wall_present; // wall exists
    } else {
      wall_dir[dir] = wall_open; // no wall
    }
  }

  // Flood fill distance
  int getFFDist() const { return ff_dist; }
  void setFFDist(int dist) { ff_dist = dist; }

  // Directions and wall states
  enum direction: int {north = 0, east = 1, south = 2, west = 3};
  enum wall_state: int {wall_unknown = -1, wall_open = 0, wall_present = 1};

private:
  int row;
  int col;
  bool visited;
  int ff_dist;
  int wall_dir[4]; // N,E,S,W => -1 unknown, 0 open, 1 wall
};

class maze_map {
public:
  // All pointers start as nullptr; cells are created lazily on first visit.
  maze_map() : visited_count(0) {
    for (int r = 0; r < MAZE_ROWS; ++r) {
      for (int c = 0; c < MAZE_COLS; ++c) {
        grid[r][c] = nullptr;
      }
    }
  }

  ~maze_map() {
    // clean up any allocated cells
    for (int r = 0; r < MAZE_ROWS; ++r) {
      for (int c = 0; c < MAZE_COLS; ++c) {
        delete grid[r][c];
        grid[r][c] = nullptr;
      }
    }
  }

  int percent() const {
    return (visited_count * 100) / ((MAZE_ROWS * MAZE_COLS) - NUM_CORNER_CELLS);
  }

  void add_visited_count() { visited_count++; }

  ///////////////////// Flood Fill Algorithm /////////////////////
  void flood_fill(int goal_r, int goal_c) {
    // Ensure goal exists so BFS can start
    maze_cell* goal = ensure_cell(goal_r, goal_c);

    // Reset distances on existing cells
    for (int r = 0; r < MAZE_ROWS; ++r) {
      for (int c = 0; c < MAZE_COLS; ++c) {
        if (grid[r][c]) grid[r][c]->setFFDist(FLOOD_FILL_INF);
      }
    }

    SimpleQueue<maze_cell*, MAZE_ROWS*MAZE_COLS> q;
    goal->setFFDist(0);
    q.enqueue(goal);

    while (!q.isEmpty()) {
      maze_cell* cur;
      q.dequeue(cur);

      const int r = cur->getRow();
      const int c = cur->getCol();
      const int d = cur->getFFDist();

      for (int dir = 0; dir < 4; ++dir) {
        const int nr = r + row_step[dir];
        const int nc = c + col_step[dir];

        if (!in_bounds(nr, nc)) continue;
        if (!cur->isOpen(dir))  continue;  // blocked edge

        maze_cell* n = grid[nr][nc];
        if (!n) {
          // Allocate neighbor since we know the edge is open
          n = ensure_cell(nr, nc);
          // For consistency, mark its opposite edge open
          // setWallState() expects a distance; pass a value >= WALL_THRESHOLD
          n->setWallState(opposite_dir(dir), WALL_THRESHOLD + 1);
        }

        if (n->getFFDist() > d + 1) {
          n->setFFDist(d + 1);
          q.enqueue(n);
        }
      }
    }
  }


  // Decide best move for the robot from (r,c)
  int choose_best_dir(int r, int c) {
    maze_cell* cur = get_cell(r, c);
    if (!cur) return -1; // current cell not initialised yet

    int best_dir = -1;
    int best_dist = FLOOD_FILL_INF;

    for (int dir = 0; dir < 4; ++dir) {
      if (cur->isOpen(dir)) {
        const int nr = r + row_step[dir];
        const int nc = c + col_step[dir];
        if (in_bounds(nr, nc)) {
          // treat uninitialised neighbor as INF
          int nd = FLOOD_FILL_INF;
          if (grid[nr][nc]) nd = grid[nr][nc]->getFFDist();
          if (nd < best_dist) {
            best_dist = nd;
            best_dir = dir;
          }
        }
      }
    }
    return best_dir;  // returns N/E/S/W direction to move
  }

  // Mark current cell visited and set walls from LiDAR (front/left/right).
  // Call ONCE when the robot is centred in a cell.
  void update(int row, int col, int heading, int front_mm, int left_mm, int right_mm) {
    maze_cell* cell = ensure_cell(row, col); // allocate on first touch
    Serial.print("New cell at: ");
    Serial.print(row);
    Serial.print(", ");
    Serial.println(col);


    if (!cell->isVisited()) {
      cell->setVisited(true);
      add_visited_count();
    }

    // Convert robot-relative to absolute directions, then set walls
    cell->setWallState(rel_to_abs(heading, 0), front_mm);
    cell->setWallState(rel_to_abs(heading, 1), right_mm);
    cell->setWallState(rel_to_abs(heading, 3), left_mm);
  }

  static bool in_bounds(int r, int c) {
    return (r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS);
  }

  static int opposite_dir(int d) {
    return (d + 2) & 0x3;
  }

  static int rel_to_abs(int h, int rel) {
    return (h + rel) & 0x3;
  } // rel: 0=front,1=right,2=back,3=left

private:
  // Returns existing cell pointer or nullptr
  maze_cell* get_cell(int r, int c) const {
    if (!in_bounds(r, c)) return nullptr;
    return grid[r][c];
  }

  // Ensures a cell exists; allocates and constructs if needed
  maze_cell* ensure_cell(int r, int c) {
    if (!in_bounds(r, c)) return nullptr;
    if (!grid[r][c]) {
      grid[r][c] = new maze_cell(r, c, FLOOD_FILL_INF);
      Serial.println("New cell");
    }
    return grid[r][c];
  }

  maze_cell* grid[MAZE_ROWS][MAZE_COLS]; // lazily allocated cells (nullptr until visited)
  int        visited_count;
};

}  // namespace mtrn3100
