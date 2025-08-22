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
// static const int row_step[4] = {-1, 0, +1, 0};
// static const int col_step[4] = {0, +1,  0, -1};

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
  enum direction: int { north = 0, east = 1, south = 2, west = 3 };
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

  // Returns existing cell pointer or nullptr
  maze_cell* get_cell(int r, int c) const {
    if (!in_bounds(r, c)) return nullptr;
    return grid[r][c];
  }

  maze_cell* get_node(int r, int c) {
    if (in_bounds(r, c)) return grid[r][c];
    Serial.println("Invalid cell access.");
    return nullptr;
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
    // Only flood over KNOWN/ALLOCATED cells.
    maze_cell* goal = get_cell(goal_r, goal_c);
    if (!goal) {
      // Goal is not on the known map yet; nothing to do.
      return;
    }

    // Reset distances on existing cells only
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
        if (!cur->isOpen(dir)) continue;              // only traverse known-open edges

        const int nr = r + row_step[dir];
        const int nc = c + col_step[dir];
        if (!in_bounds(nr, nc)) continue;

        // READ-ONLY: do not allocate unknown neighbours here
        maze_cell* n = get_cell(nr, nc);
        if (!n) continue;

        if (n->getFFDist() > d + 1) {
          n->setFFDist(d + 1);
          q.enqueue(n);
        }
      }
    }
  }


  // int choose_best_dir(int r, int c, int goal_r, int goal_c) {
  //   maze_cell* cur = get_cell(r, c);
  //   if (!cur) return -1;

  //   // Current Euclidean distance (in cell units)
  //   const float cur_d = hypotf((float)(goal_r - r), (float)(goal_c - c));

  //   int best_dir = -1;
  //   float best_d = cur_d - 1e-6f;  // require strictly closer than current

  //   for (int dir = 0; dir < 4; ++dir) {
  //     if (!cur->isOpen(dir)) continue;

  //     const int nr = r + row_step[dir];
  //     const int nc = c + col_step[dir];
  //     if (!in_bounds(nr, nc)) continue;

  //     const float d = hypotf((float)(goal_r - nr), (float)(goal_c - nc));

  //     // Only accept neighbours that are strictly closer than the current cell
  //     if (d < best_d) {
  //       best_d  = d;
  //       best_dir = dir;
  //     }
  //   }

  //   return best_dir;  // -1 means "no closer step" → caller should stop
  // }

  int choose_best_dir(int r, int c, int goal_r, int goal_c) {
    maze_cell* cur = get_cell(r, c);
    if (!cur) return -1;

    // Current Euclidean distance (in cell units)
    const float cur_d = hypotf((float)(goal_r - r), (float)(goal_c - c));

    int   best_dir = -1;
    float best_d   = cur_d - 1e-6f;  // require strictly closer than current

    for (int dir = 0; dir < 4; ++dir) {
      // 1) You cannot go where there's a wall (or unknown): only through known-open edges
      if (!cur->isOpen(dir)) continue;

      const int nr = r + row_step[dir];
      const int nc = c + col_step[dir];
      if (!in_bounds(nr, nc)) continue;

      // 2) If neighbour already exists and says there's a wall on the reciprocal side, skip
      if (maze_cell* n = get_cell(nr, nc)) {
        if (n->isWall(opposite_dir(dir))) continue;
        // (If n is unknown/null, we'll allow the step based on current cell's open edge.
        //  The neighbour will be created/mirrored when you update after moving.)
      }

      // 3) Greedy: choose the neighbour with strictly smaller Euclidean distance to goal
      const float d = hypotf((float)(goal_r - nr), (float)(goal_c - nc));
      if (d < best_d) {
        best_d  = d;
        best_dir = dir;
      }
    }

    return best_dir;  // -1 => no legal step that gets you closer
  }

  // Mark current cell visited and set walls from LiDAR (front/left/right).
  // Call ONCE when the robot is centred in a cell.
  void update(int row, int col, int heading, int front_mm, int left_mm, int right_mm) {
    maze_cell* cell = ensure_cell(row, col);

    if (!cell->isVisited()) {
      cell->setVisited(true);
      add_visited_count();
    }

    const int abs_front = rel_to_abs(heading, 0);
    const int abs_right = rel_to_abs(heading, 1);
    const int abs_left  = rel_to_abs(heading, 3);   // <-- was 2; fix to 3 (left)

    // Write walls for current cell
    cell->setWallState(abs_front, front_mm);
    cell->setWallState(abs_right, right_mm);
    cell->setWallState(abs_left,  left_mm);

    // Mirror to neighbours
    // mirror_to_neighbor(row, col, abs_front, front_mm);
    // mirror_to_neighbor(row, col, abs_right, right_mm);
    // mirror_to_neighbor(row, col, abs_left,  left_mm);
  }

  // Add this inside class maze_map (public or private as you prefer)
  void mirror_to_neighbor(int row, int col, int abs_dir, int dist_mm) {
    // Compute neighbour coordinates in the absolute direction
    const int nr = row + row_step[abs_dir];
    const int nc = col + col_step[abs_dir];
    if (!in_bounds(nr, nc)) return;

    if (dist_mm >= WALL_THRESHOLD) {
      // Edge is OPEN → ensure neighbour exists and mark its opposite edge OPEN
      maze_cell* n = ensure_cell(nr, nc);
      n->setWallState(opposite_dir(abs_dir), WALL_THRESHOLD + 1);
    } else {
      // Edge is a WALL → if neighbour exists, mark its opposite edge as WALL
      maze_cell* n = get_cell(nr, nc);
      if (n) {
        n->setWallState(opposite_dir(abs_dir), 0); // any value < WALL_THRESHOLD
      }
    }
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
  // Ensures a cell exists; allocates and constructs if needed
  maze_cell* ensure_cell(int r, int c) {
    if (!in_bounds(r, c)) return nullptr;
    if (!grid[r][c]) {
      grid[r][c] = new maze_cell(r, c, FLOOD_FILL_INF);
    }
    return grid[r][c];
  }

  maze_cell* grid[MAZE_ROWS][MAZE_COLS]; // lazily allocated cells (nullptr until visited)
  int visited_count;
};

}  // namespace mtrn3100

// // Flood fill to create a graph to map the maze to the goal while tracking where it is
// void autonom_map(int start_r, int start_c, int goal_r, int goal_c, int heading) {
//   int row = start_r;
//   int col = start_c;

//   while (!(row == goal_r && col == goal_c)) {

//     // Update map with sensors for the *current cell*
//     maze.update(row, col, heading,
//                 frontLidar.readMillimetres(),
//                 leftLidar.readMillimetres(),
//                 rightLidar.readMillimetres());

//     // // Optional if you still keep flood fill for other logic
//     // maze.flood_fill(goal_r, goal_c);

//     // Choose a step that is strictly closer than the current cell
//     int dir = maze.choose_best_dir(row, col, goal_r, goal_c);
//     if (dir == -1) {
//       // No strictly-closer neighbour → either at goal or boxed in
//       break;
//     }

//     Serial.print("Moving from: ");
//     Serial.print(row);
//     Serial.print(" ");
//     Serial.print(col);
//     Serial.print("Heading ");
//     Serial.println(heading);

//     // Command the move (ideally blocking until cell center reached)
//     controller.move_direction(dir);

//     // Update discrete pose
//     row += row_step[dir];
//     col += col_step[dir];
//     heading = dir;  // absolute headings must match N,E,S,W = 0..3
//   }

//   Serial.println("Reached goal!");
//   leftMotor.setPWM(MOTOFF);
//   rightMotor.setPWM(MOTOFF);
//   delay(10000);
// }
