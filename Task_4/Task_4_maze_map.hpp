#pragma once
#include <Arduino.h>
#include <GenericQueue.h>

#define MAZE_ROWS 9
#define MAZE_COLS 9
#define CELL_SIZE 180        // mm per cell
#define WALL_THRESHOLD 120   // mm: < threshold = wall, >= threshold = open
#define NUM_CORNER_CELLS 12
#define FLOOD_FILL_INF 255

/////// Add controller so that we can move the robot and use the lidars ///////


// offsets for N,E,S,W (row increases downward, col increases right)
static const int row_step[4] = {-1, 0, +1, 0};
static const int col_step[4] = {0, +1,  0, -1};

namespace mtrn3100 {

class maze_cell {
  public:
    // Create a cell with unknown walls and unvisited
    maze_cell(int r, int c, int ff_dist) : row(r), col(c), ff_dist(ff_dist), visited(false) {
      for (int i = 0; i < 4; i++) {
        wall_dir[i] = wall_unknown;
      }
    }: 

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
    int getFloodFillDistance() const { return ff_dist; }
    void setFloodFillDistance(int dist) { ff_dist = dist; }
    
    // Mark current cell visited and set walls from LiDAR (front/left/right).
    // Call ONCE when the robot is centred in a cell.
    void update(int heading, int front_mm, int left_mm, int right_mm) {
      if (!visited()) {
        setVisited(true);
        visited_count++;
      }

      // Convert robot-relative to absolute directions, then set walls
      setWall(rel_to_abs(heading, direction.forward), front_mm); 
      setWall(rel_to_abs(heading, direction.right), right_mm); 
      setWall(rel_to_abs(heading, direction.left), left_mm);  
    }

    private:
    // Directions and wall states
    enum direction: int {forward = 0, right = 1, back = 2, left = 3};
    enum wall_state: int {wall_unknown = -1, wall_open = 0, wall_present = 1};

    int row;         
    int col;          
    bool visited;     
    int wall_dir[4]; // N,E,S,W => -1 unknown, 0 open, 1 wall
    int ff_dist;
}

class maze_map {
public:
  // Setup the maze full of empty cells - infinite distance and unvisited
  maze_map() { 
    for (int r = 0; r < MAZE_ROWS; r++) {
      for (int c = 0; c < MAZE_COLS; c++) {
        maze_cell(r, c, FLOOD_FILL_INF);
      }
    }
    visited_count = 0;
  }

  int percent() const {
    return (visited_count * 100) / ((MAZE_ROWS * MAZE_COLS) - NUM_CORNER_CELLS);
  }

  ///////////////////// Flood Fill Algorithm /////////////////////
  void flood_fill(int goal_r, int goal_c) {
    // Init goal
    GenericQueue<maze_cell*> q;
    maze_cell& goal = grid[goal_r][goal_c];
    goal.setFloodFill(0);
    q.enqueue(&goal);

    while (!q.isEmpty()) {
      maze_cell* cur;
      q.dequeue(cur);

      int r = cur->getRow();
      int c = cur->getCol();
      int d = cur->getFloodFill();

      for (int dir = 0; dir < 4; dir++) {
        int nr = r + row_step[dir];
        int nc = c + col_step[dir];

        if (in_bounds(nr, nc) && cur->isOpen(dir)) {
          maze_cell& n = grid[nr][nc];
          if (n.getFloodFill() > d + 1) {
            n.setFloodFill(d + 1);
            q.enqueue(&n);
          }
        }
      }
    }
  }

  // Decide best move for the robot from (r,c)
  int choose_best_dir(int r, int c) {
    maze_cell& cur = grid[r][c];
    int best_dir = -1;
    int best_dist = FLOOD_FILL_INF;

    for (int dir = 0; dir < 4; dir++) {
      if (cur.isOpen(dir)) {
        int nr = r + row_step[dir];
        int nc = c + col_step[dir];
        if (in_bounds(nr, nc)) {
          int nd = grid[nr][nc].getFloodFill();
            if (nd < best_dist) {
              best_dist = nd;
              best_dir = dir;
            }
          }
        }
      }
      return best_dir;  // returns N/E/S/W direction to move
    }

  // Flood fill to create a graph to map the maze to the goal while tracking where it is
  void autonom_map(int start_r, int start_c, int goal_r, int goal_c, int heading) {
    int row = start_r;
    int col = start_c;

    while (!(row == goal_r && col == goal_c)) {
        // 1. Update current cell with LiDAR readings
        // (front_mm, left_mm, right_mm come from sensors each step)
        update(row, col, heading, front_mm, left_mm, right_mm);

        // 2. Run flood fill from goal
        flood_fill(goal_r, goal_c);

        // 3. Pick best direction to move
        int dir = choose_best_dir(row, col);
        if (dir == -1) {
            Serial.println("Dead end, no path found!");
            return;
        }

        ///////////// Replace with actual controller movement /////////////
        // 4. Move robot in chosen direction
        row += row_step[dir];
        col += col_step[dir];
        heading = dir; // update robot’s heading (simplified)

        Serial.print("Moving to cell: ");
        Serial.print(row);
        Serial.print(", ");
        Serial.println(col);
    }

    Serial.println("Reached goal!");
  }

private:
  static bool in_bounds(int r, int c) { 
    return (r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS); 
  }
  
  static int opposite_dir(int d) { 
    return (d + 2) & 0x3; 
  }
  static int rel_to_abs(int h, int rel){ 
    return (h + rel) & 0x3; 
  } // rel: 0=front,1=right,2=back,3=left

private:
  maze_cell grid[MAZE_ROWS][MAZE_COLS]; // Creates a 2D array of maze cells
  int visited_count = 0;
};

} 