#pragma once
#include <Arduino.h>
#include <GenericQueue.h>

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
    // Create a cell with unknown walls and infinite flood fill distance
    maze_cell(int r, int c) : row(r), col(c), visited(true), ff_dist(FLOOD_FILL_INF) {
      for (int i = 0; i < 4; i++) {
        wall_dir[i] = wall_unknown;
      }
    }: 

    // ---- Getters / Setters ----
    int getRow() const { return row; }
    int getCol() const { return col; }

    bool isVisited() const { return visited; }
    void setVisited(bool v) { visited = v; }

    // Wall access
    int getWall(int dir) const { return wall_dir[dir]; }
    void setWall(int dir, int state) { wall_dir[dir] = state; }

    // Utility
    bool isWall(int dir) const { return wall_dir[dir] == wall_present; }
    bool isOpen(int dir) const { return wall_dir[dir] == wall_open; }
  
    private:
    // Directions and wall states
    enum direction: int {north = 0, east = 1, south = 2, west = 3};
    enum wall_state: int {wall_unknown = -1, wall_open = 0, wall_present = 1};

    int row;         
    int col;          
    bool visited;     
    int wall_dir[4]; // N,E,S,W => -1 unknown, 0 open, 1 wall
    int ff_dist;
}

class maze_map {
public:
  // Setup the maze full of cells
  maze_map() { 
    for (int r = 0; r < MAZE_ROWS; r++) {
      for (int c = 0; c < MAZE_COLS; c++) {
        grid[r][c] = nullptr; // initially empty
      }
    }
    visited_count = 0;
  }

  void add_node(int r, int c) {
    if (in_bounds(r, c)) {
      if (grid[r][c] == nullptr) {
        grid[r][c] = new maze_cell(r, c);  // dynamic allocation
      } else {
        Serial.println("Node already exists at this position.");
      }
    }
  }

  // Add a wall between two cells
  void add_edge(int r1, int c1, int r2, int c2) {
    if (in_bounds(r1, c1) && in_bounds(r2, c2)) {
      int dr = r2 - r1;
      int dc = c2 - c1;

      int dir = -1;
      if (dr == -1 && dc == 0) dir = 0; // north
      else if (dr == 0 && dc == +1) dir = 1; // east
      else if (dr == +1 && dc == 0) dir = 2; // south
      else if (dr == 0 && dc == -1) dir = 3; // west
      else return; // invalid edge

      grid[r1][c1].wall_dir[dir] = wall_open;
      grid[r2][c2].wall_dir[opposite_dir(abs_dir)] = wall_open;
    }
  }

  // - THIS IS THE SAME AS ADD_EDGE, I WILL CHANGE IT TO ADD_EDGE BUT I'M also not sure what its trying to do
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


  // Safe access
  maze_cell* get_node(int r, int c) {
    if (in_bounds(r, c)) {
      return grid[r][c];
    } else {
      Serial.println("Invalid cell access.");
      return nullptr;
    }
  } 

  // BFS traversal
  void bfs(int start_r, int start_c) {
    if (!in_bounds(start_r, start_c)) return;

    GenericQueue<maze_cell*> q;
    grid[start_r][start_c].visited = true;
    q.enqueue(&grid[start_r][start_c]);

    while (!q.isEmpty()) {
      maze_cell* cur;
      q.dequeue(cur);

      GenericQueue<maze_cell*> neighbors;
      get_neighbors(cur->row, cur->col, neighbors);
      while (!neighbors.isEmpty()) {
        maze_cell* n;
        neighbors.dequeue(n);
        if (!n->visited) {
          n->visited = true;
          q.enqueue(n);                
        }
      }
    }
  }

  //////////////////////////////////////////////////

  // USE CELL FUNCTIONS FOR ACCESSING CELLS

  // // read-only queries for drawing
  // bool is_visited(int r, int c) const {
  //   return (r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS) ? grid[r][c].visited : false;
  // }
  
  // int wall_at(int r, int c, int dir) const {
  //   if (r < 0 || r >= MAZE_ROWS || c < 0 || c >= MAZE_COLS) return wall_unknown;
  //   return grid[r][c].wall_dir[dir];
  // }

  //////////////////////////////////////////////////

  int percent() const {
    return (visited_count * 100) / ((MAZE_ROWS * MAZE_COLS) - NUM_CORNER_CELLS);
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

  // Flood fill to create a graph to map the maze to the goal while tracking where it is
  void autonom_map(int start_x, int start_y, int goal_x, int goal_y, int heading) {
    int row = start_x;
    int col = start_y;

    // Add start position to the graph
    maze[row][col] = new map_node(row, col);

    while (row!= goal_x || col != goal_y) {
      maze[row][col] = new map_node(row, col, false);
      queue.enqueue(maze[row][col]);
    }

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
  maze_cell* grid[MAZE_ROWS][MAZE_COLS]; // pointers, initially empty
  int visited_count = 0;
};

} 