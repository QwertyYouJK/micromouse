#include <Arduino.h>
#include "Task_4_controller.hpp"
#include <U8x8lib.h>


#define MAZE_ROWS 9
#define MAZE_COLS 9
#define CELL_SIZE 180        // mm per cell
#define WALL_THRESHOLD 120   // mm: < threshold = wall, >= threshold = open
#define NUM_CORNER_CELLS 12
#define FLOOD_FILL_INF 255

// offsets for N,E,S,W (row increases downward, col increases right)
static const int row_step[4] = {-1, 0, +1, 0};
static const int col_step[4] = {0, +1,  0, -1};
enum direction : int { north = 0, east = 1, south = 2, west = 3 };

namespace mtrn3100 {

// Coordinate struct
struct Coord {
    int x, y;
    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Coord& other) const { // needed for std::map
        return (y == other.y) ? (x < other.x) : (y < other.y);
    }
};

// Maze cell
struct Cell {
    bool visited = false;
    bool walls[4] = {false, false, false, false}; // N, E, S, W
};

// Maze
class Maze {
public:
    Cell grid[MAZE_ROWS][MAZE_COLS];

    bool inBounds(Coord c) {
        return (c.x >= 0 && c.x < MAZE_ROWS && c.y >= 0 && c.y < MAZE_COLS);
    }

    void markWall(Coord c, direction d) {
        if (!inBounds(c)) return;
        grid[c.y][c.x].walls[d] = true;

        // Opposite wall for neighbor
        Coord n = neighbor(c, d);
        if (inBounds(n)) {
            grid[n.y][n.x].walls[(d + 2) % 4] = true;
        }
    }

    Coord neighbor(Coord c, direction d) {
        switch (d) {
            case direction::north: return {c.x, c.y - 1};
            case direction::south: return {c.x, c.y + 1};
            case direction::east:  return {c.x + 1, c.y};
            case direction::west:  return {c.x - 1, c.y};
        }
        return c;
    }
};


class Navigator {
public:
    Maze &maze;
    mtrn3100::Controller &controller;
    Coord start, goal, pos;
    direction heading; 

    Navigator(Maze &m, mtrn3100::Controller &c, Coord s, Coord g, direction heading)
        : maze(m), controller(c), start(s), goal(g), pos(s), heading(heading) {}

    void explore() {
        // Simple DFS exploration until reaching goal
        dfs(pos);
    }

    void dfs(mtrn3100::Coord c) {
        maze.grid[c.y][c.x].visited = true;
        if (c == goal) return;

        Serial.println("Moving");

        // Check walls using lidar
        updateWalls();
        for (int d = 0; d < 4; d++) {
            mtrn3100::Coord next = maze.neighbor(c, (direction)d);
            if (maze.inBounds(next) &&
            !maze.grid[next.y][next.x].visited &&
            !maze.grid[c.y][c.x].walls[d]) 
            {
            controller.move_direction((direction)d);
            Serial.println("Please");
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
            heading = (direction)d;
            pos = next;

            dfs(next);

            // Backtrack
            controller.move_direction((direction)((d + 2) % 4));
            heading = (direction)d;
            pos = c;
            controller.move_direction((direction)d); // restore orientation
            heading = (direction)d;
            }
        }
    }

    void updateWalls() {
        
        uint16_t leftDist  = controller.left_lidar_dist();
        uint16_t rightDist = controller.right_lidar_dist();
        uint16_t frontDist = controller.front_lidar_dist();

        if (frontDist < WALL_THRESHOLD) 
            maze.markWall(pos, heading);
        if (leftDist < WALL_THRESHOLD)
            maze.markWall(pos, (direction)((heading + 3) % 4));
        if (rightDist < WALL_THRESHOLD)
            maze.markWall(pos, (direction)((heading + 1) % 4));
    }

  Coord shortestPath(Coord from, Coord to, Coord* outPath, int &outLen) {
    bool visited[MAZE_ROWS][MAZE_COLS];
    Coord parent[MAZE_ROWS][MAZE_COLS];

    // Reset arrays
    for (int y = 0; y < MAZE_ROWS; y++) {
        for (int x = 0; x < MAZE_COLS; x++) {
            visited[y][x] = false;
            parent[y][x] = {-1, -1};
        }
    }

    // Simple array-based queue
    Coord queue[MAZE_ROWS * MAZE_COLS];
    int qHead = 0, qTail = 0;

    queue[qTail++] = from;
    visited[from.y][from.x] = true;
    parent[from.y][from.x] = from;

    // BFS
    while (qHead < qTail) {
        Coord cur = queue[qHead++];
        if (cur == to) break;

        for (int d = 0; d < 4; d++) {
            if (!maze.grid[cur.y][cur.x].walls[d]) {
                Coord n = maze.neighbor(cur, (direction)d);
                if (maze.inBounds(n) && !visited[n.y][n.x]) {
                    visited[n.y][n.x] = true;
                    parent[n.y][n.x] = cur;
                    queue[qTail++] = n;
                }
            }
        }
    }

    // Reconstruct path into temporary buffer
    Coord temp[MAZE_ROWS * MAZE_COLS];
    int len = 0;
    Coord step = to;

    if (parent[step.y][step.x].x == -1 && parent[step.y][step.x].y == -1) {
        outLen = 0; // no path found
        return {-1, -1};
    }

    while (!(step == from) && len < MAZE_ROWS * MAZE_COLS) {
        temp[len++] = step;
        step = parent[step.y][step.x];
    }
    temp[len++] = from;

    // Reverse into output
    for (int i = 0; i < len; i++) {
        outPath[i] = temp[len - 1 - i];
    }

    outLen = len;
    return to;
  }

    void followPath(Coord* path, int len) {
        for (int i = 1; i < len; i++) {
            Coord next = path[i];
            direction d = directionTo(pos, next);
            controller.move_direction(d);
            heading = d;    
            pos = next;
        }
    }

    direction directionTo(Coord from, Coord to) {
        if (to.y == from.y - 1) return north;
        if (to.y == from.y + 1) return south;
        if (to.x == from.x + 1) return east;
        if (to.x == from.x - 1) return west;
        return heading;
    }
};

} // namespace mtrn3100