#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <stdbool.h>
#include "queue.h"

#define START Location(0, 0)
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

enum WallState {
  EXIT = 0,     // Tường trống (đã xác nhận)
  WALL = 1,     // Có tường (đã xác nhận)
  UNKNOWN = 2,  // Chưa rõ
  VIRTUAL = 3,  // Tường ảo
};

struct WallInfo {
  WallState north : 2;
  WallState east  : 2;
  WallState south : 2;
  WallState west  : 2;
};

enum MazeMask {
  MASK_OPEN = 0x01,    // Dùng khi dò đường
  MASK_CLOSED = 0x03,  // Dùng khi chạy tốc độ
};

enum Heading { NORTH, EAST, SOUTH, WEST, HEADING_COUNT, BLOCKED = 99 };

inline Heading right_from(const Heading heading) {
  return static_cast<Heading>((heading + 1) % HEADING_COUNT);
}

inline Heading left_from(const Heading heading) {
  return static_cast<Heading>((heading + HEADING_COUNT - 1) % HEADING_COUNT);
}

inline Heading ahead_from(const Heading heading) {
  return heading;
}

inline Heading behind_from(const Heading heading) {
  return static_cast<Heading>((heading + 2) % HEADING_COUNT);
}

enum Direction { AHEAD, RIGHT, BACK, LEFT, DIRECTION_COUNT };

class Location {
 public:
  uint8_t x;
  uint8_t y;

  Location() : x(0), y(0) {}
  Location(uint8_t ix, uint8_t iy) : x(ix), y(iy) {}

  bool is_in_maze() const { return x < MAZE_WIDTH && y < MAZE_HEIGHT; }
  bool operator==(const Location &obj) const { return x == obj.x && y == obj.y; }
  bool operator!=(const Location &obj) const { return x != obj.x || y != obj.y; }

  Location north() const { return Location(x, (y + 1) % MAZE_HEIGHT); }
  Location east() const { return Location((x + 1) % MAZE_WIDTH, y); }
  Location south() const { return Location(x, (y + MAZE_HEIGHT - 1) % MAZE_HEIGHT); }
  Location west() const { return Location((x + MAZE_WIDTH - 1) % MAZE_WIDTH, y); }

  Location neighbour(const Heading heading) const {
    switch (heading) {
      case NORTH: return north();
      case EAST:  return east();
      case SOUTH: return south();
      case WEST:  return west();
      default:    return *this;
    }
  }
};

class Maze {
 public:
  Maze();

  bool is_goal(const Location cell) const;
  WallInfo walls(const Location cell) const;
  bool has_unknown_walls(const Location cell) const;
  int wall_count(const Location cell) const;
  bool cell_is_visited(const Location cell) const;
  bool is_exit(const Location cell, const Heading heading) const;

  void update_wall_state(const Location cell, const Heading heading, const WallState state);
  void initialise();
  void set_mask(const MazeMask mask);
  MazeMask get_mask() const;

  uint16_t neighbour_cost(const Location cell, const Heading heading) const;
  uint16_t cost(const Location cell) const;

  void flood_to_center();
  void flood(const Location target);
  Heading heading_to_smallest(const Location cell, const Heading start_heading) const;
  bool flood_queue_overflow() const;

 private:
  void set_wall_state(const Location loc, const Heading heading, const WallState state);

  MazeMask m_mask;
  uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT];
  WallInfo m_walls[MAZE_WIDTH][MAZE_HEIGHT];
  bool m_flood_queue_overflow;
};

#endif  // MAZE_H