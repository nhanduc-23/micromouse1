#include "maze.h"

Maze::Maze() : m_mask(MASK_OPEN), m_flood_queue_overflow(false) {}

bool Maze::is_goal(const Location cell) const {
  return (cell.x == 7 || cell.x == 8) && (cell.y == 7 || cell.y == 8);
}

WallInfo Maze::walls(const Location cell) const {
  return m_walls[cell.x][cell.y];
}

bool Maze::has_unknown_walls(const Location cell) const {
  WallInfo walls_here = m_walls[cell.x][cell.y];
  return (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN ||
          walls_here.south == UNKNOWN || walls_here.west == UNKNOWN);
}

int Maze::wall_count(const Location cell) const {
  int count = 4;
  if (is_exit(cell, NORTH)) count -= 1;
  if (is_exit(cell, EAST))  count -= 1;
  if (is_exit(cell, SOUTH)) count -= 1;
  if (is_exit(cell, WEST))  count -= 1;
  return count;
}

bool Maze::cell_is_visited(const Location cell) const {
  return !has_unknown_walls(cell);
}

bool Maze::is_exit(const Location cell, const Heading heading) const {
  bool result = false;
  WallInfo walls_here = m_walls[cell.x][cell.y];
  switch (heading) {
    case NORTH: result = (walls_here.north & m_mask) == EXIT; break;
    case EAST:  result = (walls_here.east & m_mask) == EXIT; break;
    case SOUTH: result = (walls_here.south & m_mask) == EXIT; break;
    case WEST:  result = (walls_here.west & m_mask) == EXIT; break;
    default:    result = false; break;
  }
  return result;
}

void Maze::update_wall_state(const Location cell, const Heading heading, const WallState state) {
  switch (heading) {
    case NORTH:
      if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN) return;
      break;
    case EAST:
      if ((m_walls[cell.x][cell.y].east & UNKNOWN) != UNKNOWN) return;
      break;
    case WEST:
      if ((m_walls[cell.x][cell.y].west & UNKNOWN) != UNKNOWN) return;
      break;
    case SOUTH:
      if ((m_walls[cell.x][cell.y].south & UNKNOWN) != UNKNOWN) return;
      break;
    default:
      break;
  }
  set_wall_state(cell, heading, state);
}

void Maze::initialise() {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      m_walls[x][y].north = UNKNOWN;
      m_walls[x][y].east = UNKNOWN;
      m_walls[x][y].south = UNKNOWN;
      m_walls[x][y].west = UNKNOWN;
    }
  }
  for (int x = 0; x < MAZE_WIDTH; x++) {
    m_walls[x][0].south = WALL;
    m_walls[x][MAZE_HEIGHT - 1].north = WALL;
  }
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    m_walls[0][y].west = WALL;
    m_walls[MAZE_WIDTH - 1][y].east = WALL;
  }

  set_wall_state(START, EAST, WALL);
  set_wall_state(START, NORTH, EXIT);

  set_mask(MASK_OPEN);
  flood_to_center();
}

void Maze::set_mask(const MazeMask mask) {
  m_mask = mask;
}

MazeMask Maze::get_mask() const {
  return m_mask;
}

uint16_t Maze::neighbour_cost(const Location cell, const Heading heading) const {
  if (!is_exit(cell, heading)) {
    return MAX_COST;
  }
  Location next_cell = cell.neighbour(heading);
  return m_cost[next_cell.x][next_cell.y];
}

uint16_t Maze::cost(const Location cell) const {
  return m_cost[cell.x][cell.y];
}

void Maze::flood_to_center() {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      m_cost[x][y] = (uint8_t)MAX_COST;
    }
  }

  m_flood_queue_overflow = false;
  Queue<Location, MAZE_CELL_COUNT / 4> queue;

  Location goals[4] = {Location(7, 7), Location(7, 8), Location(8, 7), Location(8, 8)};
  for (int i = 0; i < 4; i++) {
    m_cost[goals[i].x][goals[i].y] = 0;
    queue.add(goals[i]);
  }

  while (queue.size() > 0) {
    Location here = queue.head();
    uint16_t newCost = m_cost[here.x][here.y] + 1;

    for (int h = NORTH; h < HEADING_COUNT; h++) {
      Heading heading = static_cast<Heading>(h);
      if (is_exit(here, heading)) {
        Location nextCell = here.neighbour(heading);
        if (m_cost[nextCell.x][nextCell.y] > newCost) {
          m_cost[nextCell.x][nextCell.y] = newCost;
          if (!queue.add(nextCell)) {
            m_flood_queue_overflow = true;
          }
        }
      }
    }
  }
}

void Maze::flood(const Location target) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      m_cost[x][y] = (uint8_t)MAX_COST;
    }
  }

  m_flood_queue_overflow = false;
  Queue<Location, MAZE_CELL_COUNT / 4> queue;
  m_cost[target.x][target.y] = 0;
  queue.add(target);

  while (queue.size() > 0) {
    Location here = queue.head();
    uint16_t newCost = m_cost[here.x][here.y] + 1;

    for (int h = NORTH; h < HEADING_COUNT; h++) {
      Heading heading = static_cast<Heading>(h);
      if (is_exit(here, heading)) {
        Location nextCell = here.neighbour(heading);
        if (m_cost[nextCell.x][nextCell.y] > newCost) {
          m_cost[nextCell.x][nextCell.y] = newCost;
          if (!queue.add(nextCell)) {
            m_flood_queue_overflow = true;
          }
        }
      }
    }
  }
}

Heading Maze::heading_to_smallest(const Location cell, const Heading start_heading) const {
  Heading next_heading = start_heading;
  Heading best_heading = BLOCKED;
  uint16_t best_cost = cost(cell);
  uint16_t c;

  c = neighbour_cost(cell, next_heading);
  if (c < best_cost) {
    best_cost = c;
    best_heading = next_heading;
  }
  next_heading = right_from(start_heading);
  c = neighbour_cost(cell, next_heading);
  if (c < best_cost) {
    best_cost = c;
    best_heading = next_heading;
  }
  next_heading = left_from(start_heading);
  c = neighbour_cost(cell, next_heading);
  if (c < best_cost) {
    best_cost = c;
    best_heading = next_heading;
  }
  next_heading = behind_from(start_heading);
  c = neighbour_cost(cell, next_heading);
  if (c < best_cost) {
    best_cost = c;
    best_heading = next_heading;
  }
  if (best_cost == MAX_COST) {
    best_heading = BLOCKED;
  }
  return best_heading;
}

bool Maze::flood_queue_overflow() const {
  return m_flood_queue_overflow;
}

void Maze::set_wall_state(const Location loc, const Heading heading, const WallState state) {
  switch (heading) {
    case NORTH:
      m_walls[loc.x][loc.y].north = state;
      m_walls[loc.north().x][loc.north().y].south = state;
      break;
    case EAST:
      m_walls[loc.x][loc.y].east = state;
      m_walls[loc.east().x][loc.east().y].west = state;
      break;
    case WEST:
      m_walls[loc.x][loc.y].west = state;
      m_walls[loc.west().x][loc.west().y].east = state;
      break;
    case SOUTH:
      m_walls[loc.x][loc.y].south = state;
      m_walls[loc.south().x][loc.south().y].north = state;
      break;
    default:
      break;
  }
}