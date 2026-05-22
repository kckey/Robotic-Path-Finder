# Robotic Path Finder

Industrial warehouse robotic path-planning simulator for autonomous mobile robot route planning.

This project models a warehouse environment with shelving, terrain costs, waypoints, and selectable path-planning algorithms. It is intended as a visual proof of concept for robotic navigation through warehouse aisles, docks, fast lanes, congested areas, and ramps.

## Features

- Path-planning algorithms: A*, Dijkstra, Greedy Best-First, and BFS
- 4-connected or 8-connected movement
- Warehouse wall/shelving placement
- Terrain painting for concrete, fast lanes, congested zones, and ramps
- Multiple waypoints for dock-to-pick-to-drop-off routing
- Step-by-step exploration visualization
- Smooth robot animation along the computed route
- Route metrics: nodes explored, path cost, solve time, and turn count
- Procedural warehouse floor generation
- Save/load map JSON files
- Export route instructions as text

## Repository Layout

```text
src/
  warehouse_path_planner.py   Current final simulator
legacy/
  maze_original.py            Earlier version preserved for reference
docs/
  usage.md                    Controls and workflow notes
examples/
  warehouse_example.json      Example saved warehouse map
```

## Run

Install Pygame:

```bash
python3 -m pip install pygame
```

Start the simulator:

```bash
python3 src/warehouse_path_planner.py
```

## Use Case

The simulator represents robotic path planning in a warehouse environment. A user can place storage racks, assign terrain costs, define a start point and destination, add intermediate waypoints, and compare path planning algorithms before translating the concept into a physical AMR or warehouse automation project.
