# Usage Notes

## Planning Workflow

1. Place the dock/start point.
2. Place the drop-off/end point.
3. Add optional intermediate waypoints for pick locations or required aisle transitions.
4. Draw warehouse shelving/walls.
5. Paint terrain costs for fast lanes, congested zones, ramps, and concrete.
6. Choose a path-planning algorithm.
7. Click run to compute and animate the route.

## Algorithms

- **A\*:** Balanced route search using cost and heuristic distance.
- **Dijkstra:** Cost-optimal search without a destination heuristic.
- **Greedy Best-First:** Fast heuristic-driven search that may not be cost-optimal.
- **BFS:** Unweighted breadth-first search.

## Terrain

- **Concrete:** normal warehouse floor.
- **Fast Lane:** preferred marked travel corridor with lower cost.
- **Congested:** area with higher cost, representing people, pallets, or traffic.
- **Ramp:** medium-cost incline or transition zone.

## Outputs

The simulator can save/load floor maps as JSON and export route instructions to text for documentation or implementation planning.
