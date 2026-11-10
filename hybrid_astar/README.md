# Single-Robot Hybrid A*

This directory contains a standalone C++17 planner and a pybind11 Python module.
It uses a static occupancy grid, a convex robot footprint, and forward Hybrid A*
search over position and heading. There are no robot-to-robot constraints, waits,
timestamps, parallel search, or ROS dependencies.

Build and test from the repository root:

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

Import the module from the build directory:

    PYTHONPATH=build python3

Python example:

    from hybrid_astar import HybridAStar, OccupancyMap, OriginalMap, Pose2D

    grid = [[False] * 12 for _ in range(12)]  # grid[y][x], True is occupied
    world = OriginalMap(grid, 0.0, 0.0, 1.0)
    collision_map = OccupancyMap(world, [], 16)  # [] is a point robot
    result = HybridAStar(collision_map).solve(
        Pose2D(1.0, 1.0, 0.0), Pose2D(8.0, 8.0, 0.0)
    )
    if result.success:
        print([(point.pose.x, point.pose.y, point.pose.theta) for point in result.path])

The map origin is the center of cell (0, 0). A nonempty footprint is a convex
polygon in robot-local coordinates, supplied as Point2D vertices. Grid space
outside the map and occupied cells are collision regions. Each motion is checked
against the whole static obstacle map before expansion.

MotionPrimitiveConfig describes displacement per action: forward and lateral
distance plus angular change. To translate a reference agent configuration, use
PlanMaxVelocity * PlanDt, PlanMaxSlipVelocity * PlanDt, and
PlanMaxAngularVelocity * PlanDt for the three maximum steps, with the matching
step counts. The search state itself has no time coordinate.

The goal is reached when position and heading enter the requested map cell and
angle bin. The result cost counts actions, and path[i].motion moves from
waypoint i to waypoint i + 1; the final motion is zero. A planner instance can
be used for repeated independent searches. The search retains one best pose
per discretized state, so very tight passages can require finer spatial or
angular resolution.
