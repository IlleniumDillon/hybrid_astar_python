import hybrid_astar as ha


grid = [[False for _ in range(8)] for _ in range(8)]
grid[3][3] = True
original = ha.OriginalMap(grid, 0.0, 0.0, 1.0)
assert original.width == 8
assert original.is_occupied(3, 3)

occupancy = ha.OccupancyMap(original, [], 16)
planner = ha.HybridAStar(occupancy)
result = planner.solve(ha.Pose2D(1.0, 1.0, 0.0), ha.Pose2D(5.0, 1.0, 0.0))
assert result.success
assert result.cost == 4.0
assert len(result.path) == 5
assert result.path[0].motion.forward == 1.0
assert result.path[-1].motion.forward == 0.0

blocked = planner.solve(ha.Pose2D(1.0, 1.0, 0.0), ha.Pose2D(3.0, 3.0, 0.0))
assert not blocked.success
assert blocked.path == []
