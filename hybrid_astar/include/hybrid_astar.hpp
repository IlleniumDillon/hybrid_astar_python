#pragma once

#include "motion_primitives.hpp"
#include "occupancy_map.hpp"

#include <cstddef>
#include <limits>
#include <vector>

struct PathWaypoint {
    Pose2D pose;
    Motion motion; // Motion from this pose to the next; zero at the final pose.
};

struct HybridAStarResult {
    bool success = false;
    std::size_t iterations = 0;
    double cost = std::numeric_limits<double>::infinity(); // Number of motion primitives.
    std::vector<PathWaypoint> path;
};

struct HybridAStarConfig {
    MotionPrimitiveConfig motion;
    std::size_t max_iterations = 0; // Zero means no explicit limit.
};

class HybridAStar {
public:
    HybridAStar(OccupancyMap occupancy_map, HybridAStarConfig config = {});

    HybridAStarResult solve(const Pose2D& start, const Pose2D& goal) const;

    const OccupancyMap& occupancyMap() const { return occupancy_map_; }
    const std::vector<Motion>& motionPrimitives() const { return motions_; }

private:
    OccupancyMap occupancy_map_;
    HybridAStarConfig config_;
    std::vector<Motion> motions_;
};
