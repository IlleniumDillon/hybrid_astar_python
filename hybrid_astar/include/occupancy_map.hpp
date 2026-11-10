#pragma once

#include "motion_primitives.hpp"
#include "original_map.hpp"

#include <vector>

class OccupancyMap {
public:
    OccupancyMap(OriginalMap map, std::vector<Point2D> footprint, int angle_bins);

    const OriginalMap& originalMap() const noexcept { return map_; }
    int angleBins() const noexcept { return angle_bins_; }
    double angleResolution() const noexcept { return angle_resolution_; }
    double footprintRadius() const noexcept { return footprint_radius_; }

    bool isOccupied(const Pose2D& pose) const;
    bool isOccupied(int x, int y, int heading_bin) const;
    bool isMotionCollisionFree(const Pose2D& start, const Motion& motion) const;

private:
    OriginalMap map_;
    std::vector<Point2D> footprint_;
    int angle_bins_;
    double angle_resolution_;
    double footprint_radius_ = 0.0;
};
