#include "hybrid_astar.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

OriginalMap emptyMap(int width, int height) {
    return OriginalMap(std::vector<std::vector<bool>>(
        height, std::vector<bool>(width, false)), 0.0, 0.0, 1.0);
}

void checkPath(const HybridAStarResult& result, const OccupancyMap& map) {
    require(result.success, "expected a path");
    require(!result.path.empty(), "successful path is empty");
    require(result.path.size() == static_cast<std::size_t>(result.cost) + 1,
            "path length does not match action cost");
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        require(!map.isOccupied(result.path[i].pose), "path contains a collision");
        if (i + 1 == result.path.size()) break;
        const auto& waypoint = result.path[i];
        const Pose2D expected = applyMotion(waypoint.pose, waypoint.motion);
        const Pose2D actual = result.path[i + 1].pose;
        require(std::hypot(expected.x - actual.x, expected.y - actual.y) < 1e-8,
                "waypoint motion does not reach the next pose");
        require(std::abs(normalizeAngle(expected.theta - actual.theta)) < 1e-8 ||
                    std::abs(normalizeAngle(actual.theta - expected.theta)) < 1e-8,
                "waypoint heading does not match its motion");
        for (int sample = 1; sample <= 40; ++sample) {
            require(!map.isOccupied(applyMotion(waypoint.pose, waypoint.motion,
                                               static_cast<double>(sample) / 40)),
                    "path crosses an obstacle");
        }
    }
}

void testStraightPathAndReuse() {
    OccupancyMap map(emptyMap(9, 9), {}, 16);
    HybridAStar planner(map);
    const Pose2D start{1.0, 1.0, 0.0};
    const Pose2D goal{5.0, 1.0, 0.0};
    const auto first = planner.solve(start, goal);
    checkPath(first, map);
    require(first.cost == 4.0, "straight path should take four actions");
    const auto second = planner.solve(start, goal);
    require(second.success && second.cost == first.cost,
            "planner should work on repeated calls");
    require(!planner.solve(start, {20.0, 1.0, 0.0}).success,
            "out-of-bounds goal should fail");
    require(planner.solve(start, goal).success,
            "failed solve should not affect a later solve");
}

void testObstacleDetour() {
    auto data = std::vector<std::vector<bool>>(12, std::vector<bool>(12, false));
    for (int y = 3; y <= 8; ++y) data[y][5] = true;
    OccupancyMap map(OriginalMap(data, 0.0, 0.0, 1.0), {}, 4);
    HybridAStarConfig config;
    config.motion.max_angular_step = kPi / 2.0;
    HybridAStar planner(map, config);
    const auto path = planner.solve({2.0, 5.0, 0.0}, {9.0, 5.0, 0.0});
    checkPath(path, map);
    require(path.cost > 7.0, "wall should require a detour");
}

void testFootprintAndBoundaries() {
    auto data = std::vector<std::vector<bool>>(7, std::vector<bool>(7, false));
    data[3][3] = true;
    const std::vector<Point2D> square{{-0.35, -0.35}, {0.35, -0.35},
                                      {0.35, 0.35}, {-0.35, 0.35}};
    OccupancyMap map(OriginalMap(data, 0.0, 0.0, 1.0), square, 16);
    require(map.isOccupied(Pose2D{2.4, 3.0, 0.0}),
            "footprint should collide with obstacle cell area");
    require(map.isOccupied(Pose2D{-0.25, 1.0, 0.0}),
            "footprint extending outside the map should collide");
    require(!map.isOccupied(Pose2D{1.0, 1.0, 0.0}),
            "free footprint incorrectly collides");
    const Motion quarter_turn{0.0, 0.0, kPi / 2.0};
    require(!map.isOccupied(Pose2D{-0.02, 1.0, 0.0}) &&
                !map.isOccupied(Pose2D{-0.02, 1.0, kPi / 2.0}),
            "rotation endpoints should be clear in the swept test");
    require(!map.isMotionCollisionFree({-0.02, 1.0, 0.0}, quarter_turn),
            "rotating footprint should not cross the map boundary");
    require(map.isMotionCollisionFree({1.0, 1.0, 0.0}, quarter_turn),
            "clear rotation should remain available");
}

void testSweptCollision() {
    auto data = std::vector<std::vector<bool>>(5, std::vector<bool>(10, false));
    data[2][3] = true;
    OccupancyMap map(OriginalMap(data, 0.0, 0.0, 1.0), {}, 1);
    HybridAStarConfig config;
    config.motion.max_forward_step = 4.0;
    config.motion.max_angular_step = 0.0;
    config.motion.angular_steps = 0;
    HybridAStar planner(map, config);
    require(!planner.solve({1.0, 2.0, 0.0}, {5.0, 2.0, 0.0}).success,
            "primitive must not jump over an occupied cell");
}

void testCornerGrazingCollision() {
    auto data = std::vector<std::vector<bool>>(8, std::vector<bool>(8, false));
    data[2][3] = true;
    OccupancyMap map(OriginalMap(data, 0.0, 0.0, 1.0), {}, 8);
    HybridAStarConfig config;
    config.motion.max_forward_step = 4.0 * std::sqrt(2.0);
    config.motion.max_angular_step = 0.0;
    config.motion.angular_steps = 0;
    HybridAStar planner(map, config);
    require(!planner.solve({1.0, 0.99, kPi / 4.0},
                           {5.0, 4.99, kPi / 4.0}).success,
            "primitive must not graze a cell corner between samples");
}

void testSmallPrimitives() {
    OccupancyMap map(emptyMap(6, 4), {}, 4);
    HybridAStarConfig config;
    config.motion.max_forward_step = 0.4;
    config.motion.max_angular_step = 0.0;
    config.motion.angular_steps = 0;
    HybridAStar planner(map, config);
    const auto path = planner.solve({1.0, 1.0, 0.0}, {2.0, 1.0, 0.0});
    checkPath(path, map);
    require(path.cost == 2.0, "subcell moves should accumulate toward the goal");

    config.motion.max_forward_step = 0.0;
    config.motion.forward_steps = 0;
    config.motion.max_angular_step = kPi / 8.0;
    config.motion.angular_steps = 1;
    HybridAStar turning(map, config);
    const auto turn = turning.solve({2.0, 2.0, 0.0}, {2.0, 2.0, kPi / 2.0});
    checkPath(turn, map);
    require(turn.cost == 2.0, "small rotations should accumulate across angle bins");
}

void testHeadingWrap() {
    OccupancyMap map(emptyMap(5, 5), {}, 16);
    HybridAStar planner(map);
    const auto result = planner.solve({2.0, 2.0, 2.0 * kPi - 0.01},
                                      {2.0, 2.0, 0.01});
    require(result.success && result.cost == 0.0 && result.path.size() == 1,
            "headings on either side of 2pi should share a bin");
}

} // namespace

int main() {
    try {
        testStraightPathAndReuse();
        testObstacleDetour();
        testFootprintAndBoundaries();
        testSweptCollision();
        testCornerGrazingCollision();
        testSmallPrimitives();
        testHeadingWrap();
        std::cout << "Hybrid A* planner checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
