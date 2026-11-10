#include "hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct Key {
    std::int64_t x;
    std::int64_t y;
    int heading;

    bool operator==(const Key& other) const {
        return x == other.x && y == other.y && heading == other.heading;
    }
};

struct KeyHash {
    std::size_t operator()(const Key& key) const {
        std::size_t hash = std::hash<std::int64_t>{}(key.x);
        hash ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<int>{}(key.heading) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

struct Label {
    Key key;
    Pose2D pose;
    Motion incoming{};
    std::size_t parent = 0;
    bool has_parent = false;
    std::size_t steps = 0;
};

struct OpenEntry {
    double priority;
    double obstacle_distance;
    std::size_t label;
    std::size_t order;
};

struct OpenEntryLater {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        if (a.priority != b.priority) return a.priority > b.priority;
        if (a.obstacle_distance != b.obstacle_distance) {
            return a.obstacle_distance > b.obstacle_distance;
        }
        return a.order > b.order;
    }
};

struct SearchGrid {
    double position_resolution;
    int heading_bins;
};

Key poseKey(const Pose2D& pose, const OccupancyMap& map, const SearchGrid& grid) {
    const OriginalMap& original = map.originalMap();
    const double angle_resolution = kTwoPi / grid.heading_bins;
    const int heading = static_cast<int>(std::llround(normalizeAngle(pose.theta) / angle_resolution))
        % grid.heading_bins;
    return {
        std::llround((pose.x - original.originX) / grid.position_resolution),
        std::llround((pose.y - original.originY) / grid.position_resolution),
        heading
    };
}

SearchGrid searchGrid(const OccupancyMap& map, const std::vector<Motion>& motions) {
    double smallest_translation = std::numeric_limits<double>::infinity();
    double smallest_rotation = std::numeric_limits<double>::infinity();
    for (const Motion& motion : motions) {
        if (std::hypot(motion.forward, motion.lateral) > 0.0) {
            try {
                const Pose2D endpoint = applyMotion({0.0, 0.0, 0.0}, motion);
                const double displacement = std::hypot(endpoint.x, endpoint.y);
                if (displacement > 0.0) {
                    smallest_translation = std::min(smallest_translation, displacement);
                }
            } catch (const std::invalid_argument&) {
                // An overflowing primitive cannot produce a usable successor.
            }
        }
        if (motion.angular != 0.0) {
            smallest_rotation = std::min(smallest_rotation, std::abs(motion.angular));
        }
    }
    const double map_resolution = map.originalMap().posRes;
    double position_resolution = map_resolution;
    if (std::isfinite(smallest_translation)) {
        position_resolution = std::min(map_resolution,
            std::max(smallest_translation / 2.0, map_resolution / 1'000'000.0));
    }
    int heading_bins = map.angleBins();
    if (std::isfinite(smallest_rotation)) {
        const double desired_bins = std::ceil(2.0 * kTwoPi / smallest_rotation);
        heading_bins = std::max(heading_bins,
            static_cast<int>(std::min(desired_bins, 1'000'000.0)));
    }
    return {position_resolution, heading_bins};
}

double circularDistance(double a, double b) {
    const double difference = std::abs(normalizeAngle(a) - normalizeAngle(b));
    return std::min(difference, kTwoPi - difference);
}

double heuristic(const Pose2D& pose, const Key& goal, const OccupancyMap& map,
                 const MotionPrimitiveConfig& motion) {
    const OriginalMap& original = map.originalMap();
    const double goal_x = original.originX + goal.x * original.posRes;
    const double goal_y = original.originY + goal.y * original.posRes;
    const double half_cell = original.posRes / 2.0;
    const double dx = std::max(0.0, std::abs(pose.x - goal_x) - half_cell);
    const double dy = std::max(0.0, std::abs(pose.y - goal_y) - half_cell);
    const double max_translation = std::hypot(motion.max_forward_step, motion.max_lateral_step);
    const double translation_steps = max_translation > 0.0
        ? std::hypot(dx, dy) / max_translation
        : (dx == 0.0 && dy == 0.0 ? 0.0 : std::numeric_limits<double>::infinity());

    const double goal_heading = goal.heading * map.angleResolution();
    const double heading_gap = std::max(0.0,
        circularDistance(pose.theta, goal_heading) - map.angleResolution() / 2.0);
    const double rotation_steps = motion.max_angular_step > 0.0
        ? heading_gap / motion.max_angular_step
        : (heading_gap == 0.0 ? 0.0 : std::numeric_limits<double>::infinity());
    return std::max(translation_steps, rotation_steps);
}

std::vector<double> obstacleDistances(const OriginalMap& map, const Key& goal) {
    const std::size_t cells = static_cast<std::size_t>(map.widthI) * map.heightI;
    std::vector<double> distances(cells, std::numeric_limits<double>::infinity());
    const auto index = [&map](int x, int y) {
        return static_cast<std::size_t>(y) * map.widthI + x;
    };
    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    distances[index(static_cast<int>(goal.x), static_cast<int>(goal.y))] = 0.0;
    open.push({0.0, index(static_cast<int>(goal.x), static_cast<int>(goal.y))});

    constexpr std::array<std::pair<int, int>, 8> directions{{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    }};
    while (!open.empty()) {
        const auto [distance, cell] = open.top();
        open.pop();
        if (distance > distances[cell]) continue;
        const int x = static_cast<int>(cell % map.widthI);
        const int y = static_cast<int>(cell / map.widthI);
        for (const auto& [dx, dy] : directions) {
            const int next_x = x + dx;
            const int next_y = y + dy;
            if (map.isOccupied(next_x, next_y)) continue;
            if (dx != 0 && dy != 0 &&
                (map.isOccupied(x + dx, y) || map.isOccupied(x, y + dy))) continue;
            const std::size_t next = index(next_x, next_y);
            const double next_distance = distance + std::hypot(dx, dy) * map.posRes;
            if (next_distance < distances[next]) {
                distances[next] = next_distance;
                open.push({next_distance, next});
            }
        }
    }
    return distances;
}

} // namespace

HybridAStar::HybridAStar(OccupancyMap occupancy_map, HybridAStarConfig config)
    : occupancy_map_(std::move(occupancy_map)), config_(config),
      motions_(generateMotionPrimitives(config.motion)) {}

HybridAStarResult HybridAStar::solve(const Pose2D& start, const Pose2D& goal) const {
    HybridAStarResult result;
    if (occupancy_map_.isOccupied(start) || occupancy_map_.isOccupied(goal)) return result;

    const SearchGrid map_grid{occupancy_map_.originalMap().posRes, occupancy_map_.angleBins()};
    const Key goal_key = poseKey(goal, occupancy_map_, map_grid);
    if (poseKey(start, occupancy_map_, map_grid) == goal_key) {
        result.success = true;
        result.iterations = 1;
        result.cost = 0.0;
        result.path.push_back({start, {}});
        return result;
    }
    const SearchGrid search_grid = searchGrid(occupancy_map_, motions_);
    const Key start_key = poseKey(start, occupancy_map_, search_grid);
    const OriginalMap& original = occupancy_map_.originalMap();
    const auto field = obstacleDistances(original, goal_key);
    const auto obstacle_distance = [&field, &original, &map_grid, this](const Pose2D& pose) {
        const Key cell = poseKey(pose, occupancy_map_, map_grid);
        return field[static_cast<std::size_t>(cell.y) * original.widthI + cell.x];
    };

    std::vector<Label> labels;
    std::unordered_map<Key, std::size_t, KeyHash> best;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryLater> open;
    labels.push_back({start_key, start});
    best.emplace(start_key, 0);
    std::size_t order = 0;
    open.push({heuristic(start, goal_key, occupancy_map_, config_.motion),
               obstacle_distance(start), 0, order++});

    while (!open.empty()) {
        const OpenEntry entry = open.top();
        open.pop();
        const Label current = labels[entry.label];
        if (best.at(current.key) != entry.label) continue;
        ++result.iterations;

        if (poseKey(current.pose, occupancy_map_, map_grid) == goal_key) {
            result.success = true;
            result.cost = static_cast<double>(current.steps);
            std::vector<std::size_t> reverse_labels;
            std::size_t label = entry.label;
            while (true) {
                reverse_labels.push_back(label);
                if (!labels[label].has_parent) break;
                label = labels[label].parent;
            }
            result.path.reserve(reverse_labels.size());
            for (auto it = reverse_labels.rbegin(); it != reverse_labels.rend(); ++it) {
                result.path.push_back({labels[*it].pose, {}});
            }
            for (std::size_t i = 0; i + 1 < result.path.size(); ++i) {
                result.path[i].motion =
                    labels[reverse_labels[reverse_labels.size() - 2 - i]].incoming;
            }
            return result;
        }
        if (config_.max_iterations != 0 && result.iterations >= config_.max_iterations) break;

        const Pose2D current_pose = current.pose;
        const std::size_t next_steps = current.steps + 1;
        for (const Motion& motion : motions_) {
            Pose2D next_pose;
            try {
                next_pose = applyMotion(current_pose, motion);
            } catch (const std::invalid_argument&) {
                continue;
            }
            if (occupancy_map_.isOccupied(next_pose)) continue;
            const Key next_key = poseKey(next_pose, occupancy_map_, search_grid);
            if (next_key == current.key) continue;
            auto neighbor_it = best.find(next_key);
            if (neighbor_it != best.end() &&
                labels[neighbor_it->second].steps <= next_steps) continue;
            if (!occupancy_map_.isMotionCollisionFree(current_pose, motion)) continue;

            const double h = heuristic(next_pose, goal_key, occupancy_map_, config_.motion);
            if (!std::isfinite(h)) continue;
            const std::size_t next_label = labels.size();
            labels.push_back({next_key, next_pose, motion, entry.label, true, next_steps});
            best[next_key] = next_label;
            open.push({static_cast<double>(next_steps) + h,
                       obstacle_distance(next_pose), next_label, order++});
        }
    }
    return result;
}
