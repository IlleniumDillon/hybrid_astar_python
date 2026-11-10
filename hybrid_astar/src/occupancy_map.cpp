#include "occupancy_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

long double cross(const Point2D& a, const Point2D& b, const Point2D& c) {
    const long double abx = static_cast<long double>(b.x) - a.x;
    const long double aby = static_cast<long double>(b.y) - a.y;
    const long double acx = static_cast<long double>(c.x) - a.x;
    const long double acy = static_cast<long double>(c.y) - a.y;
    return abx * acy - aby * acx;
}

void validateFootprint(std::vector<Point2D>& footprint) {
    if (footprint.empty()) {
        return;
    }
    for (const auto& vertex : footprint) {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)) {
            throw std::invalid_argument("footprint vertices must be finite");
        }
    }
    if (footprint.size() > 3 &&
        footprint.front().x == footprint.back().x &&
        footprint.front().y == footprint.back().y) {
        footprint.pop_back();
    }
    if (footprint.size() < 3) {
        throw std::invalid_argument("footprint needs at least three vertices");
    }

    long double area2 = 0.0L;
    for (std::size_t i = 0; i < footprint.size(); ++i) {
        const auto& a = footprint[i];
        const auto& b = footprint[(i + 1) % footprint.size()];
        area2 += static_cast<long double>(a.x) * b.y -
                 static_cast<long double>(a.y) * b.x;
        for (std::size_t j = 0; j < i; ++j) {
            if (a.x == footprint[j].x && a.y == footprint[j].y) {
                throw std::invalid_argument("footprint vertices must be distinct");
            }
        }
    }
    if (area2 == 0.0L || !std::isfinite(area2)) {
        throw std::invalid_argument("footprint must have nonzero finite area");
    }

    const bool counterclockwise = area2 > 0.0L;
    for (std::size_t i = 0; i < footprint.size(); ++i) {
        const auto& a = footprint[i];
        const auto& b = footprint[(i + 1) % footprint.size()];
        for (const auto& vertex : footprint) {
            const long double side = cross(a, b, vertex);
            if ((counterclockwise && side < 0.0L) ||
                (!counterclockwise && side > 0.0L)) {
                throw std::invalid_argument("footprint must be a convex polygon");
            }
        }
    }
}

bool overlapsCell(const std::vector<Point2D>& polygon,
                  long double cell_x, long double cell_y, long double half_cell) {
    long double min_x = std::numeric_limits<long double>::infinity();
    long double max_x = -std::numeric_limits<long double>::infinity();
    long double min_y = std::numeric_limits<long double>::infinity();
    long double max_y = -std::numeric_limits<long double>::infinity();
    for (const auto& vertex : polygon) {
        min_x = std::min(min_x, static_cast<long double>(vertex.x));
        max_x = std::max(max_x, static_cast<long double>(vertex.x));
        min_y = std::min(min_y, static_cast<long double>(vertex.y));
        max_y = std::max(max_y, static_cast<long double>(vertex.y));
    }
    const long double left = cell_x - half_cell;
    const long double right = cell_x + half_cell;
    const long double bottom = cell_y - half_cell;
    const long double top = cell_y + half_cell;
    if (max_x < left || min_x > right || max_y < bottom || min_y > top) {
        return false;
    }

    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        const long double axis_x = static_cast<long double>(a.y) - b.y;
        const long double axis_y = static_cast<long double>(b.x) - a.x;
        long double poly_min = std::numeric_limits<long double>::infinity();
        long double poly_max = -std::numeric_limits<long double>::infinity();
        for (const auto& vertex : polygon) {
            const long double projection = axis_x * vertex.x + axis_y * vertex.y;
            poly_min = std::min(poly_min, projection);
            poly_max = std::max(poly_max, projection);
        }
        const long double cell_center = axis_x * cell_x + axis_y * cell_y;
        const long double cell_radius = half_cell *
            (std::abs(axis_x) + std::abs(axis_y));
        if (poly_max < cell_center - cell_radius ||
            poly_min > cell_center + cell_radius) {
            return false;
        }
    }
    return true;
}

std::vector<Point2D> convexHull(std::vector<Point2D> points) {
    std::sort(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    points.erase(std::unique(points.begin(), points.end(),
        [](const Point2D& a, const Point2D& b) {
            return a.x == b.x && a.y == b.y;
        }), points.end());
    if (points.size() <= 2) {
        return points;
    }

    std::vector<Point2D> hull;
    hull.reserve(points.size() * 2);
    for (const auto& point : points) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0L) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const std::size_t lower_size = hull.size();
    for (std::size_t i = points.size() - 1; i > 0; --i) {
        const auto& point = points[i - 1];
        while (hull.size() > lower_size &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0L) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    hull.pop_back();
    return hull;
}

bool envelopeClear(const OriginalMap& map, const std::vector<Point2D>& hull,
                   long double error) {
    long double min_x = std::numeric_limits<long double>::infinity();
    long double max_x = -std::numeric_limits<long double>::infinity();
    long double min_y = std::numeric_limits<long double>::infinity();
    long double max_y = -std::numeric_limits<long double>::infinity();
    for (const auto& vertex : hull) {
        min_x = std::min(min_x, static_cast<long double>(vertex.x));
        max_x = std::max(max_x, static_cast<long double>(vertex.x));
        min_y = std::min(min_y, static_cast<long double>(vertex.y));
        max_y = std::max(max_y, static_cast<long double>(vertex.y));
    }

    const long double resolution = map.posRes;
    const long double origin_x = map.originX;
    const long double origin_y = map.originY;
    const long double half = resolution / 2.0L;
    if (min_x - error <= origin_x - half ||
        max_x + error >= origin_x + (map.widthI - 0.5L) * resolution ||
        min_y - error <= origin_y - half ||
        max_y + error >= origin_y + (map.heightI - 0.5L) * resolution) {
        return false;
    }

    const auto firstIndex = [resolution](long double minimum,
                                          long double origin, int size) {
        return static_cast<int>(std::clamp(
            std::ceil((minimum - origin) / resolution - 0.5L),
            0.0L, static_cast<long double>(size - 1)));
    };
    const auto lastIndex = [resolution](long double maximum,
                                         long double origin, int size) {
        return static_cast<int>(std::clamp(
            std::floor((maximum - origin) / resolution + 0.5L),
            0.0L, static_cast<long double>(size - 1)));
    };
    const int min_ix = firstIndex(min_x - error, origin_x, map.widthI);
    const int max_ix = lastIndex(max_x + error, origin_x, map.widthI);
    const int min_iy = firstIndex(min_y - error, origin_y, map.heightI);
    const int max_iy = lastIndex(max_y + error, origin_y, map.heightI);
    for (int iy = min_iy; iy <= max_iy; ++iy) {
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            if (map.isOccupied(ix, iy) &&
                overlapsCell(hull,
                    origin_x + static_cast<long double>(ix) * resolution,
                    origin_y + static_cast<long double>(iy) * resolution,
                    half + error)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

OccupancyMap::OccupancyMap(OriginalMap map, std::vector<Point2D> footprint,
                           int angle_bins)
    : map_(std::move(map)), footprint_(std::move(footprint)),
      angle_bins_(angle_bins),
      angle_resolution_(angle_bins > 0 ? kTwoPi / angle_bins : 0.0) {
    if (map_.widthI <= 0 || map_.heightI <= 0 ||
        map_.data.size() != static_cast<std::size_t>(map_.heightI) ||
        !std::isfinite(map_.originX) || !std::isfinite(map_.originY) ||
        !std::isfinite(map_.posRes) || map_.posRes <= 0.0) {
        throw std::invalid_argument("occupancy map requires a valid grid");
    }
    for (const auto& row : map_.data) {
        if (row.size() != static_cast<std::size_t>(map_.widthI)) {
            throw std::invalid_argument("occupancy grid rows must have equal width");
        }
    }
    if (angle_bins_ <= 0) {
        throw std::invalid_argument("angle bin count must be positive");
    }
    validateFootprint(footprint_);
    for (const auto& vertex : footprint_) {
        footprint_radius_ = std::max(footprint_radius_, std::hypot(vertex.x, vertex.y));
    }
    if (!std::isfinite(footprint_radius_)) {
        throw std::invalid_argument("footprint radius must be finite");
    }
}

bool OccupancyMap::isOccupied(int x, int y, int heading_bin) const {
    if (x < 0 || x >= map_.widthI || y < 0 || y >= map_.heightI ||
        heading_bin < 0 || heading_bin >= angle_bins_) {
        return true;
    }
    const Pose2D pose{
        map_.originX + static_cast<double>(x) * map_.posRes,
        map_.originY + static_cast<double>(y) * map_.posRes,
        heading_bin * angle_resolution_
    };
    return isOccupied(pose);
}

bool OccupancyMap::isOccupied(const Pose2D& pose) const {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta)) {
        return true;
    }

    const double half = 0.5 * map_.posRes;
    const double left = map_.originX - half;
    const double right = map_.originX + (static_cast<double>(map_.widthI) - 0.5) * map_.posRes;
    const double bottom = map_.originY - half;
    const double top = map_.originY + (static_cast<double>(map_.heightI) - 0.5) * map_.posRes;

    std::vector<Point2D> polygon;
    polygon.reserve(footprint_.size());
    double min_x = footprint_.empty() ? pose.x : std::numeric_limits<double>::infinity();
    double max_x = footprint_.empty() ? pose.x : -std::numeric_limits<double>::infinity();
    double min_y = footprint_.empty() ? pose.y : std::numeric_limits<double>::infinity();
    double max_y = footprint_.empty() ? pose.y : -std::numeric_limits<double>::infinity();
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    for (const auto& vertex : footprint_) {
        const Point2D transformed{
            pose.x + c * vertex.x - s * vertex.y,
            pose.y + s * vertex.x + c * vertex.y
        };
        if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y)) {
            return true;
        }
        min_x = std::min(min_x, transformed.x);
        max_x = std::max(max_x, transformed.x);
        min_y = std::min(min_y, transformed.y);
        max_y = std::max(max_y, transformed.y);
        polygon.push_back(transformed);
    }

    if (min_x <= left || max_x >= right || min_y <= bottom || max_y >= top) {
        return true;
    }

    const int min_ix = std::max(0, static_cast<int>(std::ceil((min_x - map_.originX) / map_.posRes - 0.5)));
    const int max_ix = std::min(map_.widthI - 1, static_cast<int>(std::floor((max_x - map_.originX) / map_.posRes + 0.5)));
    const int min_iy = std::max(0, static_cast<int>(std::ceil((min_y - map_.originY) / map_.posRes - 0.5)));
    const int max_iy = std::min(map_.heightI - 1, static_cast<int>(std::floor((max_y - map_.originY) / map_.posRes + 0.5)));
    for (int iy = min_iy; iy <= max_iy; ++iy) {
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            if (!map_.isOccupied(ix, iy)) {
                continue;
            }
            if (polygon.empty() || overlapsCell(
                    polygon,
                    map_.originX + static_cast<double>(ix) * map_.posRes,
                    map_.originY + static_cast<double>(iy) * map_.posRes,
                    half)) {
                return true;
            }
        }
    }
    return false;
}

bool OccupancyMap::isMotionCollisionFree(const Pose2D& start,
                                         const Motion& motion) const {
    if (isOccupied(start) || !std::isfinite(motion.forward) ||
        !std::isfinite(motion.lateral) || !std::isfinite(motion.angular)) {
        return false;
    }

    Pose2D end;
    try {
        end = applyMotion(start, motion);
    } catch (const std::invalid_argument&) {
        return false;
    }
    if (isOccupied(end)) {
        return false;
    }

    const auto footprintAt = [this](const Pose2D& pose) {
        std::vector<Point2D> vertices;
        if (footprint_.empty()) {
            vertices.push_back({pose.x, pose.y});
            return vertices;
        }
        vertices.reserve(footprint_.size());
        const double c = std::cos(pose.theta);
        const double s = std::sin(pose.theta);
        for (const auto& vertex : footprint_) {
            const Point2D transformed{
                pose.x + c * vertex.x - s * vertex.y,
                pose.y + s * vertex.x + c * vertex.y
            };
            if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y)) {
                return std::vector<Point2D>{};
            }
            vertices.push_back(transformed);
        }
        return vertices;
    };
    const std::vector<Point2D> start_vertices = footprintAt(start);
    const std::vector<Point2D> end_vertices = footprintAt(end);
    if (start_vertices.empty() || end_vertices.empty()) {
        return false;
    }

    // A vertex's second derivative is bounded by the turning translation
    // acceleration plus the footprint's centripetal acceleration.
    const long double turn = std::abs(static_cast<long double>(motion.angular));
    const long double translation = std::hypot(
        static_cast<long double>(motion.forward),
        static_cast<long double>(motion.lateral));
    const long double acceleration = turn * translation +
        turn * turn * static_cast<long double>(footprint_radius_);
    if (!std::isfinite(acceleration)) {
        return false;
    }

    constexpr int kMaxDepth = 22;
    const auto checkInterval = [&](const auto& recurse, double t0, double t1,
                                   const std::vector<Point2D>& at_start,
                                   const std::vector<Point2D>& at_end,
                                   int depth) -> bool {
        std::vector<Point2D> endpoint_vertices = at_start;
        endpoint_vertices.insert(endpoint_vertices.end(), at_end.begin(), at_end.end());
        const std::vector<Point2D> hull = convexHull(std::move(endpoint_vertices));
        const long double span = static_cast<long double>(t1) - t0;
        const long double bound = acceleration * span * span / 8.0L;
        if (!std::isfinite(bound)) {
            return false;
        }
        const long double error = bound > 0.0L
            ? std::nextafter(bound, std::numeric_limits<long double>::infinity())
            : 0.0L;
        if (envelopeClear(map_, hull, error)) {
            return true;
        }
        if (motion.angular == 0.0 || depth >= kMaxDepth) {
            return false;
        }

        const double middle = t0 + (t1 - t0) / 2.0;
        if (middle <= t0 || middle >= t1) {
            return false;
        }
        Pose2D midway;
        try {
            midway = applyMotion(start, motion, middle);
        } catch (const std::invalid_argument&) {
            return false;
        }
        if (isOccupied(midway)) {
            return false;
        }
        const std::vector<Point2D> at_middle = footprintAt(midway);
        if (at_middle.empty()) {
            return false;
        }
        return recurse(recurse, t0, middle, at_start, at_middle, depth + 1) &&
               recurse(recurse, middle, t1, at_middle, at_end, depth + 1);
    };
    return checkInterval(checkInterval, 0.0, 1.0,
                         start_vertices, end_vertices, 0);
}
