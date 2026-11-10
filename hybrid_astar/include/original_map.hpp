#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

class OriginalMap {
public:
    int widthI = 0;
    int heightI = 0;
    double widthD = 0.0;
    double heightD = 0.0;
    double originX = 0.0;
    double originY = 0.0;
    double posRes = 0.0;
    std::vector<std::vector<bool>> data; // true means occupied

    OriginalMap() = default;

    OriginalMap(const std::vector<std::vector<bool>>& cells,
                double origin_x, double origin_y, double resolution)
        : originX(origin_x), originY(origin_y), posRes(resolution), data(cells) {
        if (data.empty() || data.front().empty() ||
            data.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            data.front().size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            !std::isfinite(originX) || !std::isfinite(originY) ||
            !std::isfinite(posRes) || posRes <= 0.0) {
            throw std::invalid_argument("invalid occupancy grid dimensions, origin, or resolution");
        }
        for (const auto& row : data) {
            if (row.size() != data.front().size()) {
                throw std::invalid_argument("occupancy grid rows must have equal width");
            }
        }

        widthI = static_cast<int>(data.front().size());
        heightI = static_cast<int>(data.size());
        widthD = static_cast<double>(widthI) * posRes;
        heightD = static_cast<double>(heightI) * posRes;
        if (!std::isfinite(widthD) || !std::isfinite(heightD) ||
            !std::isfinite(originX - 0.5 * posRes) ||
            !std::isfinite(originY - 0.5 * posRes) ||
            !std::isfinite(originX + (static_cast<double>(widthI) - 0.5) * posRes) ||
            !std::isfinite(originY + (static_cast<double>(heightI) - 0.5) * posRes)) {
            throw std::invalid_argument("occupancy grid world bounds must be finite");
        }
    }

    bool isOccupied(int x, int y) const noexcept {
        return x < 0 || y < 0 || x >= widthI || y >= heightI ||
               data[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
    }

    bool isOccupied(double x, double y) const noexcept {
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(originX) || !std::isfinite(originY) ||
            !std::isfinite(posRes) || posRes <= 0.0) {
            return true;
        }
        const double gx = std::round((x - originX) / posRes);
        const double gy = std::round((y - originY) / posRes);
        if (!std::isfinite(gx) || !std::isfinite(gy) ||
            gx < 0.0 || gy < 0.0 || gx >= widthI || gy >= heightI) {
            return true;
        }
        return isOccupied(static_cast<int>(gx), static_cast<int>(gy));
    }

    template <typename T, typename U>
    bool isOccupied(const T& x, const U& y) const noexcept {
        static_assert(std::is_arithmetic_v<T> && std::is_arithmetic_v<U>,
                      "coordinates must be arithmetic");
        if constexpr (std::is_integral_v<T> && std::is_integral_v<U>) {
            const long double gx = static_cast<long double>(x);
            const long double gy = static_cast<long double>(y);
            if (gx < 0.0L || gy < 0.0L || gx >= widthI || gy >= heightI) {
                return true;
            }
            return isOccupied(static_cast<int>(x), static_cast<int>(y));
        } else {
            return isOccupied(static_cast<double>(x), static_cast<double>(y));
        }
    }

    template <typename T>
    bool isOccupied(const T& point) const noexcept {
        static_assert(std::tuple_size<T>::value >= 2,
                      "point must have at least two coordinates");
        return isOccupied(std::get<0>(point), std::get<1>(point));
    }
};
