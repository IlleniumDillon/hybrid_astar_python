# pragma once

#include <vector>
#include <tuple>

class OriginalMap {
public:
    int widthI = 0;
    int heightI = 0;
    double widthD = 0.0;
    double heightD = 0.0;
    double originX = 0.0;
    double originY = 0.0;
    double posRes = 0.0;
    std::vector<std::vector<bool>> data; // true occupied, false free

    OriginalMap() {}
    OriginalMap(const std::vector<std::vector<bool>>& data_, double originX_, double originY_, double posRes_)
        : data(data_), originX(originX_), originY(originY_), posRes(posRes_) {
        heightI = data.size();
        widthI = (heightI > 0) ? data[0].size() : 0;
        widthD = widthI * posRes;
        heightD = heightI * posRes;
    }

    template <typename T>
    bool isOccupied(const T& p) const {
        // get number of elements in tuple
        constexpr size_t dim = std::tuple_size<T>::value;
        static_assert(dim >= 2, "Point type must have at least 2 dimensions");
        // determine int or double
        if constexpr (std::is_integral<typename std::tuple_element<0, T>::type>::value) {
            int x = static_cast<int>(std::get<0>(p));
            int y = static_cast<int>(std::get<1>(p));
            if (x < 0 || x >= widthI || y < 0 || y >= heightI) {
                return true; // out of bounds is considered occupied
            }
            return data[y][x];
        } else if constexpr (std::is_floating_point<typename std::tuple_element<0, T>::type>::value) {
            double x_d = static_cast<double>(std::get<0>(p));
            double y_d = static_cast<double>(std::get<1>(p));
            int x = static_cast<int>(std::round((x_d - originX) / posRes));
            int y = static_cast<int>(std::round((y_d - originY) / posRes));
            if (x < 0 || x >= widthI || y < 0 || y >= heightI) {
                return true; // out of bounds is considered occupied
            }
            return data[y][x];
        } else {
            static_assert(std::is_integral<typename std::tuple_element<0, T>::type>::value ||
                    std::is_floating_point<typename std::tuple_element<0, T>::type>::value, "Point type must have integral or floating point coordinates");
        }
    }

    template <typename T>
    bool isOccupied(const T& x, const T& y) const {
        if constexpr (std::is_integral<T>::value) {
            int xi = static_cast<int>(x);
            int yi = static_cast<int>(y);
            if (xi < 0 || xi >= widthI || yi < 0 || yi >= heightI) {
                return true; // out of bounds is considered occupied
            }
            return data[yi][xi];
        } else if constexpr (std::is_floating_point<T>::value) {
            double xd = static_cast<double>(x);
            double yd = static_cast<double>(y);
            int xi = static_cast<int>(std::round((xd - originX) / posRes));
            int yi = static_cast<int>(std::round((yd - originY) / posRes));
            if (xi < 0 || xi >= widthI || yi < 0 || yi >= heightI) {
                return true; // out of bounds is considered occupied
            }
            return data[yi][xi];
        } else {
            static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "Coordinate type must be integral or floating point");
        }
    }
};