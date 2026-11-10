#include "hybrid_astar.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(hybrid_astar, module) {
    module.doc() = "Single-robot Hybrid A* path planning";

    py::class_<Point2D>(module, "Point2D")
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Point2D::x)
        .def_readwrite("y", &Point2D::y);

    py::class_<Pose2D>(module, "Pose2D")
        .def(py::init<double, double, double>(),
             py::arg("x"), py::arg("y"), py::arg("theta"))
        .def_readwrite("x", &Pose2D::x)
        .def_readwrite("y", &Pose2D::y)
        .def_readwrite("theta", &Pose2D::theta);

    py::class_<Motion>(module, "Motion")
        .def(py::init<double, double, double>(),
             py::arg("forward"), py::arg("lateral"), py::arg("angular"))
        .def_readwrite("forward", &Motion::forward)
        .def_readwrite("lateral", &Motion::lateral)
        .def_readwrite("angular", &Motion::angular);

    py::class_<MotionPrimitiveConfig>(module, "MotionPrimitiveConfig")
        .def(py::init<>())
        .def_readwrite("max_forward_step", &MotionPrimitiveConfig::max_forward_step)
        .def_readwrite("max_lateral_step", &MotionPrimitiveConfig::max_lateral_step)
        .def_readwrite("max_angular_step", &MotionPrimitiveConfig::max_angular_step)
        .def_readwrite("forward_steps", &MotionPrimitiveConfig::forward_steps)
        .def_readwrite("lateral_steps", &MotionPrimitiveConfig::lateral_steps)
        .def_readwrite("angular_steps", &MotionPrimitiveConfig::angular_steps);

    py::class_<OriginalMap>(module, "OriginalMap")
        .def(py::init<const std::vector<std::vector<bool>>&, double, double, double>(),
             py::arg("grid"), py::arg("origin_x"), py::arg("origin_y"),
             py::arg("resolution"))
        .def_property_readonly("grid", [](const OriginalMap& map) { return map.data; })
        .def_readonly("width", &OriginalMap::widthI)
        .def_readonly("height", &OriginalMap::heightI)
        .def_readonly("origin_x", &OriginalMap::originX)
        .def_readonly("origin_y", &OriginalMap::originY)
        .def_readonly("resolution", &OriginalMap::posRes)
        .def("is_occupied", [](const OriginalMap& map, int x, int y) {
            return map.isOccupied(x, y);
        }, py::arg("x"), py::arg("y"));

    py::class_<OccupancyMap>(module, "OccupancyMap")
        .def(py::init<OriginalMap, std::vector<Point2D>, int>(),
             py::arg("original_map"), py::arg("footprint"), py::arg("angle_bins"))
        .def_property_readonly("angle_bins", &OccupancyMap::angleBins)
        .def_property_readonly("angle_resolution", &OccupancyMap::angleResolution)
        .def_property_readonly("original_map", &OccupancyMap::originalMap,
                               py::return_value_policy::reference_internal)
        .def("is_occupied",
             py::overload_cast<const Pose2D&>(&OccupancyMap::isOccupied, py::const_),
             py::arg("pose"));

    py::class_<HybridAStarConfig>(module, "HybridAStarConfig")
        .def(py::init<>())
        .def_readwrite("motion", &HybridAStarConfig::motion)
        .def_readwrite("max_iterations", &HybridAStarConfig::max_iterations);

    py::class_<PathWaypoint>(module, "PathWaypoint")
        .def_readonly("pose", &PathWaypoint::pose)
        .def_readonly("motion", &PathWaypoint::motion);

    py::class_<HybridAStarResult>(module, "HybridAStarResult")
        .def_readonly("success", &HybridAStarResult::success)
        .def_readonly("iterations", &HybridAStarResult::iterations)
        .def_readonly("cost", &HybridAStarResult::cost)
        .def_readonly("path", &HybridAStarResult::path);

    py::class_<HybridAStar>(module, "HybridAStar")
        .def(py::init<OccupancyMap, HybridAStarConfig>(),
             py::arg("occupancy_map"), py::arg("config") = HybridAStarConfig{})
        .def("solve", &HybridAStar::solve, py::arg("start"), py::arg("goal"))
        .def_property_readonly("primitives", &HybridAStar::motionPrimitives);
}
