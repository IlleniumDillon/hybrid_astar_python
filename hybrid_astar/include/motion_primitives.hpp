#pragma once

#include <vector>

struct Point2D {
    double x;
    double y;
};

struct Pose2D {
    double x;
    double y;
    double theta;
};

// One primitive describes a displacement in the robot's body frame.
struct Motion {
    double forward;
    double lateral;
    double angular;
};

struct MotionPrimitiveConfig {
    double max_forward_step = 1.0;
    double max_lateral_step = 0.0;
    double max_angular_step = 0.39269908169872414;
    int forward_steps = 1;
    int lateral_steps = 0;
    int angular_steps = 1;
};

double normalizeAngle(double angle);
Pose2D applyMotion(const Pose2D& start, const Motion& motion, double fraction = 1.0);
std::vector<Motion> generateMotionPrimitives(const MotionPrimitiveConfig& config);
