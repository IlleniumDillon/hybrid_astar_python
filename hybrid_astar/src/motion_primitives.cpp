#include "motion_primitives.hpp"

#include <cmath>
#include <stdexcept>

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

bool finiteMotion(const Motion& motion) {
    return std::isfinite(motion.forward) && std::isfinite(motion.lateral) &&
           std::isfinite(motion.angular);
}

} // namespace

double normalizeAngle(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("angle must be finite");
    }
    double normalized = std::fmod(angle, kTwoPi);
    if (normalized < 0.0) {
        normalized += kTwoPi;
    }
    return normalized == kTwoPi ? 0.0 : normalized;
}

Pose2D applyMotion(const Pose2D& start, const Motion& motion, double fraction) {
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(start.theta) || !finiteMotion(motion) ||
        !std::isfinite(fraction)) {
        throw std::invalid_argument("pose, motion, and fraction must be finite");
    }

    const double turn = motion.angular * fraction;
    if (!std::isfinite(turn)) {
        throw std::invalid_argument("motion turn exceeds finite range");
    }

    double sinc;
    double cosc;
    if (std::abs(turn) < 1e-4) {
        const double turn2 = turn * turn;
        sinc = 1.0 - turn2 / 6.0 + turn2 * turn2 / 120.0;
        cosc = turn * (0.5 - turn2 / 24.0 + turn2 * turn2 / 720.0);
    } else {
        sinc = std::sin(turn) / turn;
        const double half_sine = std::sin(turn / 2.0);
        cosc = 2.0 * half_sine * half_sine / turn;
    }

    const double forward = motion.forward * fraction;
    const double lateral = motion.lateral * fraction;
    const double body_x = forward * sinc - lateral * cosc;
    const double body_y = forward * cosc + lateral * sinc;
    const double heading = normalizeAngle(start.theta);
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    const Pose2D result{
        start.x + cosine * body_x - sine * body_y,
        start.y + sine * body_x + cosine * body_y,
        normalizeAngle(heading + turn)
    };
    if (!std::isfinite(result.x) || !std::isfinite(result.y)) {
        throw std::invalid_argument("motion result exceeds finite range");
    }
    return result;
}

std::vector<Motion> generateMotionPrimitives(const MotionPrimitiveConfig& config) {
    if (!std::isfinite(config.max_forward_step) ||
        !std::isfinite(config.max_lateral_step) ||
        !std::isfinite(config.max_angular_step) ||
        config.max_forward_step < 0.0 || config.max_lateral_step < 0.0 ||
        config.max_angular_step < 0.0 || config.forward_steps < 0 ||
        config.lateral_steps < 0 || config.angular_steps < 0) {
        throw std::invalid_argument("invalid motion primitive configuration");
    }

    std::vector<Motion> primitives;
    for (int fi = 0; fi <= config.forward_steps; ++fi) {
        if (fi != 0 && config.max_forward_step == 0.0) {
            continue;
        }
        const double forward = config.forward_steps == 0 ? 0.0 :
            config.max_forward_step * (static_cast<double>(fi) / config.forward_steps);
        for (int li = -config.lateral_steps; li <= config.lateral_steps; ++li) {
            if (li != 0 && config.max_lateral_step == 0.0) {
                continue;
            }
            const double lateral = config.lateral_steps == 0 ? 0.0 :
                config.max_lateral_step * (static_cast<double>(li) / config.lateral_steps);
            for (int ai = -config.angular_steps; ai <= config.angular_steps; ++ai) {
                if (ai != 0 && config.max_angular_step == 0.0) {
                    continue;
                }
                const double angular = config.angular_steps == 0 ? 0.0 :
                    config.max_angular_step * (static_cast<double>(ai) / config.angular_steps);
                const Motion motion{forward, lateral, angular};
                if (forward == 0.0 && lateral == 0.0 && angular == 0.0) {
                    continue;
                }
                primitives.push_back(motion);
            }
        }
    }
    return primitives;
}
