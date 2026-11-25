#include "opw_kinematics/OPWKinematics.h"

#include "Transforms.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace opw {
namespace {

using detail::kJointLimitTolerance;
using detail::kPi;
using detail::kTwoPi;

// The free arm joint enters the wrist rotation through one sine and one cosine.
// Recover the coefficients at 0, pi/2, pi, and -pi/2 to find limit crossings.
template <typename WristExpression>
void AppendCrossings(const std::array<Eigen::Matrix3d, 4>& quadrants,
                     WristExpression expression, double lower, double upper,
                     std::vector<double>& crossings) {
    const double at_zero = expression(quadrants[0]);
    const double at_pi = expression(quadrants[2]);
    const double cosine = (at_zero - at_pi) / 2.0;
    const double sine = (expression(quadrants[1]) - expression(quadrants[3])) / 2.0;
    const double constant = (at_zero + at_pi) / 2.0;
    const double amplitude = std::hypot(cosine, sine);
    constexpr double roundoff = 64.0 * std::numeric_limits<double>::epsilon();
    if (amplitude <= roundoff || std::abs(constant) > amplitude + roundoff) {
        return;
    }

    const double center = std::atan2(sine, cosine);
    const double span = std::acos(std::clamp(-constant / amplitude, -1.0, 1.0));
    for (double root : {center - span, center + span}) {
        const double turn = std::ceil((lower - root - kJointLimitTolerance) / kTwoPi);
        const double angle = std::fma(turn, kTwoPi, root);
        if (angle >= lower - kJointLimitTolerance && angle <= upper + kJointLimitTolerance) {
            crossings.push_back(std::clamp(angle, lower, upper));
        }
    }
}

}  // namespace

std::vector<double> OPWKinematics::FreeArmRepresentatives(
        int joint, double seed, const std::array<Eigen::Matrix3d, 4>& wrist_quadrants) const {
    const auto [joint_lower, joint_upper] = IKJointInterval(joint);
    if (joint_lower > joint_upper) {
        return {};
    }
    const double width = std::min(kTwoPi, joint_upper - joint_lower);
    const double lower = std::clamp(seed - kPi, joint_lower, std::max(joint_lower, joint_upper - width));
    const double upper = std::min(joint_upper, lower + width);
    const auto model_limits = [&](int index) -> std::pair<double, double> {
        const auto [l, u] = IKJointInterval(index);
        if (l > u) {
            return {1.0, 0.0};
        }
        const double direction = params_.rotation_directions[index];
        const double a = l * direction - params_.offsets[index];
        const double b = u * direction - params_.offsets[index];
        return {std::min(a, b), std::max(a, b)};
    };
    const auto [lower4, upper4] = model_limits(3);
    const auto [lower5, upper5] = model_limits(4);
    const auto [lower6, upper6] = model_limits(5);
    if (lower4 > upper4 || lower5 > upper5 || lower6 > upper6) {
        return {};
    }

    std::vector<double> crossings{lower, upper};
    const auto add_pitch_boundary = [&](double cosine) {
        AppendCrossings(wrist_quadrants,
                [cosine](const Eigen::Matrix3d& w) { return w(2, 2) - cosine; }, lower, upper, crossings);
    };
    // At isolated wrist singularities the two Euler-angle branches can switch.
    add_pitch_boundary(1.0);
    add_pitch_boundary(-1.0);
    if (upper5 - lower5 < kTwoPi) {
        add_pitch_boundary(std::cos(lower5));
        add_pitch_boundary(std::cos(upper5));
    }

    const auto append_axis_boundary = [&](double angle, int axis) {
        const double sine = std::sin(angle), cosine = std::cos(angle);
        AppendCrossings(wrist_quadrants, [=](const Eigen::Matrix3d& w) {
            // Zero of sin(q - angle) times the common sin(q5) factor. This
            // captures both the positive-pitch Euler branch and its flipped branch.
            return axis == 4 ? w(1, 2) * cosine - w(0, 2) * sine
                             : w(2, 1) * cosine + w(2, 0) * sine;
        }, lower, upper, crossings);
    };
    if (upper4 - lower4 < kTwoPi) {
        append_axis_boundary(lower4, 4);
        append_axis_boundary(upper4, 4);
    }
    if (upper6 - lower6 < kTwoPi) {
        append_axis_boundary(lower6, 6);
        append_axis_boundary(upper6, 6);
    }

    const bool aligned = wrist_quadrants[0](2, 2) >= 0.0;
    const bool always_singular = std::all_of(wrist_quadrants.begin(), wrist_quadrants.end(),
            [aligned](const Eigen::Matrix3d& w) {
                return (w(2, 2) >= 0.0) == aligned &&
                       std::hypot(w(0, 2), w(1, 2)) <= detail::kWristSingularTolerance;
            });
    if (always_singular) {
        // If pitch stays singular across the whole range, only the wrist phase is
        // observable. Its limits are the sum or difference of the two joint intervals.
        const double phase_lower = lower6 + (aligned ? lower4 : -upper4);
        const double phase_upper = upper6 + (aligned ? upper4 : -lower4);
        if (phase_upper - phase_lower < kTwoPi) {
            for (double limit : {phase_lower, phase_upper}) {
                const double sine = std::sin(limit), cosine = std::cos(limit);
                AppendCrossings(wrist_quadrants, [=](const Eigen::Matrix3d& w) {
                    const double phase_sine = w(1, 0) + (aligned ? -w(0, 1) : w(0, 1));
                    const double phase_cosine = w(1, 1) + (aligned ? w(0, 0) : -w(0, 0));
                    return phase_sine * cosine - phase_cosine * sine;
                }, lower, upper, crossings);
            }
        }
    }

    std::sort(crossings.begin(), crossings.end());
    crossings.erase(std::unique(crossings.begin(), crossings.end(), [](double a, double b) {
        return b - a <= kJointLimitTolerance;
    }), crossings.end());
    // Feasibility is constant between adjacent wrist-limit crossings. Test those
    // crossings and one interior point to handle both closed limits and open arcs.
    std::vector<double> representatives;
    representatives.reserve(crossings.size() * 2);
    for (std::size_t i = 0; i < crossings.size(); ++i) {
        if (i > 0) {
            representatives.push_back(crossings[i - 1] + (crossings[i] - crossings[i - 1]) / 2.0);
        }
        representatives.push_back(crossings[i]);
    }
    return representatives;
}

}  // namespace opw
