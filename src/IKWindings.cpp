#include "opw_kinematics/OPWKinematics.h"

#include "Transforms.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace opw {
namespace {

using detail::kJointLimitTolerance;
using detail::kTwoPi;

struct JointWindings {
    double angle;
    double lower;
    double upper;

    double First() const { return std::ceil((lower - angle - kJointLimitTolerance) / kTwoPi); }
    double Last() const { return std::floor((upper - angle + kJointLimitTolerance) / kTwoPi); }

    double At(double turn) const {
        return std::clamp(std::fma(turn, kTwoPi, angle), lower, upper);
    }

    std::array<double, 2> Around(double target, double first, double last) const {
        const double below = std::floor((target - angle) / kTwoPi);
        return {std::clamp(below, first, last), std::clamp(below + 1.0, first, last)};
    }
};

bool IsFixedWristAngle(int joint) {
    return joint == 3 || joint == 5;
}

}  // namespace

std::pair<double, double> OPWKinematics::IKJointInterval(int index) const {
    const auto& node = joint_nodes_[index];
    double lower = std::max(node.lower_limit, node.lower_limit_set);
    double upper = std::min(node.upper_limit, node.upper_limit_set);
    switch (joint_filters_[index]) {
        case FilterMode::POSITIVE:
            lower = std::max(lower, joint_thresholds_[index]);
            break;
        case FilterMode::NEGATIVE:
            upper = std::min(upper, -joint_thresholds_[index]);
            break;
        case FilterMode::NONE:
            break;
        default:
            return {1.0, 0.0};
    }
    return {lower, upper};
}

bool OPWKinematics::SelectClosestWindings(const Eigen::VectorXd& analytic, bool fixed_wrist_pair,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                                         Eigen::VectorXd& candidate) const {
    for (int joint = 0; joint < kDOF; ++joint) {
        if (fixed_wrist_pair && IsFixedWristAngle(joint)) {
            candidate[joint] = analytic[joint];
            continue;
        }

        const auto [lower, upper] = IKJointInterval(joint);
        const JointWindings windings{analytic[joint], lower, upper};
        const double first = windings.First();
        const double last = windings.Last();
        if (lower > upper || first > last) {
            return false;
        }
        if (first == last || ik_nearst_weight_[joint] == 0.0) {
            candidate[joint] = windings.At(first);
            continue;
        }

        // Only the windings immediately around the seed can minimize this joint's cost.
        const auto [below, above] = windings.Around(seed[joint], first, last);
        const double a = windings.At(below);
        const double b = windings.At(above);
        candidate[joint] = std::abs(b - seed[joint]) < std::abs(a - seed[joint]) ? b : a;
    }
    return true;
}

void OPWKinematics::AddLimitedSolutions(int index, int end, const Eigen::VectorXd& analytic,
                                        bool fixed_wrist_pair, bool check_elbow, Eigen::VectorXd& candidate,
                                        std::vector<Eigen::VectorXd>& solutions) const {
    if (index == end) {
        if (!check_elbow || PassesElbowFilter(candidate)) {
            solutions.push_back(candidate);
        }
        return;
    }

    // The two joints at a singular wrist have already been placed together.
    if (fixed_wrist_pair && IsFixedWristAngle(index)) {
        candidate[index] = analytic[index];
        AddLimitedSolutions(index + 1, end, analytic, fixed_wrist_pair, check_elbow, candidate, solutions);
        return;
    }

    const auto [lower, upper] = IKJointInterval(index);
    if (lower > upper) {
        return;
    }
    const JointWindings windings{analytic[index], lower, upper};
    const double last = windings.Last();
    for (double turn = windings.First(); turn <= last; turn += 1.0) {
        candidate[index] = windings.At(turn);
        AddLimitedSolutions(index + 1, end, analytic, fixed_wrist_pair, check_elbow, candidate, solutions);
        if (turn + 1.0 == turn) {
            break;
        }
    }
}

std::vector<std::pair<double, double>> OPWKinematics::SingularWristSolutions(
        double phase, bool wrist_aligned, const Eigen::Ref<const Eigen::VectorXd>& seed,
        IKSelection selection) const {
    using WristPair = std::pair<double, double>;
    const auto [lower4, upper4] = IKJointInterval(3);
    const auto [lower6, upper6] = IKJointInterval(5);
    if (lower4 > upper4 || lower6 > upper6) {
        return {};
    }

    // In calibrated joint coordinates, joint6 = slope * joint4 + intercept + 2*pi*k.
    const double coupling = wrist_aligned ? 1.0 : -1.0;
    const double direction4 = params_.rotation_directions[3];
    const double direction6 = params_.rotation_directions[5];
    const double slope = -coupling * direction4 * direction6;
    const double intercept = std::remainder(
            direction6 * (phase + params_.offsets[5] + coupling * params_.offsets[3]), kTwoPi);
    const double min_slope4 = std::min(slope * lower4, slope * upper4);
    const double max_slope4 = std::max(slope * lower4, slope * upper4);
    const JointWindings windings{intercept, lower6 - max_slope4, upper6 - min_slope4};
    const double first = windings.First();
    const double last = windings.Last();
    if (first > last) {
        return {};
    }

    const double weight4 = ik_nearst_weight_[3];
    const double weight6 = ik_nearst_weight_[5];
    const auto resolve_winding = [&](double turn) -> std::optional<WristPair> {
        const double shifted = std::fma(turn, kTwoPi, intercept);
        const double bound4_from_lower6 = slope * (lower6 - shifted);
        const double bound4_from_upper6 = slope * (upper6 - shifted);
        const double lower = std::max(lower4, std::min(bound4_from_lower6, bound4_from_upper6));
        const double upper = std::min(upper4, std::max(bound4_from_lower6, bound4_from_upper6));
        if (lower > upper + kJointLimitTolerance) {
            return std::nullopt;
        }

        // With slope ±1, the heavier joint determines the preferred point on the segment.
        const double preferred4 = weight4 >= weight6 ? seed[3] : slope * (seed[5] - shifted);
        const double on_segment = std::clamp(preferred4, lower, std::max(lower, upper));
        const double joint4 = std::clamp(on_segment, lower4, upper4);
        const double joint6 = std::clamp(std::fma(slope, joint4, shifted), lower6, upper6);
        return WristPair{joint4, joint6};
    };

    const bool elbow_depends_on_wrist = elbow_up_only_ && joint_nodes_[3].joint_type == JointType::PRISMATIC;
    if (selection == IKSelection::Nearest && !elbow_depends_on_wrist) {
        // The minimum possible cost is convex in the shifted intercept. Find the
        // leftmost unconstrained minimum, then check its adjacent feasible turns.
        const double preferred_slope4 = weight4 == 0.0 ? max_slope4 :
                slope * std::clamp(seed[3], lower4, upper4);
        const double preferred6 = weight6 == 0.0 ? lower6 : std::clamp(seed[5], lower6, upper6);
        const auto cost = [&](const WristPair& pair) {
            return (weight4 > 0.0 ? weight4 * std::abs(pair.first - seed[3]) : 0.0) +
                   (weight6 > 0.0 ? weight6 * std::abs(pair.second - seed[5]) : 0.0);
        };
        std::optional<WristPair> best;
        for (double turn : windings.Around(preferred6 - preferred_slope4, first, last)) {
            const auto pair = resolve_winding(turn);
            if (pair && (!best || cost(*pair) < cost(*best))) {
                best = pair;
            }
        }
        return best ? std::vector<WristPair>{*best} : std::vector<WristPair>{};
    }

    std::vector<WristPair> solutions;
    for (double turn = first; turn <= last; turn += 1.0) {
        if (const auto pair = resolve_winding(turn)) {
            solutions.push_back(*pair);
        }
        if (turn + 1.0 == turn) {
            break;
        }
    }
    return solutions;
}

}  // namespace opw
