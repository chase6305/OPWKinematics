#include "opw_kinematics/OPWKinematics.h"

#include "Transforms.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>

namespace opw {
namespace {

using detail::kPi;
using detail::kTwoPi;

constexpr double kTrigTolerance = 64.0 * std::numeric_limits<double>::epsilon();
constexpr double kPoseTolerance = 1e-8;

struct IKCandidate {
    Eigen::VectorXd joints;
    bool fixed_wrist_pair;
};

using ArmAngles = std::array<double, 3>;

struct FreeArmBranch {
    ArmAngles angles;
    int joint;
};

// Rz(q1) * Ry(q2 + q3) is shared by the arm and wrist calculations.
Eigen::Matrix3d ArmRotation(double s1, double c1, double s23, double c23) {
    Eigen::Matrix3d rotation;
    rotation << c1 * c23, -s1, c1 * s23,
                s1 * c23,  c1, s1 * s23,
                    -s23, 0.0,      c23;
    return rotation;
}

Eigen::Matrix3d WristInArmFrame(const ArmAngles& arm, const Eigen::Matrix3d& target) {
    const double q23 = arm[1] + arm[2];
    return ArmRotation(std::sin(arm[0]), std::cos(arm[0]),
                       std::sin(q23), std::cos(q23)).transpose() * target;
}

std::optional<double> FreeAngleAtWristSingularity(const FreeArmBranch& branch,
                                                  const Eigen::Vector3d& wrist_axis) {
    // At wrist pitch 0 or pi, the flange axis equals the signed arm Z axis.
    // Recover the free arm angle from that axis without dividing by sin(q5).
    if (branch.joint == 0) {
        const double q23 = branch.angles[1] + branch.angles[2];
        const double s23 = std::sin(q23);
        if (std::abs(s23) <= detail::kWristSingularTolerance ||
            std::abs(wrist_axis.z() - std::cos(q23)) > detail::kWristSingularTolerance) {
            return std::nullopt;
        }
        const double sign = std::copysign(1.0, s23);
        return std::atan2(sign * wrist_axis.y(), sign * wrist_axis.x());
    }
    const double s1 = std::sin(branch.angles[0]);
    const double c1 = std::cos(branch.angles[0]);
    if (std::abs(c1 * wrist_axis.y() - s1 * wrist_axis.x()) > detail::kWristSingularTolerance) {
        return std::nullopt;
    }
    const double radial = c1 * wrist_axis.x() + s1 * wrist_axis.y();
    return std::atan2(radial, wrist_axis.z()) - branch.angles[2];
}

Eigen::Matrix3d WristRotation(double q4, double q5, double q6) {
    const Eigen::Quaterniond wrist = Eigen::AngleAxisd(q4, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(q5, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(q6, Eigen::Vector3d::UnitZ());
    return wrist.toRotationMatrix();
}

SE3d ModelFK(const OPWParameters& params, const Eigen::Ref<const Eigen::VectorXd>& joints) {
    std::array<double, 6> q;
    for (std::size_t i = 0; i < q.size(); ++i) {
        q[i] = joints[i] * params.rotation_directions[i] - params.offsets[i];
    }

    const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
    const double q23 = q[1] + q[2];
    const double s23 = std::sin(q23), c23 = std::cos(q23);
    const double arm_x = params.a1 + params.c2 * std::sin(q[1]) + params.c3 * s23 + params.a2 * c23;
    const double arm_z = params.c1 + params.c2 * std::cos(q[1]) + params.c3 * c23 - params.a2 * s23;
    const Eigen::Vector3d wrist_center(arm_x * c1 - params.b * s1,
                                       arm_x * s1 + params.b * c1, arm_z);

    SE3d pose = SE3d::Identity();
    pose.linear() = ArmRotation(s1, c1, s23, c23) * WristRotation(q[3], q[4], q[5]);
    pose.translation() = wrist_center + params.c4 * pose.linear().col(2);
    return pose;
}

double ElbowBend(double reach, double upper_arm, double forearm, double reach_roundoff) {
    const double scale = std::max({reach, upper_arm, forearm});
    const double tolerance = std::max(kTrigTolerance, reach_roundoff / scale);
    reach /= scale;
    upper_arm /= scale;
    forearm /= scale;

    const double outer = upper_arm + forearm;
    const double inner = std::abs(upper_arm - forearm);
    if (reach > outer + tolerance || reach < inner - tolerance) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (outer - reach <= tolerance) {
        return 0.0;
    }
    if (reach - inner <= tolerance) {
        return kPi;
    }

    // The triangle area gives the sine without subtracting nearly equal cosines.
    const double scaled_sine = std::sqrt((outer + reach) * (outer - reach) *
                                         (reach + inner) * (reach - inner));
    const double scaled_cosine = reach * reach - upper_arm * upper_arm - forearm * forearm;
    return std::atan2(scaled_sine, scaled_cosine);
}

bool IsSamePose(const SE3d& a, const SE3d& b, double length_scale, double tolerance) {
    return a.linear().isApprox(b.linear(), tolerance) &&
           (a.translation() - b.translation()).cwiseAbs().maxCoeff() <=
                   length_scale * tolerance;
}

bool IsSameConfiguration(const IKCandidate& a, const IKCandidate& b) {
    if (a.fixed_wrist_pair != b.fixed_wrist_pair) {
        return false;
    }
    for (Eigen::Index joint = 0; joint < a.joints.size(); ++joint) {
        const double difference = a.joints[joint] - b.joints[joint];
        const bool fixed_angle = a.fixed_wrist_pair && (joint == 3 || joint == 5);
        if (std::abs(fixed_angle ? difference : std::remainder(difference, kTwoPi)) > 1e-9) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool OPWKinematics::GetFK(const Eigen::VectorXd& target_joint, SE3d& pose) const {
    if (target_joint.size() != kDOF || !target_joint.allFinite()) {
        return false;
    }

    const SE3d tcp_pose = base_to_user_.inverse() * params_.T_b_ob * ModelFK(params_, target_joint) *
                          params_.T_oe_e * tcp_;
    if (!tcp_pose.matrix().allFinite()) {
        return false;
    }
    pose = tcp_pose;
    return true;
}

bool OPWKinematics::GetIK(const SE3d& target_pose, IkRtn& solutions,
                          Eigen::VectorXd& joint_seed, std::vector<double>& distances) const {
    return SolveIK(target_pose, solutions, joint_seed, distances, IKSelection::All);
}

bool OPWKinematics::SolveIK(const SE3d& target_pose, IkRtn& solutions,
                            Eigen::VectorXd& joint_seed, std::vector<double>& distances,
                            IKSelection selection) const {
    if (joint_seed.size() != kDOF || !joint_seed.allFinite()) {
        joint_seed = home_joints_;
    }
    // The caller may reuse a previous solution as the next seed.
    const Eigen::Matrix<double, kDOF, 1> seed = joint_seed;
    solutions = IkRtn{};
    distances.clear();
    if (!detail::IsRigidTransform(target_pose)) {
        return false;
    }

    // Undo the user, base, flange, and TCP transforms before solving in OPW coordinates.
    const SE3d opw_pose = params_.T_b_ob.inverse() * base_to_user_ * target_pose *
                         tcp_.inverse() * params_.T_oe_e.inverse();
    if (!opw_pose.matrix().allFinite()) {
        return false;
    }
    const double forearm_length = std::hypot(params_.a2, params_.c3);
    const double length_scale = std::max({std::abs(params_.a1), std::abs(params_.b), std::abs(params_.c1),
                                          params_.c2, forearm_length, std::abs(params_.c4)});
    if (selection == IKSelection::Nearest && (ik_nearst_weight_.array() > 0.0).all() &&
        IsSamePose(opw_pose, ModelFK(params_, seed), length_scale, kTrigTolerance)) {
        // A valid seed that already satisfies the target has the unique minimum cost.
        bool seed_feasible = true;
        for (int joint = 0; joint < kDOF; ++joint) {
            const auto [lower, upper] = IKJointInterval(joint);
            seed_feasible = seed_feasible && seed[joint] >= lower && seed[joint] <= upper;
        }
        if (seed_feasible && PassesElbowFilter(seed)) {
            solutions.ik_joints.emplace_back(seed);
            solutions.ik_number = 1;
            solutions.success = true;
            distances.push_back(0.0);
            return true;
        }
    }

    const Eigen::Vector3d wrist_center = opw_pose.translation() -
                                        params_.c4 * opw_pose.linear().col(2);
    const double radius = std::hypot(wrist_center.x(), wrist_center.y());
    const double lateral = std::abs(params_.b);
    const double elbow_offset = std::atan2(params_.a2, params_.c3);
    const double radial_tolerance = kTrigTolerance * std::max({radius, lateral,
            std::abs(params_.a1), params_.c2, forearm_length});
    if (radius < lateral - radial_tolerance) {
        return false;
    }
    const double z = wrist_center.z() - params_.c1;
    const auto [lower1, upper1] = IKJointInterval(0);
    const auto [lower2, upper2] = IKJointInterval(1);
    if (lower1 > upper1 || lower2 > upper2) {
        return false;
    }
    const auto model_angle = [this](int joint, double external) {
        return external * params_.rotation_directions[joint] - params_.offsets[joint];
    };
    const double free_q1 = model_angle(0, std::clamp(seed[0], lower1, upper1));
    const double free_q2 = model_angle(1, std::clamp(seed[1], lower2, upper2));

    const bool shoulder_singular = radius <= radial_tolerance;
    const bool shoulder_tangent = radius - lateral <= radial_tolerance;
    const Eigen::Vector2d seed_axis(std::cos(free_q1), std::sin(free_q1));
    const double seed_radial = seed_axis.dot(wrist_center.head<2>());
    const double seed_lateral = seed_axis.x() * wrist_center.y() - seed_axis.y() * wrist_center.x();
    // A consistent seed gives the radial component by projection. Near tangency
    // this is better conditioned than recovering it from the two squared lengths.
    const bool use_seed_projection = !shoulder_singular &&
                                     std::abs(seed_lateral - params_.b) <= radial_tolerance;
    const double radial = use_seed_projection ? std::abs(seed_radial) :
            (shoulder_tangent ? 0.0 : std::sqrt(radius - lateral) * std::sqrt(radius + lateral));
    // The derivative of the projected radius with respect to the measured radius
    // is radius / radial. Propagate that roundoff into the elbow reach test.
    const double radial_roundoff = use_seed_projection || shoulder_tangent ? 0.0 :
            (radial_tolerance / 8.0) * (radius / radial);

    std::vector<IKCandidate> analytic_solutions;
    analytic_solutions.reserve(8);
    const auto external_angle = [this](int joint, double angle) {
        return std::remainder((angle + params_.offsets[joint]) * params_.rotation_directions[joint],
                              kTwoPi);
    };
    const auto add_solution = [&](const Eigen::VectorXd& joints, bool fixed_wrist_pair) {
        if (!joints.allFinite() || !IsSamePose(opw_pose, ModelFK(params_, joints), length_scale, kPoseTolerance)) {
            return;
        }
        IKCandidate candidate{joints, fixed_wrist_pair};
        if (std::none_of(analytic_solutions.begin(), analytic_solutions.end(),
                         [&candidate](const IKCandidate& other) {
                             return IsSameConfiguration(candidate, other);
                         })) {
            analytic_solutions.push_back(std::move(candidate));
        }
    };
    const Eigen::Matrix3d target_rotation = opw_pose.linear();
    const auto solve_wrist = [&](const ArmAngles& arm) {
        const Eigen::Matrix3d wrist = WristInArmFrame(arm, target_rotation);
        const double sin_q5 = std::hypot(wrist(0, 2), wrist(1, 2));
        const double q5 = std::atan2(sin_q5, wrist(2, 2));

        // The wrist exposes q4 + q6 near q5 = 0 and q6 - q4 near q5 = pi.
        const bool aligned = wrist(2, 2) >= 0.0;
        const double phase = aligned
                ? std::atan2(wrist(1, 0) - wrist(0, 1), wrist(0, 0) + wrist(1, 1))
                : std::atan2(wrist(1, 0) + wrist(0, 1), wrist(1, 1) - wrist(0, 0));
        Eigen::VectorXd joints(kDOF);
        for (int i = 0; i < static_cast<int>(arm.size()); ++i) {
            joints[i] = external_angle(i, arm[i]);
        }

        if (sin_q5 <= detail::kWristSingularTolerance) {
            joints[4] = external_angle(4, aligned ? 0.0 : kPi);
            for (const auto& [joint4, joint6] : SingularWristSolutions(phase, aligned, seed, selection)) {
                joints[3] = joint4;
                joints[5] = joint6;
                add_solution(joints, true);
            }
            return;
        }

        const double q4 = std::atan2(wrist(1, 2), wrist(0, 2));
        const double q6 = phase + (aligned ? -q4 : q4);
        for (int wrist_flip : {1, -1}) {
            const double flip_offset = wrist_flip == 1 ? 0.0 : kPi;
            joints[3] = external_angle(3, q4 + flip_offset);
            joints[4] = external_angle(4, wrist_flip * q5);
            joints[5] = external_angle(5, q6 - flip_offset);
            add_solution(joints, false);
        }
    };

    std::array<FreeArmBranch, 4> free_branches;
    std::size_t free_branch_count = 0;
    bool fully_folded = false;
    for (int shoulder : {1, -1}) {
        // With the wrist center on the base axis, Q1 is free within its limits.
        double q1 = std::atan2(wrist_center.y(), wrist_center.x()) -
                    std::atan2(params_.b, shoulder * radial);
        if (shoulder_singular) {
            q1 = free_q1 - (shoulder == 1 ? 0.0 : kPi);
        } else if (use_seed_projection && (seed_radial < 0.0) == (shoulder < 0)) {
            q1 = free_q1;
        }
        const double shoulder_to_wrist_x = shoulder * radial - params_.a1;
        const double reach = std::hypot(shoulder_to_wrist_x, z);
        const bool folded = reach <= radial_tolerance;
        if (folded && std::abs(params_.c2 - forearm_length) > radial_tolerance) {
            continue;
        }

        const double elbow_angle = folded ? kPi : ElbowBend(
                reach, params_.c2, forearm_length,
                radial_roundoff * std::abs(shoulder_to_wrist_x / reach));
        if (!std::isfinite(elbow_angle)) {
            continue;
        }
        fully_folded = fully_folded || (folded && shoulder_singular);

        for (int elbow : {1, -1}) {
            const double bend = elbow * elbow_angle;
            const double sin_bend = elbow_angle == kPi ? 0.0 : std::sin(bend);
            const double cos_half_bend = elbow_angle == kPi ? 0.0 : std::cos(bend / 2.0);
            const double arm_z = params_.c2 - forearm_length +
                                 2.0 * forearm_length * cos_half_bend * cos_half_bend;
            const double q2 = folded ? free_q2 : std::atan2(shoulder_to_wrist_x, z) -
                    std::atan2(forearm_length * sin_bend, arm_z);
            const ArmAngles arm{q1, q2, bend - elbow_offset};
            solve_wrist(arm);
            if (folded || (shoulder_singular && shoulder == 1)) {
                free_branches[free_branch_count++] = {arm, folded ? 1 : 0};
            }
        }
    }

    std::vector<Eigen::VectorXd> limited_solutions;
    Eigen::VectorXd candidate(kDOF);
    const bool elbow_uses_windings = ElbowFilterUsesWindings();
    const auto apply_limits = [&] {
        for (const auto& analytic : analytic_solutions) {
            // Full turns leave the sampled point positions unchanged for ordinary revolute joints.
            if (elbow_up_only_ && !elbow_uses_windings && !PassesElbowFilter(analytic.joints)) {
                continue;
            }
            if (selection == IKSelection::All) {
                AddLimitedSolutions(0, kDOF, analytic.joints, analytic.fixed_wrist_pair,
                                    elbow_uses_windings, candidate, limited_solutions);
            } else if (SelectClosestWindings(analytic.joints, analytic.fixed_wrist_pair, seed, candidate)) {
                if (!elbow_uses_windings || PassesElbowFilter(candidate)) {
                    limited_solutions.push_back(candidate);
                } else {
                    // The nearest windings of joints five and six apply to every elbow prefix.
                    AddLimitedSolutions(0, kElbowJointCount, analytic.joints,
                                        analytic.fixed_wrist_pair, true, candidate, limited_solutions);
                }
            }
        }
    };
    // Also test the free-arm angles that make the wrist singular. A nearby seed
    // can produce a valid wrist branch that jumps away from the previous solution.
    for (std::size_t i = 0; i < free_branch_count; ++i) {
        const auto& branch = free_branches[i];
        for (int sign : {1, -1}) {
            const Eigen::Vector3d wrist_axis = sign * target_rotation.col(2);
            if (const auto free_angle = FreeAngleAtWristSingularity(branch, wrist_axis)) {
                ArmAngles arm = branch.angles;
                arm[branch.joint] = *free_angle;
                solve_wrist(arm);
            }
        }
    }
    apply_limits();
    if (limited_solutions.empty() && fully_folded) {
        // With both arm angles free, factor out the preferred feasible wrist
        // first. This recovers both angles when the wrist is fully constrained.
        std::array<double, 3> wrist_angles;
        for (int joint = 3; joint < kDOF; ++joint) {
            const auto [lower, upper] = IKJointInterval(joint);
            if (lower > upper) {
                return false;
            }
            wrist_angles[joint - 3] = model_angle(joint, std::clamp(seed[joint], lower, upper));
        }
        const Eigen::Matrix3d arm_rotation = target_rotation *
                WristRotation(wrist_angles[0], wrist_angles[1], wrist_angles[2]).transpose();
        const double q1 = std::atan2(-arm_rotation(0, 1), arm_rotation(1, 1));
        const double q23 = std::atan2(-arm_rotation(2, 0), arm_rotation(2, 2));
        const double q3 = kPi - elbow_offset;
        analytic_solutions.clear();
        solve_wrist({q1, q23 - q3, q3});
        apply_limits();
    }
    if (limited_solutions.empty() && free_branch_count > 0) {
        // A free arm angle may restore wrist feasibility even when the seed's
        // angle is within its own limits. Search the wrist-limit intervals.
        analytic_solutions.clear();
        constexpr std::array<double, 4> quadrants{0.0, kPi / 2.0, kPi, -kPi / 2.0};
        for (std::size_t i = 0; i < free_branch_count; ++i) {
            ArmAngles arm = free_branches[i].angles;
            const int joint = free_branches[i].joint;
            std::array<Eigen::Matrix3d, 4> wrist_quadrants;
            for (std::size_t quadrant = 0; quadrant < quadrants.size(); ++quadrant) {
                arm[joint] = model_angle(joint, quadrants[quadrant]);
                wrist_quadrants[quadrant] = WristInArmFrame(arm, target_rotation);
            }
            for (double angle : FreeArmRepresentatives(joint, seed[joint], wrist_quadrants)) {
                arm[joint] = model_angle(joint, angle);
                solve_wrist(arm);
            }
        }
        apply_limits();
    }
    if (limited_solutions.empty()) {
        return false;
    }

    const auto movement = [&](const Eigen::VectorXd& solution) {
        double cost = 0.0;
        for (int j = 0; j < kDOF; ++j) {
            if (ik_nearst_weight_[j] > 0.0) {
                cost += ik_nearst_weight_[j] * std::abs(solution[j] - seed[j]);
            }
        }
        return cost;
    };
    if (selection == IKSelection::Nearest) {
        auto best = limited_solutions.begin();
        double best_cost = movement(*best);
        for (auto it = best + 1; it != limited_solutions.end(); ++it) {
            const double cost = movement(*it);
            if (cost < best_cost) {
                best = it;
                best_cost = cost;
            }
        }
        solutions.ik_joints.push_back(std::move(*best));
        distances.push_back(best_cost);
    } else {
        std::vector<double> costs;
        costs.reserve(limited_solutions.size());
        std::transform(limited_solutions.begin(), limited_solutions.end(),
                       std::back_inserter(costs), movement);
        std::vector<std::size_t> order(costs.size());
        std::iota(order.begin(), order.end(), std::size_t{0});
        std::stable_sort(order.begin(), order.end(), [&costs](auto a, auto b) {
            return costs[a] < costs[b];
        });
        solutions.ik_joints.reserve(order.size());
        distances.reserve(order.size());
        for (std::size_t index : order) {
            solutions.ik_joints.push_back(std::move(limited_solutions[index]));
            distances.push_back(costs[index]);
        }
    }
    solutions.ik_number = static_cast<int>(solutions.ik_joints.size());
    solutions.success = true;
    return true;
}

bool OPWKinematics::GetNearestIK(const SE3d& target_pose, IkRtn& solutions,
                                 Eigen::VectorXd& joint_seed, double& min_dist) const {
    std::vector<double> distances;
    if (!SolveIK(target_pose, solutions, joint_seed, distances, IKSelection::Nearest)) {
        min_dist = std::numeric_limits<double>::infinity();
        return false;
    }
    min_dist = distances.front();
    return true;
}

bool OPWKinematics::IsReachable(const SE3d& target_pose) const {
    IkRtn solutions;
    Eigen::VectorXd seed = home_joints_;
    std::vector<double> distances;
    return SolveIK(target_pose, solutions, seed, distances, IKSelection::Nearest);
}

}  // namespace opw
