#include <opw_kinematics/OPWKinematics.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kPi = 3.14159265358979323846;

void Check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

opw::OPWParameters MakeParameters() {
    opw::OPWParameters params;
    params.a1 = 0.1;
    params.a2 = -0.08;
    params.b = -0.02;
    params.c1 = 0.35;
    params.c2 = 0.45;
    params.c3 = 0.30;
    params.c4 = 0.07;
    return params;
}

opw::SE3d Pose(const Eigen::Vector3d& translation, double angle,
               const Eigen::Vector3d& axis) {
    opw::SE3d pose = opw::SE3d::Identity();
    pose.translate(translation);
    pose.rotate(Eigen::AngleAxisd(angle, axis));
    return pose;
}

Eigen::VectorXd ExternalAngles(const opw::OPWParameters& params,
                               const Eigen::VectorXd& model_angles) {
    Eigen::VectorXd joints(6);
    for (int i = 0; i < 6; ++i) {
        joints[i] = (model_angles[i] + params.offsets[i]) * params.rotation_directions[i];
    }
    return joints;
}

void CheckCalibratedBranches() {
    const auto geometry = MakeParameters();
    const opw::OPWKinematics model(geometry);
    opw::SE3d pose;
    Check(model.GetFK(Eigen::VectorXd::Zero(6), pose), "home FK failed");
    Check(pose.linear().isIdentity(1e-12) &&
                  pose.translation().isApprox(Eigen::Vector3d(0.02, -0.02, 1.17), 1e-12),
          "home FK differs from the geometric position");

    auto params = geometry;
    params.offsets = {0.25, -0.10, 0.20, -0.35, -0.25, 0.20};
    params.rotation_directions = {-1, 1, -1, -1, -1, 1};
    params.T_b_ob = Pose({0.17, -0.20, 0.13}, 0.42, Eigen::Vector3d::UnitX());
    params.T_oe_e = Pose({-0.11, 0.24, 0.08}, -0.55, Eigen::Vector3d::UnitY());
    const auto user_frame = Pose({0.01, 0.24, -0.32}, 0.39, Eigen::Vector3d::UnitZ());
    const auto tcp = Pose({0.08, -0.01, 0.12}, 0.21, Eigen::Vector3d::UnitX());
    opw::OPWKinematics solver(params);
    Check(solver.SetTCP(tcp) && solver.SetUserFrame(user_frame), "frame setup failed");
    Eigen::VectorXd weights(6);
    weights << 1., 2., 3., 0.5, 0.25, 0.0;
    Check(solver.SetIkNearstWeight(weights), "weight setup failed");

    Eigen::VectorXd model_angles(6);
    model_angles << 0.61, -0.80, 1.20, 0.40, -0.60, -0.70;
    Eigen::VectorXd joints = ExternalAngles(params, model_angles);
    opw::SE3d model_pose;
    Check(model.GetFK(model_angles, model_pose) && solver.GetFK(joints, pose), "calibrated FK failed");
    const opw::SE3d expected = user_frame.inverse() * params.T_b_ob * model_pose * params.T_oe_e * tcp;
    Check(pose.matrix().isApprox(expected.matrix(), 1e-12), "incorrect calibration or frame order");

    opw::IkRtn result;
    std::vector<double> distances;
    Check(solver.GetIK(pose, result, joints, distances), "calibrated IK failed");
    Check(result.ik_number == 8 && distances.size() == 8, "expected both shoulders, elbows and wrists");
    Check(distances.front() < 1e-10 && std::is_sorted(distances.begin(), distances.end()),
          "IK distances are out of order");
    for (std::size_t i = 0; i < result.ik_joints.size(); ++i) {
        const auto& solution = result.ik_joints[i];
        opw::SE3d candidate;
        Check(solver.GetFK(solution, candidate) && candidate.matrix().isApprox(pose.matrix(), 1e-9),
              "a branch does not reproduce the target pose");
        const double cost = weights.dot((solution - joints).cwiseAbs());
        Check(std::abs(cost - distances[i]) < 1e-12, "weights do not match the sorted solution");
    }

    for (double wrist : {0.0, kPi, 1e-9, kPi - 1e-9}) {
        // Both exact wrist singularities occur on the second elbow branch.
        model_angles << 0.61, 0.35, -0.90, 0.54, wrist, 0.76;
        joints = ExternalAngles(params, model_angles);
        Check(solver.GetFK(joints, pose) && solver.GetIK(pose, result, joints, distances),
              "wrist singularity IK failed");
        Check(distances.front() < 1e-5, "wrist singularity lost the seeded branch");
        if (wrist == 0.0 || wrist == kPi) {
            Check(std::abs(result.ik_joints.front()[3] - joints[3]) < 1e-10,
                  "singular wrist failed to preserve the calibrated joint-four seed");
        }
    }
}

void CheckLimitsAndFailures() {
    opw::OPWKinematics solver(MakeParameters());
    Eigen::VectorXd joints(6);
    joints << 0.3, -0.4, 0.6, 0.2, -0.5, 0.7 + 2.0 * kPi;
    opw::SE3d pose;
    Check(solver.GetFK(joints, pose), "multi-turn FK failed");
    const Eigen::VectorXd lower = joints.array() - 0.01;
    const Eigen::VectorXd upper = joints.array() + 0.01;
    Check(solver.SetJointLimits(upper, lower) && solver.SetUserJointLimits(upper, lower),
          "joint limit setup failed");
    Check(!solver.SetJointLimits(lower, upper), "invalid limits were accepted");

    Eigen::VectorXd seed = joints;
    seed[0] += 0.25;
    Check(solver.SetHomeJoints(seed), "home joints setup failed");
    seed.resize(0);
    opw::IkRtn result;
    double distance = -1.0;
    Check(solver.GetNearstIK(pose, result, seed, distance), "nearest IK did not recover the multi-turn solution");
    Check(result.ik_number == 1 && result.ik_joints.size() == 1 &&
                  result.ik_joints.front().isApprox(joints, 1e-10) && std::abs(distance - 0.25) < 1e-10,
          "incorrect result or distance for the only valid IK solution");

    Check(solver.SetJointFilter(5, opw::OPWKinematics::FilterMode::NEGATIVE), "filter setup failed");
    Check(!solver.GetNearestIK(pose, result, seed, distance) && !result.success &&
                  result.ik_number == 0 && result.ik_joints.empty() && std::isinf(distance),
          "a rejected target left valid-looking outputs");
    seed = joints;
    Check(!solver.GetNearestIK(pose, result, seed, distance), "an exact seed bypassed a joint filter");
    solver.ResetJointFilters();
    pose.translation().x() = 100.0;
    Check(!solver.IsReachable(pose), "an unreachable position was accepted");
    Check(!solver.GetFK(Eigen::VectorXd::Zero(5), pose), "FK accepted five joints");
}

void CheckShoulderAndFoldedElbow() {
    opw::OPWParameters params;
    params.c1 = 0.3;
    params.c2 = params.c3 = 0.45;
    params.c4 = 0.05;
    params.offsets = {0.2, -0.1, 0.0, 0.3, 0.0, 0.0};
    params.rotation_directions = {-1, -1, 1, 1, 1, 1};
    const opw::OPWKinematics solver(params);

    for (double elbow : {-0.7, kPi}) {
        Eigen::VectorXd model_angles(6);
        model_angles << 0.5, 0.35, elbow, 0.8, -0.4, -0.7;
        Eigen::VectorXd seed = ExternalAngles(params, model_angles);
        opw::SE3d pose;
        opw::IkRtn result;
        double distance;
        Check(solver.GetFK(seed, pose) && solver.GetNearestIK(pose, result, seed, distance) && distance < 1e-9,
              "cannot recover the seeded shoulder or folded-elbow singularity");
    }
}

void CheckSingularWristWithinLimits() {
    auto params = MakeParameters();
    params.offsets = {0.0, 0.0, 0.0, 0.25, -0.20, -0.30};
    params.rotation_directions = {1, 1, 1, -1, -1, 1};

    for (double wrist : {0.0, kPi}) {
        opw::OPWKinematics solver(params);
        Eigen::VectorXd joints(6);
        joints << 0.3, -0.4, 0.6, 0.6, -(wrist + params.offsets[4]), 0.7 + 2.0 * kPi;
        opw::SE3d pose;
        Check(solver.GetFK(joints, pose), "singular wrist FK failed");

        Eigen::VectorXd lower = joints.array() - 0.01;
        Eigen::VectorXd upper = joints.array() + 0.01;
        lower[3] = 0.45;
        upper[3] = 0.75;
        lower[5] = joints[5] - 0.2;
        upper[5] = joints[5] + 0.2;
        Check(solver.SetJointLimits(upper, lower) && solver.SetUserJointLimits(upper, lower),
              "singular wrist limit setup failed");
        Check(solver.SetJointFilter(3, opw::OPWKinematics::FilterMode::POSITIVE) &&
                      solver.SetJointAngleThreshold(3, 0.65), "singular wrist filter setup failed");

        Eigen::VectorXd seed = joints;
        seed[3] = 0.0;
        seed[5] += wrist == 0.0 ? 0.1 : -0.1;
        Eigen::VectorXd weights = Eigen::VectorXd::Ones(6);
        weights[5] = 3.0;
        Check(solver.SetIkNearstWeight(weights), "singular wrist weight setup failed");

        opw::IkRtn result;
        double distance;
        Check(solver.GetNearestIK(pose, result, seed, distance),
              "an out-of-range seed hid the admissible singular wrist family");
        Check(std::abs(result.ik_joints.front()[3] - 0.7) < 1e-10 &&
                      std::abs(result.ik_joints.front()[5] - seed[5]) < 1e-10 &&
                      std::abs(distance - 0.7) < 1e-10,
              "singular wrist representative did not minimize the weighted movement");

        solver.ResetJointFilters();
        lower[3] = -4.0 * kPi;
        lower[5] = -3.5 * kPi;
        upper[3] = 3.7 * kPi;
        upper[5] = 4.0 * kPi;
        Check(solver.SetJointLimits(upper, lower) && solver.SetUserJointLimits(upper, lower),
              "wide singular wrist limits setup failed");
        seed[3] = 14.0;
        seed[5] = 7.9;
        const std::array<std::pair<double, double>, 5> weight_pairs{{
                {1.0, 3.0}, {3.0, 1.0}, {0.0, 1.0}, {1.0, 0.0}, {0.0, 0.0},
        }};
        for (const auto& [weight4, weight6] : weight_pairs) {
            weights[3] = weight4;
            weights[5] = weight6;
            Check(solver.SetIkNearstWeight(weights), "wide singular wrist weight setup failed");
            opw::IkRtn all_solutions;
            std::vector<double> distances;
            Check(solver.GetIK(pose, all_solutions, seed, distances) && distances.size() > 1 &&
                          solver.GetNearestIK(pose, result, seed, distance) &&
                          std::abs(distance - distances.front()) < 1e-10 &&
                          result.ik_joints.front().isApprox(all_solutions.ik_joints.front(), 1e-10),
                  "nearest coupled wrist winding disagrees with exhaustive sorting");
        }
    }
}

void CheckCombinedSingularities() {
    auto params = MakeParameters();
    params.a1 = 0.14;
    params.a2 = -0.09;
    params.b = 0.08;
    params.c1 = 0.32;
    params.c2 = 0.5;
    params.c3 = 0.29;
    params.c4 = 0.08;
    const opw::OPWKinematics solver(params);
    const double forearm = std::hypot(params.a2, params.c3);
    const double forearm_angle = std::atan2(params.a2, params.c3);

    const double tangent_elbow = 0.71;
    const double arm_z = params.c2 + forearm * std::cos(tangent_elbow + forearm_angle);
    const double arm_x = forearm * std::sin(tangent_elbow + forearm_angle);
    const double tangent_shoulder = std::asin(-params.a1 / std::hypot(arm_x, arm_z)) -
                                    std::atan2(arm_x, arm_z);

    Eigen::VectorXd extension(6), folded(6), tangent(6);
    extension << 1.805374093564803, 0.17956416397876662, -forearm_angle, 2.362678753823491,
                 0.0, -2.505763846487609;
    folded << 0.5071662142366451, -0.7925647695793563, -kPi - forearm_angle,
              1.4313691938995832, 0.0, 0.7990288197294371;
    tangent << 1.65, tangent_shoulder, tangent_elbow, 0.6, 0.0, -0.4;
    for (const auto& joints : {extension, folded, tangent}) {
        opw::SE3d pose;
        opw::IkRtn result;
        Eigen::VectorXd seed = joints;
        seed[3] += 0.02;
        seed[5] -= 0.02;
        seed[4] += 0.001;
        const double base_step = joints[2] < -1.0 ? 0.01 : 0.0;
        seed[0] += base_step;
        double distance;
        Check(solver.GetFK(joints, pose) && solver.GetNearestIK(pose, result, seed, distance) &&
                      std::abs(distance - (base_step + 0.001)) < 1e-10,
              "roundoff at an arm degeneracy displaced the singular wrist solution");
    }
}

void CheckFreeArmJointWithinLimits() {
    for (int free_joint : {0, 1}) {
        opw::OPWParameters params;
        params.c1 = 0.35;
        params.c2 = 0.45;
        params.c3 = free_joint == 0 ? 0.3 : params.c2;
        params.c4 = 0.08;
        params.a1 = free_joint == 0 ? 0.0 : 0.15;
        params.b = free_joint == 0 ? 0.0 : 0.08;
        opw::OPWKinematics solver(params);

        Eigen::VectorXd joints(6);
        const double aligned_shoulder = std::atan2(-params.c3 * std::sin(-0.8),
                                                   params.c2 + params.c3 * std::cos(-0.8));
        joints << 0.55, free_joint == 0 ? aligned_shoulder : 0.55,
                  free_joint == 0 ? -0.8 : kPi, 0.40, 0.50, -0.65;
        opw::SE3d pose;
        Check(solver.GetFK(joints, pose), "free-arm FK failed");
        Eigen::VectorXd lower = Eigen::VectorXd::Constant(6, -kPi);
        Eigen::VectorXd upper = Eigen::VectorXd::Constant(6, kPi);
        lower[free_joint] = 0.45;
        upper[free_joint] = 0.65;
        Check(solver.SetJointLimits(upper, lower), "free-arm limits setup failed");

        Eigen::VectorXd seed = joints;
        seed[free_joint] = 0.0;
        opw::IkRtn result;
        double distance;
        Check(solver.GetNearestIK(pose, result, seed, distance), "an out-of-range free arm seed hid an IK solution");
        const Eigen::VectorXd& solution = result.ik_joints.front();
        opw::SE3d reproduced;
        Check(solution[free_joint] >= lower[free_joint] && solution[free_joint] <= upper[free_joint] &&
                      solver.GetFK(solution, reproduced) && reproduced.matrix().isApprox(pose.matrix(), 1e-9),
              "the free arm representative fails the limit or pose check");
    }
}

void CheckNearestWindingSelection() {
    std::vector<opw::JointNode> nodes(6);
    for (auto& node : nodes) {
        node.joint_type = opw::JointType::FIXED;
    }
    nodes[2].joint_type = opw::JointType::PRISMATIC;
    nodes[2].axis = Eigen::Vector3d::UnitZ();
    nodes[3].origin_pose.translation().z() = 0.2;
    opw::OPWKinematics solver(MakeParameters(), nodes);

    Eigen::VectorXd joints(6);
    joints << 0.3, -0.4, -0.45, 0.2, -0.5, 0.7;
    opw::SE3d target;
    Check(solver.GetFK(joints, target), "nearest winding FK failed");
    Eigen::VectorXd lower = joints.array() - 0.01;
    Eigen::VectorXd upper = joints.array() + 0.01;
    upper[2] += 2.0 * kPi;
    lower[4] -= 2.0 * kPi;
    upper[4] += 2.0 * kPi;
    lower[5] -= 2.0 * kPi;
    upper[5] += 2.0 * kPi;
    Check(solver.SetJointLimits(upper, lower) && solver.SetUserJointLimits(upper, lower),
          "nearest winding limits setup failed");
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(6);
    weights[4] = 0.0;
    Check(solver.SetIkNearstWeight(weights), "nearest winding weight setup failed");

    for (bool elbow_filter : {false, true}) {
        Check(solver.SetElbowUp(elbow_filter), "elbow filter setup failed");
        Eigen::VectorXd seed = joints;
        seed[0] += 0.03;
        seed[5] += 2.0 * kPi + 0.03;
        opw::IkRtn all_solutions, nearest;
        std::vector<double> distances;
        double min_dist;
        Check(solver.GetIK(target, all_solutions, seed, distances) && distances.size() > 1 &&
                      solver.GetNearestIK(target, nearest, seed, min_dist) && solver.IsReachable(target),
              "nearest winding solve failed");
        Check(nearest.ik_joints.front().isApprox(all_solutions.ik_joints.front(), 1e-12) &&
                      std::abs(min_dist - distances.front()) < 1e-12,
              "nearest winding search disagrees with exhaustive sorting");
        Check((nearest.ik_joints.front()[2] > 0.0) == elbow_filter,
              "the elbow filter did not select the allowed winding");

        // Reuse the result entry as input. It may be invalidated when the result is replaced.
        Check(solver.GetNearestIK(target, nearest, nearest.ik_joints.front(), min_dist) && min_dist < 1e-12,
              "reusing a previous result entry as the seed changed the solution");
    }

    nodes[2].joint_type = opw::JointType::REVOLUTE;
    nodes[2].origin_pose.translation().z() = -0.2;
    Check(solver.SetJointNode(nodes), "periodic elbow model setup failed");
    Eigen::VectorXd seed = joints;
    opw::IkRtn result;
    double distance;
    std::vector<double> distances;
    Check(!solver.GetIK(target, result, seed, distances) &&
                  !solver.GetNearestIK(target, result, seed, distance) && !solver.IsReachable(target),
          "full turns incorrectly bypassed the revolute elbow filter");
}

void CheckLengthUnits() {
    const opw::OPWParameters reference_params = MakeParameters();
    Eigen::VectorXd joints(6);
    joints << 0.5, -0.65, 0.83, 0.6, -0.35, 0.1;
    opw::SE3d reference;
    Check(opw::OPWKinematics(reference_params).GetFK(joints, reference), "reference FK failed");

    for (double scale : {1e-6, 1e6}) {
        auto params = reference_params;
        params.a1 *= scale;
        params.a2 *= scale;
        params.b *= scale;
        params.c1 *= scale;
        params.c2 *= scale;
        params.c3 *= scale;
        params.c4 *= scale;
        params.T_b_ob = Pose(scale * Eigen::Vector3d(0.2, -0.7, 0.45), 0.6, Eigen::Vector3d::UnitY());
        params.T_oe_e = Pose(scale * Eigen::Vector3d(0.5, 0.2, -0.2), -0.4, Eigen::Vector3d::UnitX());
        opw::OPWKinematics solver(params);
        opw::SE3d scaled = reference;
        scaled.translation() *= scale;
        const opw::SE3d target = params.T_b_ob * scaled * params.T_oe_e;
        opw::SE3d actual;
        Check(solver.GetFK(joints, actual) && actual.linear().isApprox(target.linear(), 1e-12) &&
                      ((actual.translation() - target.translation()) / scale).norm() < 1e-12,
              "FK changed when expressed in different length units");

        Eigen::VectorXd seed = joints;
        seed[2] += 0.04;
        opw::IkRtn solution;
        double distance;
        Check(solver.GetNearestIK(target, solution, seed, distance) &&
                      solution.ik_joints.front().isApprox(joints, 1e-10),
              "IK changed when expressed in different length units");
    }
}

void CheckWristLimitsAtFreeArmAngles() {
    opw::OPWParameters params;
    params.c1 = 0.35;
    params.c2 = 0.5;
    params.c3 = 0.3;
    params.c4 = 0.08;
    opw::OPWKinematics shoulder(params);
    Eigen::VectorXd upper(6);
    upper << 2.618, 3.1, 3.1, 1.745, 1.22, 2.094;
    Check(shoulder.SetJointLimits(upper, -upper), "free base wrist limits setup failed");
    const double elbow = 1.2;
    const double arm = std::atan2(-params.c3 * std::sin(elbow), params.c2 + params.c3 * std::cos(elbow));
    Eigen::VectorXd joints(6), seed = Eigen::VectorXd::Zero(6);
    joints << -1.9, arm, elbow, 0.7, 0.8, -1.8;
    opw::SE3d target, check;
    opw::IkRtn solution;
    double distance;
    Check(shoulder.GetFK(joints, target) && shoulder.GetNearestIK(target, solution, seed, distance) &&
                  shoulder.GetFK(solution.ik_joints.front(), check) && check.matrix().isApprox(target.matrix(), 1e-10),
          "wrist limits hid an admissible free base angle");
    std::vector<double> distances;
    Check(shoulder.GetIK(target, solution, seed, distances) && shoulder.IsReachable(target),
          "free base fallback did not reach all IK entry points");

    joints << 1.9, arm, elbow, 0.85, 0.0, -1.2;
    seed = joints;
    seed[0] -= 0.002;
    seed[1] -= 0.001;
    seed[4] -= 0.001;
    Check(shoulder.GetFK(joints, target) && shoulder.GetNearestIK(target, solution, seed, distance) &&
                  distance < 0.02 && shoulder.GetFK(solution.ik_joints.front(), check) &&
                  check.matrix().isApprox(target.matrix(), 1e-10),
          "the nearest IK branch jumps when the base and wrist become singular together");
    seed.setZero();

    for (double wrist : {0.0, kPi}) {
        joints << 0.9, 0.0, 0.0, 0.5, wrist, 0.3;
        Eigen::VectorXd lower = joints;
        upper = joints;
        lower[0] = -2.0;
        upper[0] = 2.0;
        Check(shoulder.SetJointLimits(upper, lower) && shoulder.GetFK(joints, target) &&
                      shoulder.GetNearestIK(target, solution, seed, distance) &&
                      solution.ik_joints.front().isApprox(joints, 1e-10),
              "free base fallback did not preserve the locked singular wrist phase");
    }

    params.a1 = 0.15;
    params.b = 0.08;
    params.c3 = params.c2;
    opw::OPWKinematics folded(params);
    joints << 0.3, 0.9, kPi, 0.5, 0.55, -0.4;
    Eigen::VectorXd lower = joints.array() - 0.01;
    upper = joints.array() + 0.01;
    lower[1] = -2.8;
    upper[1] = 2.8;
    Check(folded.SetJointLimits(upper, lower) && folded.GetFK(joints, target) &&
                  folded.GetNearestIK(target, solution, seed, distance) &&
                  folded.GetFK(solution.ik_joints.front(), check) && check.matrix().isApprox(target.matrix(), 1e-10),
          "wrist limits hid an admissible free shoulder angle at the folded elbow");

    params.a1 = 0.0;
    params.b = 0.0;
    params.offsets = {0.2, -0.1, 0.3, -0.2, 0.25, -0.5};
    params.rotation_directions = {-1, 1, -1, 1, -1, 1};
    opw::OPWKinematics fully_folded(params);
    Eigen::VectorXd model_joints(6);
    model_joints << 0.73, 0.95, kPi, 0.6, 0.65, -0.42;
    joints = ExternalAngles(params, model_joints);
    lower = joints;
    upper = joints;
    lower.head(2).setConstant(-2.4);
    upper.head(2).setConstant(2.4);
    Check(fully_folded.SetJointLimits(upper, lower) && fully_folded.SetUserJointLimits(upper, lower) &&
                  fully_folded.GetFK(joints, target) && fully_folded.GetNearestIK(target, solution, seed, distance) &&
                  solution.ik_joints.front().isApprox(joints, 1e-10),
          "fully folded arm failed to recover both free angles from a locked wrist");
}

}  // namespace

int main() {
    try {
        CheckCalibratedBranches();
        CheckLimitsAndFailures();
        CheckShoulderAndFoldedElbow();
        CheckSingularWristWithinLimits();
        CheckCombinedSingularities();
        CheckFreeArmJointWithinLimits();
        CheckNearestWindingSelection();
        CheckLengthUnits();
        CheckWristLimitsAtFreeArmAngles();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
