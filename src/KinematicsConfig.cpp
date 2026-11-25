#include "opw_kinematics/OPWKinematics.h"

#include "Transforms.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace opw {
namespace {

bool IsValidLimit(double lower, double upper) {
    return std::isfinite(lower) && std::isfinite(upper) && lower <= upper;
}

bool IsValidJointNode(const JointNode& node) {
    return detail::IsRigidTransform(node.origin_pose) && node.axis.allFinite() &&
           IsValidLimit(node.lower_limit, node.upper_limit) &&
           IsValidLimit(node.lower_limit_set, node.upper_limit_set);
}

bool HasNodeGeometry(const JointNode& node) {
    switch (node.joint_type) {
        case JointType::FIXED:
            return true;
        case JointType::REVOLUTE:
        case JointType::CONTINUOUS:
        case JointType::PRISMATIC:
            return node.axis.squaredNorm() > 0.0;
        case JointType::UNKNOWN:
            return false;
    }
    return false;
}

bool IsValidParameters(const OPWParameters& params) {
    const std::array geometry{params.a1, params.a2, params.b, params.c1,
                              params.c2, params.c3, params.c4};
    const auto finite = [](double value) { return std::isfinite(value); };
    const double forearm = std::hypot(params.a2, params.c3);
    return std::all_of(geometry.begin(), geometry.end(), finite) &&
           params.c2 > 0.0 && forearm > 0.0 && std::isfinite(forearm) &&
           std::all_of(params.offsets.begin(), params.offsets.end(), finite) &&
           std::all_of(params.rotation_directions.begin(), params.rotation_directions.end(),
                       [](signed char direction) { return direction == 1 || direction == -1; }) &&
           detail::IsRigidTransform(params.T_b_ob) && detail::IsRigidTransform(params.T_oe_e);
}

bool IsValidThreshold(double threshold) {
    return std::isfinite(threshold) && threshold >= 0.0;
}

}  // namespace

JointNode::JointNode()
    : JointNode(SE3d::Identity(), Eigen::Vector3d::Zero(), JointType::UNKNOWN,
                -detail::kPi, detail::kPi) {}

JointNode::JointNode(const SE3d& origin, const Eigen::Vector3d& joint_axis,
                     JointType type, double lower, double upper)
    : origin_pose(origin), axis(joint_axis), joint_type(type),
      lower_limit_set(lower), upper_limit_set(upper), lower_limit(lower), upper_limit(upper) {}

OPWKinematics::OPWKinematics(OPWParameters params)
    : OPWKinematics(std::move(params), std::vector<JointNode>(kDOF)) {}

OPWKinematics::OPWKinematics(OPWParameters params, const std::vector<JointNode>& joint_nodes)
    : params_(std::move(params)) {
    if (!IsValidParameters(params_)) {
        throw std::invalid_argument("Invalid OPW geometry or calibration");
    }
    if (!SetJointNode(joint_nodes)) {
        throw std::invalid_argument("OPW requires six valid joint nodes");
    }
}

bool OPWKinematics::SetTCP(const SE3d& pose) {
    if (!detail::IsRigidTransform(pose)) {
        return false;
    }
    tcp_ = pose;
    return true;
}

bool OPWKinematics::SetUserFrame(const SE3d& base_to_user) {
    if (!detail::IsRigidTransform(base_to_user)) {
        return false;
    }
    base_to_user_ = base_to_user;
    return true;
}

bool OPWKinematics::SetHomeJoints(const Eigen::VectorXd& joints) {
    if (joints.size() != kDOF || !joints.allFinite()) {
        return false;
    }
    home_joints_ = joints;
    return true;
}

bool OPWKinematics::SetIkNearstWeight(const Eigen::VectorXd& weights) {
    if (weights.size() != kDOF || !weights.allFinite() || (weights.array() < 0.0).any()) {
        return false;
    }
    ik_nearst_weight_ = weights;
    return true;
}

bool OPWKinematics::GetIkNearstWeight(Eigen::VectorXd& weights) const {
    weights = ik_nearst_weight_;
    return true;
}

bool OPWKinematics::SetJointNode(const std::vector<JointNode>& nodes) {
    if (nodes.size() != kDOF || !std::all_of(nodes.begin(), nodes.end(), IsValidJointNode)) {
        return false;
    }
    if (elbow_up_only_ && !std::all_of(nodes.begin(), nodes.begin() + kElbowJointCount, HasNodeGeometry)) {
        return false;
    }
    joint_nodes_ = nodes;
    return true;
}

bool OPWKinematics::SetJointChainFlange(const SE3d& flange_in_last_joint) {
    if (!detail::IsRigidTransform(flange_in_last_joint)) {
        return false;
    }
    flange_in_last_joint_ = flange_in_last_joint;
    return true;
}

bool OPWKinematics::SetJointLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    if (upper.size() != kDOF || lower.size() != kDOF || !upper.allFinite() ||
        !lower.allFinite() || (lower.array() > upper.array()).any()) {
        return false;
    }
    for (int i = 0; i < kDOF; ++i) {
        joint_nodes_[i].lower_limit = lower[i];
        joint_nodes_[i].upper_limit = upper[i];
    }
    return true;
}

bool OPWKinematics::GetJointLimits(Eigen::VectorXd& upper, Eigen::VectorXd& lower) const {
    upper.resize(kDOF);
    lower.resize(kDOF);
    for (int i = 0; i < kDOF; ++i) {
        upper[i] = joint_nodes_[i].upper_limit;
        lower[i] = joint_nodes_[i].lower_limit;
    }
    return true;
}

bool OPWKinematics::SetUserJointLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    if (upper.size() != kDOF || lower.size() != kDOF || !upper.allFinite() ||
        !lower.allFinite() || (lower.array() > upper.array()).any()) {
        return false;
    }
    for (int i = 0; i < kDOF; ++i) {
        joint_nodes_[i].lower_limit_set = lower[i];
        joint_nodes_[i].upper_limit_set = upper[i];
    }
    return true;
}

bool OPWKinematics::GetUserJointLimits(Eigen::VectorXd& upper, Eigen::VectorXd& lower) const {
    upper.resize(kDOF);
    lower.resize(kDOF);
    for (int i = 0; i < kDOF; ++i) {
        upper[i] = joint_nodes_[i].upper_limit_set;
        lower[i] = joint_nodes_[i].lower_limit_set;
    }
    return true;
}

bool OPWKinematics::SetJointFilter(std::size_t index, FilterMode mode) {
    if (index >= kDOF) {
        return false;
    }
    joint_filters_[index] = mode;
    return true;
}

void OPWKinematics::SetAllJointFilters(FilterMode mode) {
    joint_filters_.fill(mode);
}

OPWKinematics::FilterMode OPWKinematics::GetJointFilter(std::size_t index) const {
    return index < kDOF ? joint_filters_[index] : FilterMode::NONE;
}

bool OPWKinematics::SetJointAngleThreshold(std::size_t index, double threshold) {
    if (index >= kDOF || !IsValidThreshold(threshold)) {
        return false;
    }
    joint_thresholds_[index] = threshold;
    return true;
}

bool OPWKinematics::SetJointAngleThresholds(const std::vector<double>& thresholds) {
    if (thresholds.size() != kDOF ||
        !std::all_of(thresholds.begin(), thresholds.end(), IsValidThreshold)) {
        return false;
    }
    std::copy(thresholds.begin(), thresholds.end(), joint_thresholds_.begin());
    return true;
}

double OPWKinematics::GetJointAngleThreshold(std::size_t index) const {
    return index < kDOF ? joint_thresholds_[index] : 0.0;
}

bool OPWKinematics::IsValidJointAngle(std::size_t index, double angle) const {
    switch (GetJointFilter(index)) {
        case FilterMode::POSITIVE:
            return angle >= joint_thresholds_[index];
        case FilterMode::NEGATIVE:
            return angle <= -joint_thresholds_[index];
        case FilterMode::NONE:
            return true;
    }
    return false;
}

bool OPWKinematics::HasJointGeometry(std::size_t count) const {
    return std::all_of(joint_nodes_.begin(), joint_nodes_.begin() + count, HasNodeGeometry);
}

bool OPWKinematics::SetElbowUp(bool enable) {
    if (enable && !HasJointGeometry(kElbowJointCount)) {
        return false;
    }
    elbow_up_only_ = enable;
    return true;
}

void OPWKinematics::JointPoses(const Eigen::Ref<const Eigen::VectorXd>& joints,
                              std::size_t count, SE3d* poses) const {
    SE3d pose = SE3d::Identity();
    for (std::size_t i = 0; i < count; ++i) {
        const auto& node = joint_nodes_[i];
        pose = pose * node.origin_pose;
        if (node.joint_type == JointType::REVOLUTE || node.joint_type == JointType::CONTINUOUS) {
            const double axis_length = node.axis.norm();
            pose.rotate(Eigen::AngleAxisd(joints[i] * axis_length, node.axis / axis_length));
        } else if (node.joint_type == JointType::PRISMATIC) {
            pose.translate(node.axis * joints[i]);
        }
        poses[i] = pose;
    }
}

bool OPWKinematics::GetAllFK(const Eigen::VectorXd& target_joint, std::vector<SE3d>& poses) const {
    poses.clear();
    if (target_joint.size() != kDOF || !target_joint.allFinite() || !HasJointGeometry(kDOF)) {
        return false;
    }
    poses.resize(kDOF + 1);
    JointPoses(target_joint, kDOF, poses.data());
    poses.back() = poses[kDOF - 1] * flange_in_last_joint_ * tcp_;
    const SE3d user_to_base = base_to_user_.inverse();
    for (auto& pose : poses) {
        pose = user_to_base * pose;
        if (!pose.matrix().allFinite()) {
            poses.clear();
            return false;
        }
    }
    return true;
}

bool OPWKinematics::PassesElbowFilter(const Eigen::Ref<const Eigen::VectorXd>& joints) const {
    if (!elbow_up_only_) {
        return true;
    }
    std::array<SE3d, kElbowJointCount> poses;
    JointPoses(joints, poses.size(), poses.data());
    const double shoulder_z = poses[1].translation().z();
    const double elbow_z = poses[2].translation().z();
    const double wrist_z = poses[3].translation().z();
    const double height_scale = std::max({std::abs(shoulder_z), std::abs(elbow_z), std::abs(wrist_z)});
    const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() * height_scale;
    return !(shoulder_z - elbow_z > tolerance && wrist_z - elbow_z > tolerance);
}

bool OPWKinematics::ElbowFilterUsesWindings() const {
    if (!elbow_up_only_) {
        return false;
    }
    for (int joint = 0; joint < kElbowJointCount; ++joint) {
        const auto& node = joint_nodes_[joint];
        if (node.joint_type == JointType::PRISMATIC) {
            return true;
        }
        // A rotation at the fourth joint cannot move any of the sampled points.
        if (joint + 1 < kElbowJointCount && node.joint_type != JointType::FIXED && node.axis.norm() != 1.0) {
            return true;
        }
    }
    return false;
}

}  // namespace opw
