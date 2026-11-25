#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace opw {

using SE3d = Eigen::Isometry3d;

struct OPWParameters {
    // Link dimensions in the same length unit as the pose translations.
    double a1 = 0.0;
    double a2 = 0.0;
    double c1 = 0.0;
    double c2 = 0.0;
    double c3 = 0.0;
    double c4 = 0.0;
    double b = 0.0;

    SE3d T_b_ob = SE3d::Identity();  // OPW base frame expressed in the robot base frame.
    SE3d T_oe_e = SE3d::Identity();  // Robot flange frame expressed in the OPW flange frame.

    // Internal OPW angle = joint angle * rotation direction - offset.
    std::array<double, 6> offsets{};
    std::array<signed char, 6> rotation_directions{1, 1, 1, 1, 1, 1};
};

enum class JointType { UNKNOWN, REVOLUTE, PRISMATIC, CONTINUOUS, FIXED };

struct JointNode {
    JointNode();
    JointNode(const SE3d& origin, const Eigen::Vector3d& axis, JointType type,
              double lower, double upper);

    SE3d origin_pose;      // Joint frame expressed in the parent frame at zero travel.
    Eigen::Vector3d axis;  // Motion axis expressed in the joint frame.
    JointType joint_type;
    double lower_limit_set;
    double upper_limit_set;
    double lower_limit;
    double upper_limit;
};

struct IkRtn {
    bool success = false;
    int dof = 6;
    int ik_number = 0;
    std::vector<Eigen::VectorXd> ik_joints;
};

class OPWKinematics {
public:
    enum class FilterMode { NONE = 0, POSITIVE = 1, NEGATIVE = -1 };

    // Throws std::invalid_argument for invalid parameters or joint nodes.
    explicit OPWKinematics(OPWParameters params);
    OPWKinematics(OPWParameters params, const std::vector<JointNode>& joint_nodes);

    bool GetFK(const Eigen::VectorXd& target_joint, SE3d& pose) const;
    // Uses the supplied joint chain and appends its TCP pose to the joint poses.
    bool GetAllFK(const Eigen::VectorXd& target_joint, std::vector<SE3d>& poses) const;
    // Poses are expressed in the user frame. Distances match the sorted solutions.
    // Replaces an invalid seed with the configured home joints.
    bool GetIK(const SE3d& target_pose, IkRtn& solutions,
               Eigen::VectorXd& joint_seed, std::vector<double>& distances) const;
    bool GetNearestIK(const SE3d& target_pose, IkRtn& solutions,
                      Eigen::VectorXd& joint_seed, double& min_dist) const;
    bool GetNearstIK(const SE3d& target_pose, IkRtn& solutions,
                     Eigen::VectorXd& joint_seed, double& min_dist) const {
        return GetNearestIK(target_pose, solutions, joint_seed, min_dist);
    }
    bool IsReachable(const SE3d& target_pose) const;

    int GetDOF() const { return kDOF; }
    bool SetTCP(const SE3d& pose);
    SE3d GetTCP() const { return tcp_; }
    bool SetUserFrame(const SE3d& base_to_user);
    SE3d GetUserFrame() const { return base_to_user_; }
    bool SetHomeJoints(const Eigen::VectorXd& joints);
    Eigen::VectorXd GetHomeJoints() const { return home_joints_; }
    bool SetIkNearstWeight(const Eigen::VectorXd& weights);
    bool GetIkNearstWeight(Eigen::VectorXd& weights) const;

    bool SetJointNode(const std::vector<JointNode>& nodes);
    std::vector<JointNode> GetJointNode() const { return joint_nodes_; }
    // Fixed transform after the last moving joint, used by the joint-chain FK.
    bool SetJointChainFlange(const SE3d& flange_in_last_joint);
    bool SetJointLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower);
    bool GetJointLimits(Eigen::VectorXd& upper, Eigen::VectorXd& lower) const;
    bool SetUserJointLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower);
    bool GetUserJointLimits(Eigen::VectorXd& upper, Eigen::VectorXd& lower) const;

    bool SetJointFilter(std::size_t index, FilterMode mode);
    void SetAllJointFilters(FilterMode mode);
    void ResetJointFilters() { SetAllJointFilters(FilterMode::NONE); }
    FilterMode GetJointFilter(std::size_t index) const;
    bool SetJointAngleThreshold(std::size_t index, double threshold);
    bool SetJointAngleThresholds(const std::vector<double>& thresholds);
    double GetJointAngleThreshold(std::size_t index) const;
    bool IsValidJointAngle(std::size_t index, double angle) const;

    // The upstream elbow-up rule uses the first four joint origins and axes.
    bool SetElbowUp(bool enable);
    bool IsElbowUp() const { return elbow_up_only_; }
    void DisableRobotConfig() { elbow_up_only_ = false; }

private:
    static constexpr int kDOF = 6;
    static constexpr int kElbowJointCount = 4;
    enum class IKSelection { All, Nearest };

    bool HasJointGeometry(std::size_t count) const;
    void JointPoses(const Eigen::Ref<const Eigen::VectorXd>& joints, std::size_t count, SE3d* poses) const;
    std::pair<double, double> IKJointInterval(int index) const;
    bool PassesElbowFilter(const Eigen::Ref<const Eigen::VectorXd>& joints) const;
    bool ElbowFilterUsesWindings() const;
    bool SelectClosestWindings(const Eigen::VectorXd& analytic, bool fixed_wrist_pair,
                              const Eigen::Ref<const Eigen::VectorXd>& seed,
                              Eigen::VectorXd& candidate) const;
    std::vector<std::pair<double, double>> SingularWristSolutions(
            double phase, bool wrist_aligned, const Eigen::Ref<const Eigen::VectorXd>& seed,
            IKSelection selection) const;
    std::vector<double> FreeArmRepresentatives(int joint, double seed,
            const std::array<Eigen::Matrix3d, 4>& wrist_quadrants) const;
    void AddLimitedSolutions(int index, int end, const Eigen::VectorXd& analytic,
                             bool fixed_wrist_pair, bool check_elbow, Eigen::VectorXd& candidate,
                             std::vector<Eigen::VectorXd>& solutions) const;
    bool SolveIK(const SE3d& target_pose, IkRtn& solutions, Eigen::VectorXd& joint_seed,
                 std::vector<double>& distances, IKSelection selection) const;

    OPWParameters params_;
    std::vector<JointNode> joint_nodes_;
    SE3d flange_in_last_joint_ = SE3d::Identity();
    SE3d tcp_ = SE3d::Identity();
    SE3d base_to_user_ = SE3d::Identity();
    Eigen::VectorXd home_joints_ = Eigen::VectorXd::Zero(kDOF);
    Eigen::VectorXd ik_nearst_weight_ = Eigen::VectorXd::Ones(kDOF);
    std::array<FilterMode, kDOF> joint_filters_{};
    std::array<double, kDOF> joint_thresholds_{};
    bool elbow_up_only_ = false;
};

}  // namespace opw
