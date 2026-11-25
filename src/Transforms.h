#pragma once

#include <Eigen/Geometry>

namespace opw::detail {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kJointLimitTolerance = 1e-10;
inline constexpr double kWristSingularTolerance = 1e-12;

inline bool IsRigidTransform(const Eigen::Isometry3d& pose) {
    const Eigen::Matrix3d rotation = pose.linear();
    const Eigen::RowVector4d last_row(0.0, 0.0, 0.0, 1.0);
    return pose.matrix().allFinite() && pose.matrix().row(3).isApprox(last_row) &&
           (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
           rotation.determinant() > 0.0;
}

}  // namespace opw::detail
