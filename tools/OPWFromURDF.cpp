#include <opw_kinematics/OPWKinematics.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using opw::SE3d;
using Eigen::Vector3d;
constexpr double kAngularTolerance = 1e-5;
constexpr double kLinearTolerance = 1e-5;
constexpr std::size_t kJointCount = 6;
using Angles = std::array<double, kJointCount>;
using Path = std::vector<urdf::JointConstSharedPtr>;

struct ChainJoint {
    urdf::JointConstSharedPtr urdf_joint;
    SE3d origin;
    Vector3d axis;
};

struct Chain {
    std::string base;
    std::string tip;
    std::array<ChainJoint, kJointCount> joints;
    SE3d tip_in_joint6;
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool IsArmJoint(const urdf::Joint& joint) {
    return !joint.mimic && (joint.type == urdf::Joint::REVOLUTE ||
                            joint.type == urdf::Joint::CONTINUOUS);
}

Path FindPath(const urdf::ModelInterface& model, const std::string& base,
              urdf::LinkConstSharedPtr link) {
    Path path;
    std::unordered_set<std::string> visited;
    while (link && link->name != base) {
        Require(visited.insert(link->name).second, "cycle in URDF chain at " + link->name);
        if (!link->parent_joint) {
            return {};
        }
        path.push_back(link->parent_joint);
        link = model.getLink(link->parent_joint->parent_link_name);
    }
    if (!link) {
        return {};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool IsArmPath(const Path& path) {
    std::size_t count = 0;
    for (const auto& joint : path) {
        if (joint->type == urdf::Joint::FIXED) {
            continue;
        }
        if (!IsArmJoint(*joint)) {
            return false;
        }
        ++count;
    }
    return count == kJointCount;
}

std::string FindTip(const urdf::ModelInterface& model, const std::string& base) {
    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    std::vector<std::string> tips;
    // Follow fixed adapters to their terminal links. Moving gripper branches
    // do not prevent the arm's flange from being a candidate.
    for (const auto& link : links) {
        const bool has_fixed_child = std::any_of(link->child_joints.begin(), link->child_joints.end(),
                [](const auto& joint) { return joint->type == urdf::Joint::FIXED; });
        if (!has_fixed_child && IsArmPath(FindPath(model, base, link))) {
            tips.push_back(link->name);
        }
    }
    Require(!tips.empty(), "no six-axis arm chain found below " + base + "; check --base and --tip");
    if (tips.size() > 1) {
        std::string message = "multiple six-axis tips below " + base + "; select with --tip:";
        for (const auto& tip : tips) {
            message += " " + tip;
        }
        throw std::runtime_error(message);
    }
    return tips.front();
}

SE3d ToIsometry(const urdf::Pose& pose, const std::string& joint) {
    const Vector3d xyz(pose.position.x, pose.position.y, pose.position.z);
    const auto& r = pose.rotation;
    const Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);
    Require(xyz.allFinite() && rotation.coeffs().allFinite() && rotation.norm() > 0.0,
            "invalid origin for joint " + joint);
    SE3d result = SE3d::Identity();
    result.translation() = xyz;
    result.linear() = rotation.normalized().toRotationMatrix();
    return result;
}

Chain ReadChain(const urdf::ModelInterface& model, const std::string& base,
                const std::string& tip) {
    Require(model.getLink(base) != nullptr, "unknown base link: " + base);
    const std::string resolved_tip = tip.empty() ? FindTip(model, base) : tip;
    const auto tip_link = model.getLink(resolved_tip);
    Require(tip_link != nullptr, "unknown tip link: " + resolved_tip);
    const Path path = FindPath(model, base, tip_link);
    Require(!path.empty(), resolved_tip + " is not below " + base);
    Require(IsArmPath(path), "selected chain must contain six independent revolute or continuous joints and fixed joints");

    Chain chain;
    chain.base = base;
    chain.tip = resolved_tip;
    SE3d origin = SE3d::Identity();
    std::size_t index = 0;
    for (const auto& joint : path) {
        // Accumulate fixed transforms into the next moving joint or the tip.
        origin = origin * ToIsometry(joint->parent_to_joint_origin_transform, joint->name);
        if (joint->type == urdf::Joint::FIXED) {
            continue;
        }
        const Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
        Require(axis.allFinite() && axis.norm() > 0.0, "invalid axis for joint " + joint->name);
        if (joint->type == urdf::Joint::REVOLUTE) {
            Require(joint->limits && std::isfinite(joint->limits->lower) &&
                            std::isfinite(joint->limits->upper) &&
                            joint->limits->lower <= joint->limits->upper,
                    "invalid limits for joint " + joint->name);
        }
        chain.joints[index++] = {joint, origin, axis.normalized()};
        origin.setIdentity();
    }
    chain.tip_in_joint6 = origin;
    return chain;
}

struct JointAxis {
    Vector3d position;
    Vector3d direction;
};

struct Configuration {
    std::array<JointAxis, kJointCount> axes;
    SE3d tip;
};

Configuration Evaluate(const Chain& chain, const Angles& angles) {
    Configuration result;
    SE3d pose = SE3d::Identity();
    for (std::size_t i = 0; i < angles.size(); ++i) {
        const auto& joint = chain.joints[i];
        pose = pose * joint.origin;
        result.axes[i] = {pose.translation(), pose.linear() * joint.axis};
        pose.rotate(Eigen::AngleAxisd(angles[i], joint.axis));
    }
    result.tip = pose * chain.tip_in_joint6;
    return result;
}

Vector3d WristCenter(const Configuration& pose) {
    const auto& fourth = pose.axes[3];
    const auto& fifth = pose.axes[4];
    const auto& sixth = pose.axes[5];
    const double alignment = fourth.direction.dot(fifth.direction);
    Require(std::abs(alignment) <= kAngularTolerance &&
                    std::abs(fifth.direction.dot(sixth.direction)) <= kAngularTolerance,
            "wrist axes 4–5 and 5–6 must be perpendicular");

    // Use the closest point on axis 4, including small angular rounding in the URDF.
    const Vector3d separation = fifth.position - fourth.position;
    const double along_fourth = (separation.dot(fourth.direction) -
                                 alignment * separation.dot(fifth.direction)) /
                                (1.0 - alignment * alignment);
    const Vector3d center = fourth.position + along_fourth * fourth.direction;
    const double fifth_gap = (center - fifth.position).cross(fifth.direction).norm();
    const double sixth_gap = (center - sixth.position).cross(sixth.direction).norm();
    if (!(fifth_gap <= kLinearTolerance && sixth_gap <= kLinearTolerance)) {
        std::ostringstream message;
        message << std::scientific << std::setprecision(3)
                << "wrist axes 4, 5 and 6 do not intersect at one point "
                << "(axis 5 gap " << fifth_gap << " m, axis 6 gap " << sixth_gap << " m, tolerance "
                << kLinearTolerance << " m)";
        throw std::runtime_error(message.str());
    }
    return center;
}

double AngleAbout(const Vector3d& from, const Vector3d& to, const Vector3d& axis) {
    return std::atan2(axis.dot(from.cross(to)), from.dot(to));
}

Vector3d AxisFacing(const Vector3d& axis, const Vector3d& target) {
    return axis.dot(target) < 0.0 ? -axis : axis;
}

void VerifyModel(const Chain& chain, const opw::OPWParameters& params, const Angles& neutral) {
    const opw::OPWKinematics solver(params);
    const std::array<Angles, 5> samples{{
            {}, neutral, {0.31, -0.41, 0.28, -0.77, 0.56, -0.33},
            {-0.52, 0.37, -0.69, 0.48, -0.81, 0.22},
            {0.87, -0.75, 0.61, 0.92, 0.84, -0.49},
    }};
    const double scale = std::max({1.0, params.c2, std::hypot(params.a2, params.c3),
                                   std::abs(params.c4), chain.tip_in_joint6.translation().norm()});
    for (const auto& sample : samples) {
        const SE3d expected = Evaluate(chain, sample).tip;
        const Eigen::Map<const Eigen::VectorXd> joints(sample.data(), sample.size());
        SE3d actual;
        Require(solver.GetFK(joints, actual) &&
                        actual.linear().isApprox(expected.linear(), 2.0 * kAngularTolerance) &&
                        (actual.translation() - expected.translation()).norm() <= 2.0 * kLinearTolerance * scale,
                "derived OPW parameters do not reproduce the URDF forward kinematics");
    }
}

opw::OPWParameters ExtractParameters(const Chain& chain) {
    opw::OPWParameters params;
    Angles neutral{};
    Configuration pose = Evaluate(chain, neutral);
    const Vector3d origin = pose.axes[0].position;
    const Vector3d z = pose.axes[0].direction;
    const Vector3d second_axis = pose.axes[1].direction;
    Require(std::abs(z.dot(second_axis)) <= kAngularTolerance, "axes 1 and 2 must be perpendicular");
    // Joint 1 sets model Z, and the shoulder displacement picks the sign of X.
    Vector3d x = second_axis.cross(z).normalized();
    const Vector3d shoulder = pose.axes[1].position - origin;
    if (x.dot(shoulder) < -kLinearTolerance) {
        x = -x;
    }
    const Vector3d y = z.cross(x);
    params.T_b_ob.translation() = origin;
    params.T_b_ob.linear().col(0) = x;
    params.T_b_ob.linear().col(1) = y;
    params.T_b_ob.linear().col(2) = z;
    params.a1 = x.dot(shoulder);
    params.c1 = z.dot(shoulder);

    const auto set_neutral = [&](std::size_t index, const Vector3d& axis, double offset) {
        const signed char direction = pose.axes[index].direction.dot(axis) >= 0.0 ? 1 : -1;
        params.rotation_directions[index] = direction;
        params.offsets[index] = offset;
        // Model angle zero occurs when the signed URDF angle equals the offset.
        neutral[index] = direction * offset;
        pose = Evaluate(chain, neutral);
    };

    const Vector3d upper_arm = pose.axes[2].position - pose.axes[1].position;
    const double upper_x = x.dot(upper_arm), upper_z = z.dot(upper_arm);
    params.c2 = std::hypot(upper_x, upper_z);
    Require(params.c2 > 0.0, "axes 2 and 3 must have nonzero separation");
    Require(pose.axes[2].direction.cross(y).norm() <= kAngularTolerance,
            "axes 2 and 3 must be parallel");
    // Align the upper arm with model Z before extracting the forearm and wrist.
    set_neutral(1, y, -std::atan2(upper_x, upper_z));

    const Vector3d wrist = WristCenter(pose);
    Vector3d fourth_axis = pose.axes[3].direction;
    Require(std::abs(fourth_axis.dot(y)) <= kAngularTolerance, "axes 3 and 4 must be perpendicular");
    const double projection = fourth_axis.dot(wrist - pose.axes[2].position);
    if (std::abs(projection) <= kLinearTolerance) {
        fourth_axis = AxisFacing(fourth_axis, z);
    } else if (projection < 0.0) {
        fourth_axis = -fourth_axis;
    }
    set_neutral(2, y, AngleAbout(fourth_axis, z, y));

    const Vector3d fifth_axis = AxisFacing(pose.axes[4].direction, y);
    set_neutral(3, z, AngleAbout(fifth_axis, y, z));
    const Vector3d sixth_axis = AxisFacing(pose.axes[5].direction, z);
    set_neutral(4, y, AngleAbout(sixth_axis, z, y));
    set_neutral(5, z, 0.0);

    const Vector3d center = WristCenter(pose);
    const Vector3d forearm = center - pose.axes[2].position;
    params.b = y.dot(center - origin);
    params.a2 = x.dot(forearm);
    params.c3 = z.dot(forearm);
    params.c4 = z.dot(pose.axes[5].position - center);

    // At model zero the flange axes match the model base axes.
    SE3d opw_flange = params.T_b_ob;
    opw_flange.translation() = center + params.c4 * z;
    params.T_oe_e = opw_flange.inverse() * pose.tip;
    VerifyModel(chain, params, neutral);
    return params;
}

void JsonString(const std::string& value) {
    constexpr char hex[] = "0123456789abcdef";
    std::cout << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') {
            std::cout << '\\' << c;
        } else if (c < 32) {
            std::cout << "\\u00" << hex[c / 16] << hex[c % 16];
        } else {
            std::cout << c;
        }
    }
    std::cout << '"';
}

template <typename Values>
void JsonArray(const Values& values) {
    std::cout << '[';
    for (std::size_t i = 0; i < static_cast<std::size_t>(values.size()); ++i) {
        std::cout << (i ? ", " : "") << +values[i];
    }
    std::cout << ']';
}

void JsonMatrix(const SE3d& pose) {
    std::cout << '[';
    for (int row = 0; row < 4; ++row) {
        if (row != 0) {
            std::cout << ", ";
        }
        JsonArray(pose.matrix().row(row));
    }
    std::cout << ']';
}

void PrintJson(const Chain& chain, const opw::OPWParameters& params) {
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "{\n  \"base_link\": ";
    JsonString(chain.base);
    std::cout << ",\n  \"tip_link\": ";
    JsonString(chain.tip);
    std::cout << ",\n  \"parameters\": {\n"
              << "    \"a1\": " << params.a1 << ", \"a2\": " << params.a2 << ", \"b\": " << params.b << ",\n"
              << "    \"c1\": " << params.c1 << ", \"c2\": " << params.c2
              << ", \"c3\": " << params.c3 << ", \"c4\": " << params.c4 << ",\n"
              << "    \"offsets\": ";
    JsonArray(params.offsets);
    std::cout << ",\n    \"rotation_directions\": ";
    JsonArray(params.rotation_directions);
    std::cout << ",\n    \"T_b_ob\": ";
    JsonMatrix(params.T_b_ob);
    std::cout << ",\n    \"T_oe_e\": ";
    JsonMatrix(params.T_oe_e);
    std::cout << "\n  },\n  \"joints\": [\n";
    for (std::size_t i = 0; i < chain.joints.size(); ++i) {
        const auto& joint = chain.joints[i];
        const auto& description = *joint.urdf_joint;
        const bool continuous = description.type == urdf::Joint::CONTINUOUS;
        std::cout << (i ? ",\n" : "") << "    {\"name\": ";
        JsonString(description.name);
        std::cout << ", \"type\": \"" << (continuous ? "continuous" : "revolute") << "\"";
        if (continuous) {
            std::cout << ", \"lower_limit\": null, \"upper_limit\": null";
        } else {
            std::cout << ", \"lower_limit\": " << description.limits->lower
                      << ", \"upper_limit\": " << description.limits->upper;
        }
        std::cout << ", \"axis\": ";
        JsonArray(joint.axis);
        std::cout << ", \"origin_pose\": ";
        JsonMatrix(joint.origin);
        std::cout << '}';
    }
    std::cout << "\n  ],\n  \"tip_in_joint6\": ";
    JsonMatrix(chain.tip_in_joint6);
    std::cout << "\n}\n";
}

constexpr char kUsage[] = "Usage: opw_from_urdf FILE [--base LINK] [--tip LINK]\n";

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << kUsage;
        return 0;
    }
    if (argc < 2) {
        std::cerr << kUsage;
        return 2;
    }

    std::string base, tip;
    for (int i = 2; i < argc; i += 2) {
        const std::string option = argv[i];
        std::string* value = option == "--base" ? &base : option == "--tip" ? &tip : nullptr;
        if (!value || !value->empty() || i + 1 == argc || argv[i + 1][0] == '-') {
            std::cerr << kUsage;
            return 2;
        }
        *value = argv[i + 1];
    }

    try {
        const auto model = urdf::parseURDFFile(argv[1]);
        Require(model && model->getRoot(), std::string("cannot read URDF: ") + argv[1]);
        if (base.empty()) {
            base = model->getRoot()->name;
        }
        const Chain chain = ReadChain(*model, base, tip);
        PrintJson(chain, ExtractParameters(chain));
    } catch (const std::exception& error) {
        std::cerr << "opw_from_urdf: " << error.what() << '\n';
        return 1;
    }
}
