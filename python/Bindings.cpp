#include <opw_kinematics/OPWKinematics.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "Transforms.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

using opw::IkRtn;
using opw::JointNode;
using opw::JointType;
using opw::OPWKinematics;
using opw::OPWParameters;
using opw::SE3d;
using JointSolutions = Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>;

struct GeometryField {
    const char* name;
    double OPWParameters::*member;
};

constexpr std::array<GeometryField, 7> kGeometryFields{{
        {"a1", &OPWParameters::a1}, {"a2", &OPWParameters::a2}, {"b", &OPWParameters::b},
        {"c1", &OPWParameters::c1}, {"c2", &OPWParameters::c2},
        {"c3", &OPWParameters::c3}, {"c4", &OPWParameters::c4},
}};

SE3d ToPose(const Eigen::Matrix4d& matrix) {
    const SE3d pose(matrix);
    if (!opw::detail::IsRigidTransform(pose)) {
        throw py::value_error("expected a finite 4x4 rigid transform");
    }
    return pose;
}

// Return matrices by value so editing an array cannot silently change a pose.
template <typename Object>
void BindPoseProperty(py::class_<Object>& cls, const char* name, SE3d Object::*member) {
    cls.def_property(name,
        [member](const Object& obj) -> Eigen::Matrix4d { return (obj.*member).matrix(); },
        [member](Object& obj, const Eigen::Matrix4d& matrix) { obj.*member = ToPose(matrix); });
}

OPWKinematics FromURDFData(const py::dict& data) {
    const auto parameter_data = data["parameters"].cast<py::dict>();
    OPWParameters params;
    for (const auto& field : kGeometryFields) {
        params.*field.member = parameter_data[field.name].cast<double>();
    }
    params.offsets = parameter_data["offsets"].cast<decltype(params.offsets)>();
    params.rotation_directions =
            parameter_data["rotation_directions"].cast<decltype(params.rotation_directions)>();
    params.T_b_ob = ToPose(parameter_data["T_b_ob"].cast<Eigen::Matrix4d>());
    params.T_oe_e = ToPose(parameter_data["T_oe_e"].cast<Eigen::Matrix4d>());

    const auto joint_data = data["joints"].cast<py::list>();
    if (joint_data.size() != params.offsets.size()) {
        throw py::value_error("URDF tool data must contain six moving joints");
    }
    std::vector<JointNode> nodes;
    nodes.reserve(joint_data.size());
    for (const auto& entry : joint_data) {
        const auto joint = entry.cast<py::dict>();
        const auto type = joint["type"].cast<std::string>();
        const bool continuous = type == "continuous";
        if (!continuous && type != "revolute") {
            throw py::value_error("expected only revolute or continuous joints in URDF tool data");
        }
        // Continuous joints in URDF have no limits. Use the solver's finite default interval.
        const double lower = continuous ? -opw::detail::kPi : joint["lower_limit"].cast<double>();
        const double upper = continuous ? opw::detail::kPi : joint["upper_limit"].cast<double>();
        const auto origin = ToPose(joint["origin_pose"].cast<Eigen::Matrix4d>());
        const auto axis = joint["axis"].cast<Eigen::Vector3d>();
        if (!axis.allFinite() || std::abs(axis.norm() - 1.0) > 1e-8) {
            throw py::value_error("URDF tool data contains a non-unit joint axis");
        }
        nodes.emplace_back(origin, axis, continuous ? JointType::CONTINUOUS : JointType::REVOLUTE,
                           lower, upper);
    }

    OPWKinematics solver(std::move(params), nodes);
    solver.SetJointChainFlange(ToPose(data["tip_in_joint6"].cast<Eigen::Matrix4d>()));
    return solver;
}

}  // namespace

PYBIND11_MODULE(opw_kinematics, module) {
    module.doc() = "Analytic kinematics for six-axis OPW robots";

    auto parameters = py::class_<OPWParameters>(module, "OPWParameters");
    for (const auto& field : kGeometryFields) {
        parameters.def_readwrite(field.name, field.member);
    }
    parameters.def(py::init<>())
        .def_readwrite("offsets", &OPWParameters::offsets)
        .def_readwrite("rotation_directions", &OPWParameters::rotation_directions);
    BindPoseProperty(parameters, "T_b_ob", &OPWParameters::T_b_ob);
    BindPoseProperty(parameters, "T_oe_e", &OPWParameters::T_oe_e);

    py::enum_<JointType>(module, "JointType")
        .value("UNKNOWN", JointType::UNKNOWN)
        .value("REVOLUTE", JointType::REVOLUTE)
        .value("PRISMATIC", JointType::PRISMATIC)
        .value("CONTINUOUS", JointType::CONTINUOUS)
        .value("FIXED", JointType::FIXED);
    auto joint_node = py::class_<JointNode>(module, "JointNode");
    joint_node.def(py::init<>())
        .def(py::init([](const Eigen::Matrix4d& origin, const Eigen::Vector3d& axis,
                         JointType type, double lower, double upper) {
            return JointNode(ToPose(origin), axis, type, lower, upper);
        }), "origin_pose"_a, "axis"_a, "joint_type"_a, "lower"_a, "upper"_a)
        .def_readwrite("axis", &JointNode::axis)
        .def_readwrite("joint_type", &JointNode::joint_type)
        .def_readwrite("lower_limit", &JointNode::lower_limit)
        .def_readwrite("upper_limit", &JointNode::upper_limit)
        .def_readwrite("lower_limit_set", &JointNode::lower_limit_set)
        .def_readwrite("upper_limit_set", &JointNode::upper_limit_set);
    BindPoseProperty(joint_node, "origin_pose", &JointNode::origin_pose);

    using FilterMode = OPWKinematics::FilterMode;
    py::enum_<FilterMode>(module, "FilterMode")
        .value("NONE", FilterMode::NONE)
        .value("POSITIVE", FilterMode::POSITIVE)
        .value("NEGATIVE", FilterMode::NEGATIVE);

    py::class_<OPWKinematics>(module, "OPWKinematics")
        .def(py::init<OPWParameters>(), "params"_a)
        .def(py::init<OPWParameters, const std::vector<JointNode>&>(), "params"_a, "joint_nodes"_a)
        .def_static("from_urdf_data", &FromURDFData, "data"_a,
                    "Build a solver from the JSON output of opw_from_urdf.")
        .def_property_readonly("dof", &OPWKinematics::GetDOF)
        .def("get_fk", [](const OPWKinematics& solver, const Eigen::VectorXd& joints) -> Eigen::Matrix4d {
            SE3d pose;
            if (!solver.GetFK(joints, pose)) {
                throw py::value_error("cannot compute FK for the supplied joints");
            }
            return pose.matrix();
        }, "joints"_a, "Return the TCP pose as a 4x4 NumPy array.")
        .def("get_all_fk", [](const OPWKinematics& solver, const Eigen::VectorXd& joints) {
            std::vector<SE3d> poses;
            if (!solver.GetAllFK(joints, poses)) {
                throw py::value_error("joint-chain FK requires six finite joints and their geometry");
            }
            std::vector<Eigen::Matrix4d> matrices;
            matrices.reserve(poses.size());
            for (const auto& pose : poses) {
                matrices.push_back(pose.matrix());
            }
            return matrices;
        }, "joints"_a, "Return six joint poses and the final TCP pose as 4x4 arrays.")
        .def("get_ik", [](const OPWKinematics& solver, const Eigen::Matrix4d& target,
                          const std::optional<Eigen::VectorXd>& seed) {
            IkRtn result;
            std::vector<double> distances;
            Eigen::VectorXd joint_seed = seed.value_or(Eigen::VectorXd{});
            solver.GetIK(ToPose(target), result, joint_seed, distances);
            JointSolutions joints(result.ik_number, 6);
            Eigen::VectorXd costs(result.ik_number);
            for (int i = 0; i < result.ik_number; ++i) {
                joints.row(i) = result.ik_joints[i].transpose();
                costs[i] = distances[i];
            }
            return py::make_tuple(std::move(joints), std::move(costs));
        }, "pose"_a, "seed"_a = py::none(), "Return the sorted (N x 6 solutions, N distances).")
        .def("get_nearest_ik", [](const OPWKinematics& solver, const Eigen::Matrix4d& target,
                                  const std::optional<Eigen::VectorXd>& seed) -> py::object {
            IkRtn result;
            double distance;
            Eigen::VectorXd joint_seed = seed.value_or(Eigen::VectorXd{});
            if (!solver.GetNearestIK(ToPose(target), result, joint_seed, distance)) {
                return py::none();
            }
            return py::make_tuple(std::move(result.ik_joints.front()), distance);
        }, "pose"_a, "seed"_a = py::none(), "Return the nearest (joints, distance), or None.")
        .def("is_reachable", [](const OPWKinematics& solver, const Eigen::Matrix4d& target) {
            return solver.IsReachable(ToPose(target));
        }, "pose"_a)
        .def_property("tcp",
            [](const OPWKinematics& solver) -> Eigen::Matrix4d { return solver.GetTCP().matrix(); },
            [](OPWKinematics& solver, const Eigen::Matrix4d& pose) { solver.SetTCP(ToPose(pose)); })
        .def_property("user_frame",
            [](const OPWKinematics& solver) -> Eigen::Matrix4d { return solver.GetUserFrame().matrix(); },
            [](OPWKinematics& solver, const Eigen::Matrix4d& pose) { solver.SetUserFrame(ToPose(pose)); })
        .def("set_home_joints", &OPWKinematics::SetHomeJoints, "joints"_a)
        .def("set_weights", &OPWKinematics::SetIkNearstWeight, "weights"_a)
        .def("get_joint_limits", [](const OPWKinematics& solver) {
            Eigen::VectorXd upper, lower;
            solver.GetJointLimits(upper, lower);
            return py::make_tuple(std::move(upper), std::move(lower));
        }, "Return the system limits as (upper, lower).")
        .def("set_joint_limits", &OPWKinematics::SetJointLimits, "upper"_a, "lower"_a)
        .def("set_user_joint_limits", &OPWKinematics::SetUserJointLimits, "upper"_a, "lower"_a)
        .def("set_joint_filter", &OPWKinematics::SetJointFilter, "index"_a, "mode"_a)
        .def("set_joint_angle_threshold", &OPWKinematics::SetJointAngleThreshold, "index"_a, "threshold"_a)
        .def("reset_joint_filters", &OPWKinematics::ResetJointFilters)
        .def("set_elbow_up", &OPWKinematics::SetElbowUp, "enable"_a);
}
