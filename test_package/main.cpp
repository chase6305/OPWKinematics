#include <opw_kinematics/OPWKinematics.h>

int main() {
    opw::OPWParameters params;
    params.a1 = 0.1;
    params.a2 = -0.08;
    params.b = 0.02;
    params.c1 = 0.35;
    params.c2 = 0.45;
    params.c3 = 0.30;
    params.c4 = 0.07;
    opw::OPWKinematics solver(params);

    Eigen::VectorXd joints(6);
    joints << 0.3, -0.4, 0.6, 0.2, -0.5, 0.7;
    opw::SE3d pose;
    if (!solver.GetFK(joints, pose)) {
        return 1;
    }

    opw::IkRtn solutions;
    double distance;
    return solver.GetNearestIK(pose, solutions, joints, distance) && distance < 1e-8 ? 0 : 1;
}
