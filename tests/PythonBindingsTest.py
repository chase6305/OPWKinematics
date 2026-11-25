import json
import subprocess
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
import opw_kinematics as opw


def main():
    params = opw.OPWParameters()
    params.a1, params.a2, params.b = 0.1, -0.08, 0.02
    params.c1, params.c2, params.c3, params.c4 = 0.35, 0.45, 0.30, 0.07
    params.offsets = [0.1, -0.2, 0.0, 0.3, 0.0, 0.0]
    params.rotation_directions = [1, -1, 1, -1, 1, 1]
    params.T_b_ob = np.eye(4)
    solver = opw.OPWKinematics(params)
    tcp = np.eye(4)
    tcp[2, 3] = 0.05
    solver.tcp = tcp
    np.testing.assert_allclose(solver.tcp, tcp)

    seed = np.array([0.3, -0.4, 0.6, 0.2, -0.5, 0.7])
    assert solver.set_home_joints(seed)
    pose = solver.get_fk(seed)
    assert pose.shape == (4, 4)
    joints, distances = solver.get_ik(pose)
    assert joints.shape == (len(distances), 6)
    assert np.all(distances[1:] >= distances[:-1])
    np.testing.assert_allclose(joints[0], seed, atol=1e-10)
    nearest, cost = solver.get_nearest_ik(pose, seed)
    np.testing.assert_allclose(nearest, seed, atol=1e-10)
    assert cost < 1e-10

    unreachable = pose.copy()
    unreachable[0, 3] = 100.0
    joints, distances = solver.get_ik(unreachable)
    assert joints.shape == (0, 6) and distances.shape == (0,)
    assert solver.get_nearest_ik(unreachable) is None

    invalid_pose = np.eye(4)
    invalid_pose[0, 0] = 2.0
    try:
        solver.tcp = invalid_pose
    except ValueError:
        np.testing.assert_allclose(solver.tcp, tcp)
    else:
        raise AssertionError("invalid rigid transform was accepted")


def check_urdf_data(tool_path, fixture_path):
    data = json.loads(subprocess.check_output([tool_path, fixture_path], text=True))
    solver = opw.OPWKinematics.from_urdf_data(data)
    upper, lower = solver.get_joint_limits()
    np.testing.assert_allclose(upper[:5], [j["upper_limit"] for j in data["joints"][:5]])
    np.testing.assert_allclose(lower[:5], [j["lower_limit"] for j in data["joints"][:5]])
    assert upper[5] == np.pi and lower[5] == -np.pi

    tcp = np.eye(4)
    tcp[:3, 3] = [0.08, 0.02, -0.03]
    solver.tcp = tcp
    user_frame = np.eye(4)
    user_frame[:3, 3] = [0.01, 0.20, 0.07]
    solver.user_frame = user_frame
    joints = np.array([0.3, -0.4, 0.6, 0.2, -0.5, 0.7])
    analytical_pose = solver.get_fk(joints)
    poses = solver.get_all_fk(joints)
    assert len(poses) == 7
    flange_in_joint6 = np.array(data["tip_in_joint6"])
    np.testing.assert_allclose(poses[-2] @ flange_in_joint6 @ tcp, analytical_pose, atol=1e-10)
    np.testing.assert_allclose(poses[-1], analytical_pose, atol=1e-10)
    nearest, _ = solver.get_nearest_ik(analytical_pose, joints)
    np.testing.assert_allclose(nearest, joints, atol=1e-10)


if __name__ == "__main__":
    main()
    if len(sys.argv) == 4:
        check_urdf_data(sys.argv[2], sys.argv[3])
