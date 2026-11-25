# OPWKinematics

[简体中文](README.zh-CN.md)

A C++17 library for six-axis industrial robots with an ortho-parallel base and a spherical wrist. It provides forward kinematics (FK), analytic inverse kinematics (IK), and seed-based selection of the nearest solution. Eigen is the only public dependency.

## Conan 2

Run `conan profile detect` if you do not have a default profile. Run the following command from the repository root to create `opw-kinematics/0.1.0` locally. Add `-o 'opw-kinematics/*:shared=True'` for a shared library.

```sh
conan create . --build=missing -c tools.build:jobs=2
```

Add the dependency to the consumer project's `conanfile.txt` and configure CMake with the generated toolchain.

```ini
[requires]
opw-kinematics/0.1.0

[generators]
CMakeDeps
CMakeToolchain
```

```cmake
find_package(OPWKinematics 0.1 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE OPWKinematics::OPWKinematics)
```

## CMake without Conan

The build requires CMake 3.15 or later and an Eigen 3.3 or later installation discoverable by CMake. Consumers can add the installation prefix to `CMAKE_PREFIX_PATH` and use the same `find_package` call and link target shown above.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
(cd build && ctest --parallel 1 --output-on-failure)
cmake --install build --prefix "$PWD/build/install"
```

## C++ usage

```cpp
#include <opw_kinematics/OPWKinematics.h>

int main() {
    opw::OPWParameters params;
    params.a1 = 0.10;
    params.a2 = -0.08;
    params.b = 0.02;
    params.c1 = 0.35;
    params.c2 = 0.45;
    params.c3 = 0.30;
    params.c4 = 0.07;
    opw::OPWKinematics solver(params);

    Eigen::VectorXd joints(6);
    joints << 0.3, -0.4, 0.6, 0.2, -0.5, 0.7;
    opw::SE3d target;
    if (!solver.GetFK(joints, target)) {
        return 1;
    }

    Eigen::VectorXd seed = joints;
    opw::IkRtn solutions;
    std::vector<double> distances;
    return solver.GetIK(target, solutions, seed, distances) ? 0 : 1;
}
```

- `SE3d` aliases `Eigen::Isometry3d`, which handles rigid-transform composition and inversion. Joint angles and offsets are in radians. Link dimensions and pose translations use the same length unit.
- The model angle is `joint * rotation_direction - offset`. Negate offsets imported from a model that adds them. Rotation directions default to `+1`.
- `T_b_ob` is the OPW base pose in the robot base frame. `T_oe_e` is the robot flange pose in the OPW flange frame. `SetTCP` takes the TCP pose in the robot flange frame. `SetUserFrame` takes the user frame pose in the robot base frame. All four transforms default to identity. FK returns and IK accepts the TCP pose in the user frame.
- `GetIK` sorts solutions by `Σ weight[i] * |joint[i] - seed[i]|` and returns matching `distances`. `GetNearestIK` uses the same cost and resolves the nearest allowed joint equivalents directly. `GetNearstIK` remains available as an alias. A seed with the wrong size or non-finite values is replaced by the home joints, which default to zero.
- `GetIK` enumerates `2π` equivalents within the intersection of system and user joint limits, so broad multi-turn ranges can produce many results. Both limits default to `[-π, π]`. Pass custom `JointNode` values or extend both sets of limits to allow multiple turns.
- At a wrist singularity, IK returns the minimum-cost pair for each feasible winding of the coupled joints. At base and folded-elbow singularities, IK tries seed-based arm angles first. When these violate wrist limits, it tests the permitted intervals of the free arm joint. A fully folded arm with a locked wrist recovers both free arm angles from the wrist orientation. Singular configurations are represented by a finite set of solutions.
- `GetAllFK` uses the supplied joint chain and returns six joint poses followed by the TCP pose. Use `SetJointChainFlange` to apply a fixed transform between the last moving joint and the chain's flange. `SetElbowUp(true)` uses the first four joint origins and axes.

## Python usage

The optional `opw_kinematics` module uses CPython and NumPy. With Conan and a single-configuration `Release` profile, build and expose the module from the repository root.

```sh
conan build . --build=missing -c tools.build:jobs=2 -o '&:with_python=True'
export PYTHONPATH="$PWD/build/Release${PYTHONPATH:+:$PYTHONPATH}"
```

Set `OPW_PYTHON_EXECUTABLE` to build for a specific interpreter and use that interpreter to import the module. Use `-o 'opw-kinematics/*:with_python=True'` with `conan create` to include the module in the package. The consumer's `conanrun` script adds the module to `PYTHONPATH`. For a direct CMake build, enable `-DOPW_BUILD_PYTHON=ON` with CMake 3.18 or newer, CPython 3.8 or newer with development headers, and pybind11 2.12 or newer.

```python
import numpy as np
import opw_kinematics as opw

params = opw.OPWParameters()
params.a1, params.a2, params.b = 0.10, -0.08, 0.02
params.c1, params.c2, params.c3, params.c4 = 0.35, 0.45, 0.30, 0.07
solver = opw.OPWKinematics(params)

seed = np.array([0.3, -0.4, 0.6, 0.2, -0.5, 0.7])
pose = solver.get_fk(seed)
joints, distances = solver.get_ik(pose, seed)
nearest = solver.get_nearest_ik(pose, seed)
```

Poses are 4×4 NumPy arrays. `get_ik` returns an `N × 6` joint array and `N` distances. For an unreachable pose it returns two empty arrays. `get_nearest_ik` returns `(joints, distance)` or `None`. Both IK methods use the configured home joints when the seed is omitted.

## Extracting OPW parameters from URDF

The standalone `opw_from_urdf` command derives OPW parameters from URDF joint axes and transforms. It verifies the model against URDF forward kinematics at several configurations. Enable the tool to add the urdfdom dependency.

```sh
conan build . --build=missing -c tools.build:jobs=2 -o '&:with_urdf_tool=True'
build/Release/opw_from_urdf robot.urdf --base base_link --tip tool0 > opw.json
```

The executable path above assumes a single-configuration generator and a `Release` profile. To include the executable in the Conan package, run `conan create . --build=missing -c tools.build:jobs=2 -o 'opw-kinematics/*:with_urdf_tool=True'`. To build directly with CMake, install the urdfdom development package and set `-DOPW_BUILD_URDF_TOOL=ON`.

- `--base` defaults to the URDF root link. If exactly one tip qualifies, the tool detects it automatically. When several tips qualify, the tool lists the candidates for `--tip`.
- The chain must contain six independent `revolute` or `continuous` joints. Fixed joints can occur anywhere in the chain. Expand Xacro files to URDF before using the tool.
- The extracted parameters include joint zero offsets and the OPW link offsets `a1`, `a2`, and `b`. The three wrist axes must still intersect. The diagnostic reports the axis gaps when the wrist fails this check.
- The JSON `parameters` correspond to `OPWParameters`. The 4×4 matrices `T_b_ob` and `T_oe_e` make FK return the selected tip pose in the selected base frame. `joints` lists names, axes, origins, and limits in base-to-tip order. Fixed transforms are folded into the next `origin_pose`, with any transform after joint 6 stored in `tip_in_joint6`. Continuous joints have `null` limits.
- For a chain that fails the OPW geometry checks, the tool writes a diagnostic to standard error and exits with a non-zero status.

For CobotMagicArm, use `--base base_link --tip link6` with the URDF without a gripper. Use `--tip gripper_base` for either of the gripper versions.

To build the command and bindings together, pass both Conan options in the same build.

```sh
conan build . --build=missing -c tools.build:jobs=2 \
  -o '&:with_urdf_tool=True' -o '&:with_python=True'
```

The Python binding loads this output with `OPWKinematics.from_urdf_data`. The resulting solver uses the imported joint limits and fixed tip transform for both TCP FK paths. Continuous joints start with the finite interval `[-π, π]`.

```python
import json
import opw_kinematics as opw

with open("opw.json", encoding="utf-8") as source:
    solver = opw.OPWKinematics.from_urdf_data(json.load(source))

upper, lower = solver.get_joint_limits()
joints = (lower + upper) / 2
joint_and_tcp_poses = solver.get_all_fk(joints)
```

## Interactive Viser example

With the Python bindings and URDF tool built as above, run the example in the same Python environment.

```sh
python3 -m pip install 'viser>=1.1,<2'
python3 examples/viser_kinematics.py --python-dir build/Release
```

Open `http://127.0.0.1:8080`. The blue arm shows FK from the six joint controls. Drag the target to preview an IK solution, then release it to compute the full set. You can also enter **Target XYZ (m)**, select a solution by index, or copy the displayed solution to the FK sliders with **Use IK as seed**. The panel shows the TCP error.

The bundled OPW model is loaded by default. Pass `--urdf robot.urdf` with optional `--base` and `--tip` link names for a different arm, or pass `--json opw.json` to reuse extracted parameters. If the extractor is elsewhere, pass its path with `--extractor`.

For URDF visuals, install `'viser[urdf]>=1.1,<2'` and pass `--meshes` with a URDF that has accessible visual assets.

```sh
python3 -m pip install 'viser[urdf]>=1.1,<2'
python3 examples/viser_kinematics.py --python-dir build/Release --meshes --urdf robot.urdf --base base_link --tip tool0
```

## Reference

The OPW forward and inverse kinematics implemented here are based on the following paper.

Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur. “An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist.” *Proceedings of the Austrian Robotics Workshop*, Linz, Austria, 22–23 May 2014. [Revised paper shared by the authors](https://www.researchgate.net/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist).
