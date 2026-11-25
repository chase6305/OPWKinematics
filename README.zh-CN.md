# OPWKinematics

[English](README.md)

适用于正交平行基座、球形腕部的六轴工业机械臂。提供解析正运动学（FK）、全部离散解析逆解（IK）和按种子选取最近逆解。C++17，公共依赖只有 Eigen。

## Conan 2

首次使用 Conan 且没有默认 profile 时，运行 `conan profile detect`。在本仓库根目录执行以下命令即可创建本地包 `opw-kinematics/0.1.0`。需要动态库时添加 `-o 'opw-kinematics/*:shared=True'`。

```sh
conan create . --build=missing -c tools.build:jobs=2
```

在消费项目的 `conanfile.txt` 中声明依赖，并通过 Conan 生成的 toolchain 配置 CMake。

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

## 直接使用 CMake

需要 CMake 3.15 或更新版本和可被 CMake 找到的 Eigen 3.3 或更新版本。安装后，消费项目可将安装前缀加入 `CMAKE_PREFIX_PATH`，使用相同的 `find_package` 和链接目标。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
(cd build && ctest --parallel 1 --output-on-failure)
cmake --install build --prefix "$PWD/build/install"
```

## C++ 用法

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

- `SE3d` 是 `Eigen::Isometry3d`，用于刚体变换的组合与求逆。关节角和关节偏移以弧度表示，几何参数和位姿平移使用同一长度单位。
- 内部模型角为 `joint * rotation_direction - offset`。从采用加偏移约定的参数表迁移时，将偏移取负。旋转方向默认为 `+1`。
- `T_b_ob` 表示 OPW 基座相对于实际基座的位姿，`T_oe_e` 表示实际法兰相对于 OPW 法兰的位姿。`SetTCP` 接收 TCP 相对于实际法兰的位姿。`SetUserFrame` 接收用户坐标系相对于实际基座的位姿。FK 和 IK 的目标 TCP 位姿都以用户坐标系表示，四个变换均默认为单位变换。
- `GetIK` 按 `Σ weight[i] * |joint[i] - seed[i]|` 由近到远返回，`distances` 与 `solutions.ik_joints` 一一对应。`GetNearestIK` 按同一代价直接选取限位内最近的等价关节角，兼容拼写 `GetNearstIK`。种子为空、长度不为 6 或包含非有限值时，使用 `SetHomeJoints` 设置的关节角，默认全零。
- `GetIK` 枚举系统限位和用户限位交集内的 `2π` 等价解，较宽的多圈范围可能产生大量结果。两组限位均默认为 `[-π, π]`。多圈关节可在构造时传入自己的 `JointNode`，或同时扩展两组限位。
- 腕部奇异时，IK 为耦合关节的每个可行圈数选取加权运动量最小的一组解。底座及完全折叠的肘部奇异时，IK 先尝试种子对应的臂部角度。如这些角度无法满足腕部限位，再搜索自由关节的可行区间。完全折叠且腕部锁定时，可根据腕部姿态恢复两个自由臂部角。奇异位形返回有限组代表解。
- `GetAllFK` 从传入的关节链计算六个关节位姿和末端 TCP。用 `SetJointChainFlange` 指定最后一个可动关节到法兰之间的固定变换。启用肘部筛选 `SetElbowUp(true)` 时，需要提供前四个关节的原点和轴。

## Python 用法

可选的 `opw_kinematics` 模块使用 CPython 和 NumPy。以下命令适用于 Conan 的 `Release` profile 和单配置生成器，在仓库根目录执行。

```sh
conan build . --build=missing -c tools.build:jobs=2 -o '&:with_python=True'
export PYTHONPATH="$PWD/build/Release${PYTHONPATH:+:$PYTHONPATH}"
```

可通过 `OPW_PYTHON_EXECUTABLE` 指定解释器，构建和导入时使用同一个解释器。给 `conan create` 添加 `-o 'opw-kinematics/*:with_python=True'` 可将模块打入包中，消费项目的 `conanrun` 脚本会设置 `PYTHONPATH`。直接用 CMake 构建时，设置 `-DOPW_BUILD_PYTHON=ON`，需要 CMake 3.18 或更新版本、带开发头文件的 CPython 3.8 或更新版本，以及 pybind11 2.12 或更新版本。

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

位姿使用 4×4 NumPy 数组。`get_ik` 返回 `N × 6` 的解矩阵和 `N` 个距离，目标不可达时返回两个空数组。`get_nearest_ik` 返回 `(joints, distance)` 或 `None`。两个 IK 方法都可省略种子，省略时使用配置的初始关节角。

## 从 URDF 提取 OPW 参数

`opw_from_urdf` 是独立的命令行工具。它根据 URDF 关节轴和连接变换推导参数，并在多个关节位形下对比 URDF 链和 OPW 模型的正运动学。工具通过可选的 urdfdom 依赖解析 URDF。

```sh
conan build . --build=missing -c tools.build:jobs=2 -o '&:with_urdf_tool=True'
build/Release/opw_from_urdf robot.urdf --base base_link --tip tool0 > opw.json
```

上面的路径对应 Conan 的 `Release` profile 和单配置生成器。需要在 Conan 包内安装工具时，运行 `conan create . --build=missing -c tools.build:jobs=2 -o 'opw-kinematics/*:with_urdf_tool=True'`。直接使用 CMake 时，安装系统的 urdfdom 开发包并加上 `-DOPW_BUILD_URDF_TOOL=ON`。

- `--base` 默认使用 URDF 根链接。只有一个符合条件的末端时，工具会自动选取。有多个候选末端时，工具会列出可传给 `--tip` 的链接名。
- 所选链需要有六个独立的 `revolute` 或 `continuous` 关节，允许在任意位置插入固定关节。传入普通 URDF 文件。如果模型以 Xacro 编写，先展开为 URDF。
- 工具会提取关节零位偏置和 `a1`、`a2`、`b` 几何偏置。腕部三根轴线需要共点。未通过这项检查时，工具会报告轴线间距。
- JSON 的 `parameters` 对应 `OPWParameters`。`T_b_ob` 和 `T_oe_e` 是 4×4 矩阵，使求解器在所选基座坐标系下返回所选末端的位姿。`joints` 按基座到末端的顺序列出关节名、轴、原点变换和限位。固定接头合并进后续关节的 `origin_pose`，第六轴以后的固定变换单独记录为 `tip_in_joint6`。连续关节的上下限为 `null`。
- 对于轴线不符合 OPW 几何的链，工具将原因写入标准错误并以非零状态退出。

CobotMagicArm 无夹爪版本使用 `--base base_link --tip link6`，带夹爪版本使用 `--tip gripper_base`。

如需同时构建命令行工具和 Python 模块，在同一次 Conan 构建中启用两个选项。

```sh
conan build . --build=missing -c tools.build:jobs=2 \
  -o '&:with_urdf_tool=True' -o '&:with_python=True'
```

Python 可用 `OPWKinematics.from_urdf_data` 直接读取工具输出。构造后的求解器使用导入的关节限位和固定末端变换，两种 FK 接口均默认以选定的末端为 TCP。连续关节的初始限位为 `[-π, π]`。

```python
import json
import opw_kinematics as opw

with open("opw.json", encoding="utf-8") as source:
    solver = opw.OPWKinematics.from_urdf_data(json.load(source))

upper, lower = solver.get_joint_limits()
joints = (lower + upper) / 2
joint_and_tcp_poses = solver.get_all_fk(joints)
```

## 参考论文

本仓库的 OPW 正运动学和逆运动学实现基于以下论文。

Mathias Brandstötter、Arthur Angerer、Michael Hofbaur。“An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist.” *Proceedings of the Austrian Robotics Workshop*，奥地利林茨，2014 年 5 月 22–23 日。[作者发布的修订版](https://www.researchgate.net/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist)。
