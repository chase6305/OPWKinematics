"""Explore an OPW robot and its FK/IK solutions in Viser."""

import argparse
import asyncio
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import viser
import viser.transforms as tf

REPO = Path(__file__).resolve().parents[1]
SEED_COLOR = (47, 110, 218)
SOLUTION_COLOR = (226, 145, 33)
EDIT_INTERVAL = 1 / 25  # Seconds between viewer updates while dragging.


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--urdf", type=Path, help="Use the bundled OPW arm by default.")
    source.add_argument("--json", type=Path, help="Read JSON saved by opw_from_urdf.")
    parser.add_argument("--python-dir", type=Path, help="Directory containing the built opw_kinematics module.")
    parser.add_argument("--extractor", type=Path, help="Path to the opw_from_urdf executable.")
    parser.add_argument("--base", help="Base link of the selected URDF chain.")
    parser.add_argument("--tip", help="Tip link of the selected URDF chain.")
    parser.add_argument("--meshes", action="store_true", help="Render the selected IK solution with the URDF visual meshes.")
    parser.add_argument("--host", default="127.0.0.1", help="Address for the Viser server.")
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP port.")
    args = parser.parse_args()
    if args.json and any((args.extractor, args.base, args.tip)):
        parser.error("--extractor, --base and --tip apply only to URDF input")
    if args.meshes and args.urdf is None:
        parser.error("--meshes requires a model path passed with --urdf")
    return parser, args


def load_chain(args, module_path):
    if args.json:
        return json.loads(args.json.read_text(encoding="utf-8"))

    executable = "opw_from_urdf.exe" if sys.platform == "win32" else "opw_from_urdf"
    extractor = args.extractor
    if extractor is None:
        sibling = module_path.with_name(executable)
        default_build = REPO / "build" / "Release" / executable
        extractor = next((path for path in (sibling, default_build) if path.is_file()), None)
        extractor = extractor or shutil.which(executable)
    if extractor is None:
        raise ValueError("opw_from_urdf was not found. Pass --extractor or use --json.")

    urdf = args.urdf or REPO / "tests" / "fixtures" / "opw_arm.urdf"
    command = [str(Path(extractor).resolve()), str(urdf)]
    for flag, value in (("--base", args.base), ("--tip", args.tip)):
        if value is not None:
            command.extend((flag, value))
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise ValueError(result.stderr.strip() or f"opw_from_urdf exited with {result.returncode}")
    return json.loads(result.stdout)


def chain_points(poses):
    # The binding returns each joint frame followed by the TCP frame.
    return np.vstack((np.zeros(3), poses[:, :3, 3])).astype(np.float32)


def line_segments(points):
    return np.stack((points[:-1], points[1:]), axis=1)


def place_frame(handle, pose):
    handle.position = pose[:3, 3]
    handle.wxyz = tf.SO3.from_matrix(pose[:3, :3]).wxyz


def load_visual_model(path):
    try:
        import yourdfpy
    except ImportError as error:
        raise ValueError("install 'viser[urdf]>=1.1,<2' to use --meshes") from error
    def locate_mesh(fname):
        reference = Path(fname)
        local = path.parent / reference
        if local.is_file():
            return str(local)
        # Some extracted robot assets place the URDF next to the mesh folders
        # even though the original URDF refers to them from a subdirectory.
        if reference.parts and reference.parts[0] == "..":
            relocated = path.parent.joinpath(*reference.parts[1:])
            if relocated.is_file():
                return str(relocated)
        return yourdfpy.filename_handler_magic(fname, dir=path.parent)

    model = yourdfpy.URDF.load(
        path, build_scene_graph=True, load_meshes=True, filename_handler=locate_mesh
    )
    if model.scene is None or not model.scene.geometry:
        raise ValueError(f"{path} has no loadable visual geometry")
    return model


class ArmView:
    def __init__(self, server, name, poses, color, scale, *, show_tcp):
        self.server = server
        points = chain_points(poses)
        self.root = server.scene.add_frame(name, show_axes=False)
        self.links = server.scene.add_line_segments(
            f"{name}/links", line_segments(points), color, thickness=0.018 * scale
        )
        self.joints = [
            server.scene.add_icosphere(
                f"{name}/joint_{i}", radius=0.019 * scale, color=color,
                subdivisions=1, position=point,
            )
            for i, point in enumerate(points[1:-1], start=1)
        ]
        self.tcp = None
        if show_tcp:
            self.tcp = server.scene.add_frame(
                f"{name}/tcp", axes_length=0.11 * scale, axes_radius=0.003 * scale
            )
            place_frame(self.tcp, poses[-1])

    def update(self, poses):
        points = chain_points(poses)
        with self.server.atomic():
            self.links.points = line_segments(points)
            for joint, point in zip(self.joints, points[1:-1]):
                joint.position = point
            if self.tcp is not None:
                place_frame(self.tcp, poses[-1])
            self.root.visible = True

    def show_skeleton(self, visible):
        self.links.visible = visible
        for joint in self.joints:
            joint.visible = visible


class SolutionMeshes:
    def __init__(self, server, robot, data):
        from viser.extras import ViserUrdf

        name = "/ik_solution/meshes"
        self.root = server.scene.add_frame(name, show_axes=False)
        self.viewer = ViserUrdf(server, robot, root_node_name=name)
        names = list(self.viewer.get_actuated_joint_names())
        self.indices = [names.index(joint["name"]) for joint in data["joints"]]
        limits = self.viewer.get_actuated_joint_limits()
        self.configuration = np.array([
            np.clip(0.0, -np.inf if lo is None else lo, np.inf if hi is None else hi)
            for lo, hi in (limits[name] for name in names)
        ])
        self.viewer.update_cfg(self.configuration)
        # FK poses are expressed in the base selected by the extractor.
        place_frame(self.root, np.linalg.inv(robot.get_transform(data["base_link"])))

    def update(self, joints):
        if np.array_equal(self.configuration[self.indices], joints):
            return
        self.configuration[self.indices] = joints
        self.viewer.update_cfg(self.configuration)


def show_kinematics(server, solver, data, visual_model=None):
    upper, lower = solver.get_joint_limits()
    lower_deg, upper_deg = np.rad2deg((lower, upper))
    initial_deg = np.clip((25, -45, 60, -30, 45, 20), lower_deg, upper_deg)
    initial = np.deg2rad(initial_deg)

    def poses_for(joints):
        poses = np.asarray(solver.get_all_fk(joints))
        # Use the same fitted TCP pose for rendering and the IK error.
        poses[-1] = solver.get_fk(joints)
        return poses

    poses = poses_for(initial)
    # Start with a second reachable pose so the two arms are easy to compare.
    start_goal = np.clip(initial + np.deg2rad((38, -5, -22, 20, 14, -15)), lower, upper)
    goal_poses = poses_for(start_goal)
    points = chain_points(poses)
    scale = max(0.25, float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()))
    overview = np.vstack((points, chain_points(goal_poses)))
    center = (overview.min(axis=0) + overview.max(axis=0)) / 2

    server.gui.configure_theme(control_width="large", show_share_button=False, brand_color=SEED_COLOR)
    server.gui.main_panel.dock_right()
    server.scene.set_up_direction("+z")
    server.initial_camera.look_at = center
    server.initial_camera.position = center + scale * np.array((0.95, -1.35, 0.95))
    server.initial_camera.fov = np.deg2rad(45)
    server.scene.add_grid(
        "/floor", width=3 * scale, height=3 * scale, cell_size=scale / 10,
        section_size=scale / 2, cell_color=(218, 224, 232), section_color=(166, 179, 196),
        position=(0, 0, -0.01 * scale),
    )
    server.scene.add_frame("/base", axes_length=0.13 * scale, axes_radius=0.003 * scale)
    seed_view = ArmView(server, "/fk_seed", poses, SEED_COLOR, scale, show_tcp=True)
    solution_view = ArmView(server, "/ik_solution", goal_poses, SOLUTION_COLOR, scale, show_tcp=False)
    meshes = SolutionMeshes(server, visual_model, data) if visual_model is not None else None
    target = server.scene.add_transform_controls("/ik_target", scale=0.18 * scale)
    place_frame(target, goal_poses[-1])

    legend = server.gui.add_markdown("")
    mesh_toggle = server.gui.add_checkbox("Show URDF meshes", initial_value=True) if meshes is not None else None

    with server.gui.add_folder("FK seed (degrees)"):
        sliders = []
        for i, joint in enumerate(data["joints"]):
            label = f"J{i + 1} · {joint['name']}"
            lo, hi, start = map(float, (lower_deg[i], upper_deg[i], initial_deg[i]))
            if lo == hi:
                sliders.append(server.gui.add_number(label, start, disabled=True))
            else:
                sliders.append(server.gui.add_slider(
                    label, lo, hi, min(0.1, (hi - lo) / 100), start, marks=()
                ))

    with server.gui.add_folder("Inverse kinematics"):
        xyz = server.gui.add_vector3("Target XYZ (m)", tuple(target.position), step=0.001)
        follow = server.gui.add_checkbox("Solve on edit", initial_value=True)
        copy_fk = server.gui.add_button("Set target to FK")
        solve_button = server.gui.add_button("Solve IK")
        branch = server.gui.add_number("Solution index", 1, min=1, max=1, step=1, disabled=True)
        track = server.gui.add_checkbox("Track selected solution", initial_value=False)
        adopt = server.gui.add_button("Use IK as seed", disabled=True)
        counts = server.gui.add_markdown("")
        details = server.gui.add_markdown("")

    solutions = np.empty((0, 6))
    distances = np.empty(0)
    solved_pose = goal_poses[-1].copy()
    tracked_joints = None
    pending_edit = None
    pending_preview = False
    seed_dirty = False

    def read_seed():
        return np.deg2rad([slider.value for slider in sliders])

    def selected_index():
        return min(max(int(branch.value), 1), len(solutions)) - 1

    def show_solution():
        nonlocal tracked_joints
        if not len(solutions):
            return

        index = selected_index()
        joints = solutions[index]
        if track.value:
            tracked_joints = joints.copy()
        if mesh_toggle is not None and mesh_toggle.value:
            with server.atomic():
                meshes.update(joints)
                solution_view.root.visible = True
            actual = solver.get_fk(joints)
        else:
            current = poses_for(joints)
            solution_view.update(current)
            actual = current[-1]

        translation = np.linalg.norm(actual[:3, 3] - solved_pose[:3, 3])
        relative_rotation = solved_pose[:3, :3].T @ actual[:3, :3]
        rotation = np.rad2deg(np.linalg.norm(tf.SO3.from_matrix(relative_rotation).log()))
        angles = " · ".join(f"{angle:.1f}°" for angle in np.rad2deg(joints))
        details.content = (
            f"Seed distance **{distances[index]:.3f}**  \n"
            f"TCP error **{translation:.3g} m** / **{rotation:.3g}°**  \n"
            f"J1–J6 &nbsp; {angles}"
        )

    def update_display():
        show_meshes = mesh_toggle is not None and mesh_toggle.value
        with server.atomic():
            solution_view.show_skeleton(not show_meshes)
            if meshes is not None:
                meshes.root.visible = show_meshes
            solution_label = "URDF mesh" if show_meshes else "Gold"
            legend.content = (f"`{data['base_link']}` → `{data['tip_link']}`  \n"
                              f"Blue · FK seed &nbsp;&nbsp; {solution_label} · IK solution")
            show_solution()

    def update_results(message, *, preview=False):
        count = len(solutions)
        with server.atomic():
            counts.content = message
            branch.max = max(1, count)
            branch.disabled = preview or count == 0
            adopt.disabled = branch.disabled
            if count:
                if not preview and track.value and tracked_joints is not None:
                    # Compare joint coordinates, including full turns, so a rank
                    # change does not switch a nearby physical configuration.
                    movement = np.abs(solutions - tracked_joints).sum(axis=1)
                    branch.value = int(np.argmin(movement)) + 1
                else:
                    branch.value = 1
                show_solution()
            else:
                branch.value = 1
                solution_view.root.visible = False
                details.content = ""

    def solve_target(*, preview=False):
        nonlocal solutions, distances, solved_pose
        solved_pose = np.eye(4)
        solved_pose[:3, :3] = tf.SO3(np.asarray(target.wxyz)).as_matrix()
        solved_pose[:3, 3] = target.position
        seed = read_seed()
        if preview:
            reference = tracked_joints if track.value and tracked_joints is not None else seed
            nearest = solver.get_nearest_ik(solved_pose, reference)
            if nearest is not None:
                joints, _ = nearest
                solutions = joints[None, :]
                distances = np.array([np.abs(joints - seed).sum()])
                update_results("**Nearest IK preview**", preview=True)
                return
            solutions, distances = np.empty((0, 6)), np.empty(0)
        else:
            solutions, distances = solver.get_ik(solved_pose, seed)
        count = len(solutions)
        if count:
            update_results(f"**{count} IK solution{'s' if count != 1 else ''}**")
        else:
            update_results("**No IK solution within the joint limits**")

    def refresh(*, force_ik=False, preview=False):
        nonlocal pending_edit, seed_dirty, solutions, distances
        if pending_edit is not None:
            pending_edit.cancel()
            pending_edit = None
        if seed_dirty:
            seed_view.update(poses_for(read_seed()))
            seed_dirty = False
        xyz.value = tuple(target.position)
        if force_ik or follow.value:
            solve_target(preview=preview)
        else:
            solutions = np.empty((0, 6))
            distances = np.empty(0)
            update_results("Target or seed changed. Select **Solve IK**.")

    def schedule_refresh(*, preview=False):
        # Keep a single scheduled update and read the newest inputs when it runs.
        nonlocal pending_edit, pending_preview
        pending_preview = preview
        if pending_edit is None:
            pending_edit = asyncio.get_running_loop().call_later(
                EDIT_INTERVAL, lambda: refresh(preview=pending_preview)
            )

    # GUI setters also emit events. Only browser input schedules another edit.
    async def seed_changed(event):
        nonlocal seed_dirty
        if event.client is not None:
            seed_dirty = True
            schedule_refresh()

    for slider in sliders:
        slider.on_update(seed_changed)

    @target.on_update
    async def _(event):
        if event.phase == "update":
            schedule_refresh(preview=True)
        elif event.phase == "end":
            refresh()

    @xyz.on_update
    async def _(event):
        if event.client is not None and np.isfinite(xyz.value).all():
            target.position = xyz.value
            schedule_refresh()

    @follow.on_update
    async def _(event):
        if event.client is not None and (follow.value or pending_edit is not None):
            refresh()

    @copy_fk.on_click
    async def _(_event):
        place_frame(target, solver.get_fk(read_seed()))
        refresh(force_ik=True)

    @solve_button.on_click
    async def _(_event):
        refresh(force_ik=True)

    @branch.on_update
    async def _(event):
        if event.client is not None and len(solutions):
            track.value = True
            show_solution()

    @track.on_update
    async def _(event):
        nonlocal tracked_joints
        if event.client is None:
            return
        if not track.value:
            tracked_joints = None
            branch.value = 1
        show_solution()

    @adopt.on_click
    async def _(_event):
        nonlocal seed_dirty, tracked_joints
        if pending_edit is not None:
            refresh()
        if not len(solutions):
            return
        degrees = np.clip(np.rad2deg(solutions[selected_index()]), lower_deg, upper_deg)
        for slider, angle in zip(sliders, degrees):
            slider.value = float(angle)
        track.value = False
        tracked_joints = None
        seed_dirty = True
        refresh(force_ik=True)

    if mesh_toggle is not None:
        @mesh_toggle.on_update
        async def _(_event):
            update_display()

    update_display()
    solve_target()


def main():
    parser, args = parse_args()
    if args.python_dir is not None:
        if not args.python_dir.is_dir():
            parser.error(f"Python module directory does not exist: {args.python_dir}")
        sys.path.insert(0, str(args.python_dir.resolve()))
    try:
        import opw_kinematics as opw
    except ImportError as error:
        parser.error(f"cannot import opw_kinematics ({error}). Build the Python bindings and pass their directory with --python-dir.")

    try:
        data = load_chain(args, Path(opw.__file__))
        solver = opw.OPWKinematics.from_urdf_data(data)
        visual_model = load_visual_model(args.urdf.resolve()) if args.meshes else None
    except (AttributeError, KeyError, TypeError, ValueError, OSError, subprocess.SubprocessError) as error:
        parser.error(f"cannot load robot chain: {error}")

    server = viser.ViserServer(host=args.host, port=args.port, label="OPWKinematics")
    try:
        show_kinematics(server, solver, data, visual_model)
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
