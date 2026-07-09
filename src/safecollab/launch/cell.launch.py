# cell.launch.py — SafeCollab integrated cell launch (Stream G, §6). The single
# entry point that wires the full closed-loop cell:
#   1. gz sim (headless server or GUI, per headless:=)
#   2. robot_state_publisher — cell.xacro via Command(['xacro ', ...])
#   3. ros_gz_sim create — spawn the model (+ the yellow operator body) into gz
#   4. joint_state_broadcaster + arm_controller spawners (ordered via OnProcessExit)
#   5. ros_gz_image bridge — gz camera/image -> ROS /camera/image
#   6. ros_gz_bridge — gz clock -> ROS /clock (sim time for every node)
#   7. planner_node — MoveIt/Pilz pick-and-place; nominal trajectory publisher
#   8. human_node — ground-truth operator model, broadcasts world->human_gt TF
#   9. motion_node — fuses nominal trajectory * safety scale, commands the arm
#  10. perception_node — camera -> perceived world->human TF + /human/uncertainty
#  11. safety_monitor — min-distance -> /safety/scale + /safety/zone, fail-safe
#
# Safety loop: perception_node -> safety_monitor -> /safety/scale -> motion_node.
# CI (§7) uses headless:=true; the plain command runs interactively with a display:
#   ros2 launch safecollab cell.launch.py [headless:=true]

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _moveit_params() -> dict:
    """Assemble the MoveIt config for planner_node's MoveItPy (Pilz pick-and-place).

    Built here (not inside the node) so the nested planning-pipeline parameters are
    declared as ROS node parameters — the only reliable way to get MoveItCpp to
    load the Pilz planner. Returns the flat param dict merged onto planner_node.
    """
    import os

    from ament_index_python.packages import get_package_share_directory
    from moveit_configs_utils import MoveItConfigsBuilder

    share = get_package_share_directory("safecollab")
    cfg = (
        MoveItConfigsBuilder("safecollab_cell", package_name="safecollab")
        .robot_description(file_path=os.path.join(share, "urdf", "cell.xacro"))
        .robot_description_semantic(
            file_path=os.path.join(share, "srdf", "cell.srdf.xacro")
        )
        .robot_description_kinematics(
            file_path=os.path.join(share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=os.path.join(share, "config", "joint_limits.yaml"))
        .pilz_cartesian_limits(
            file_path=os.path.join(share, "config", "pilz_cartesian_limits.yaml")
        )
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )
    params = cfg.to_dict()
    # to_dict() emits planning_pipelines as a FLAT list (what the move_group node
    # reads), but MoveItCpp — which MoveItPy uses — reads planning_pipelines.
    # pipeline_names. Reshape so the Pilz pipeline actually loads.
    names = params.pop("planning_pipelines")
    params["planning_pipelines"] = {
        "pipeline_names": names,
        "default_planning_pipeline": params.get("default_planning_pipeline", names[0]),
    }
    return params


def generate_launch_description():
    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description=(
            "Run gz sim in headless (server-only, no GUI) mode. "
            "Set to true for CI, Docker without a display, or launch_test (§7)."
        ),
    )
    headless = LaunchConfiguration("headless")

    safety_source_arg = DeclareLaunchArgument(
        "safety_source",
        default_value="perceived",
        description=(
            "TF frame source for the safety loop (AGENTS.md §11 cut-scope fallback #4). "
            "'perceived' (default): world→human from perception_node — headline demo path. "
            "'ground_truth': world→human_gt from human_node — fallback when perception "
            "is unavailable, so the SSM ramp/stop/resume/fail-safe can be demonstrated."
        ),
    )
    safety_source = LaunchConfiguration("safety_source")

    path_seed_arg = DeclareLaunchArgument(
        "path_seed",
        default_value="0",
        description=(
            "Integer seed for the operator's random path in human_node. "
            "0 (default): unseeded — non-deterministic, for the interactive demo. "
            ">0: seeded — deterministic, for the scenario/acceptance test harness "
            "(AT-1..AT-5 in test/scenario/test_at1_at5.py). "
            "Example: ros2 launch safecollab cell.launch.py path_seed:=42"
        ),
    )
    path_seed = LaunchConfiguration("path_seed")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description=(
            "Launch RViz with config/view.rviz to visualise the cell: robot model, "
            "camera image, and the /viz/safety_marker zone sphere + text label "
            "(P5). Default false so CI/headless and the scenario harness are "
            "unaffected. Example: ros2 launch safecollab cell.launch.py rviz:=true"
        ),
    )
    rviz = LaunchConfiguration("rviz")

    hud_arg = DeclareLaunchArgument(
        "hud",
        default_value="false",
        description=(
            "Launch the console safety HUD (hud_node): a compact, colour-coded "
            "terminal readout of zone / speed scale / min-distance (P5). Default "
            "false so CI/headless and the scenario harness are unaffected. "
            "Example: ros2 launch safecollab cell.launch.py hud:=true"
        ),
    )
    hud = LaunchConfiguration("hud")

    human_arg = DeclareLaunchArgument(
        "human",
        default_value="true",
        description=(
            "Spawn the operator body and human_node. false: robot-only cell (no "
            "operator, no ground-truth path) — use with safety:=false to show the "
            "full pick-and-place at full speed."
        ),
    )
    human = LaunchConfiguration("human")

    safety_arg = DeclareLaunchArgument(
        "safety",
        default_value="true",
        description=(
            "Run the safety loop (perception_node + safety_monitor → /safety/scale). "
            "false: no SSM scaling, so motion_node runs every planned leg at scale "
            "1.0 (full speed). Set false only alongside human:=false."
        ),
    )
    safety = LaunchConfiguration("safety")

    # ------------------------------------------------------------------
    # Gazebo simulation — exactly one variant runs (If/UnlessCondition). Both use
    # -r (run immediately). headless=true -> -s server-only (CI / no display);
    # headless=false -> server + GUI (interactive).
    # ------------------------------------------------------------------

    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    # The world is worlds/cell.sdf (empty.sdf + the gz Sensors system), NOT the
    # stock empty.sdf — without Sensors the cell camera never renders and the
    # safety loop fail-safes to 'lost'. See the cell.sdf header.
    cell_world = PathJoinSubstitution(
        [FindPackageShare("safecollab"), "worlds", "cell.sdf"]
    )

    # Headless: --headless-rendering makes the Sensors system render offscreen
    # via EGL (no X display needed), so the camera produces frames in CI/Docker.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": ["-s -r --headless-rendering ", cell_world]
        }.items(),
        condition=IfCondition(headless),
    )

    gz_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r ", cell_world]}.items(),
        condition=UnlessCondition(headless),
    )

    # robot_state_publisher — parameterised with Command(['xacro ', cell_xacro]).
    pkg = FindPackageShare("safecollab")

    cell_xacro = PathJoinSubstitution([pkg, "urdf", "cell.xacro"])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", cell_xacro]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ros_gz_sim create — spawn the model from /robot_description into gz. The
    # name matches cell.xacro's <robot name="safecollab_cell">.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "safecollab_cell"],
    )

    # ------------------------------------------------------------------
    # Operator visual body spawn — closes the perception loop (Stream E / C).
    # Spawns urdf/operator.sdf (a static yellow puck detect_human sees) at the
    # operator path start. human_node then moves it via gz /world/empty/set_pose
    # so the camera tracks the ground-truth path; the world->human_gt TF is
    # broadcast separately at 50 Hz. `ros_gz_sim create` is headless-safe (a gz
    # transport service call, no GUI).
    # ------------------------------------------------------------------

    operator_sdf = PathJoinSubstitution([pkg, "urdf", "operator.sdf"])

    spawn_operator = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            operator_sdf,
            "-name",
            "operator",
            # Start near the first waypoint's X/Y (x=0.9) at the constant marker
            # height z=_MARKER_Z=0.95 m; human_node repositions it on the first
            # gz-transport tick (~100 ms).
            "-x",
            "0.9",
            "-y",
            "0.0",
            "-z",
            "0.95",
        ],
        condition=IfCondition(human),
    )

    # ------------------------------------------------------------------
    # Controller spawners, ordered via OnProcessExit (spawn -> jsb -> arm) so the
    # gz_ros2_control controller_manager (from cell.xacro's plugin) is up before
    # we load controllers. Config: config/controllers.yaml.
    # ------------------------------------------------------------------

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # ros_gz_image bridge: gz topic camera/image -> ROS /camera/image
    # (sensor_msgs/Image). perception_node subscribes /camera/image; without this
    # bridge the topic is dead.
    # ------------------------------------------------------------------

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/image"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # /clock bridge (gz -> ROS, one-way; the '[' token = gz -> ROS only). Every
    # use_sim_time node needs it, else ROS-side sim time never advances and the
    # controller_manager logs "No clock received" each cycle.
    # ------------------------------------------------------------------

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Application nodes
    # ------------------------------------------------------------------

    # Stream D': planner_node — MoveIt/Pilz pick-and-place for the UR5e. Plans the
    # kitting cycle as Pilz PTP segments and publishes each as
    # /motion/nominal_trajectory (dense position JointTrajectory) + /task/state;
    # motion_node retimes it for SSM speed scaling.
    planner_node = Node(
        package="safecollab",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[_moveit_params(), {"use_sim_time": True}],
    )

    # Stream E: human_node — drives the simulated operator along randomised paths
    # with tray reaches (FR-11 / AT-6), broadcasting the ground-truth
    # world->human_gt TF. path_seed 0 = non-deterministic; >0 = reproducible (AT).
    human_node = Node(
        package="safecollab",
        executable="human_node",
        output="screen",
        parameters=[{"use_sim_time": True, "path_seed": path_seed}],
        condition=IfCondition(human),
    )

    # Stream D: motion_node — fuses nominal trajectory * /safety/scale via
    # retime(), handles protective stop (scale==0) and clean resume, and publishes
    # /arm_controller/joint_trajectory.
    motion_node = Node(
        package="safecollab",
        executable="motion_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # Stream C: perception_node — the input half of the safety loop. Subscribes
    # /camera/image -> classical CV -> broadcasts the *perceived* world->human TF
    # + /human/uncertainty (σ). On loss (no detection within the timeout) it stops
    # broadcasting the human TF, tripping safety_monitor's fail-safe (FR-9 / AT-5).
    # ------------------------------------------------------------------

    perception_node = Node(
        package="safecollab",
        executable="perception_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(safety),
    )

    # ------------------------------------------------------------------
    # Stream F: safety_monitor — closes the loop. Reads the perceived world->human
    # TF + σ, sweeps min separation over the robot frames (config/safety.yaml),
    # classifies, and publishes /safety/scale (0..1) + /safety/zone. Fail-safe:
    # stale/absent human TF -> ("lost", 0.0) protective stop.
    # ------------------------------------------------------------------

    safety_monitor = Node(
        package="safecollab",
        executable="safety_monitor",
        output="screen",
        parameters=[{"use_sim_time": True, "safety_source": safety_source}],
        condition=IfCondition(safety),
    )

    # P5: RViz (only when rviz:=true) — loads config/view.rviz (RobotModel +
    # Camera + safety-zone Marker) so the zone state is legible at a glance.
    rviz_config = PathJoinSubstitution([pkg, "config", "view.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    # P5: console safety HUD (only when hud:=true) — a view-only node printing the
    # live zone / scale / min-distance one-liner to the terminal.
    hud_node = Node(
        package="safecollab",
        executable="hud_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(hud),
    )

    return LaunchDescription(
        [
            headless_arg,
            safety_source_arg,
            path_seed_arg,
            rviz_arg,
            hud_arg,
            human_arg,
            safety_arg,
            # gz sim: exactly one runs, per headless.
            gz_server,  # headless=true  -> server-only (CI / no display)
            gz_full,  # headless=false -> server + GUI (interactive)
            # /clock bridge first: sim time must exist before use_sim_time nodes start.
            clock_bridge,
            robot_state_publisher,
            spawn,
            # Operator body — create retries until gz is ready (no ordering needed);
            # human_node repositions it via set_pose on the first tick.
            spawn_operator,
            # controllers ordered: spawn -> jsb_spawner -> arm_spawner
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
            ),
            camera_bridge,
            # --- application nodes ---
            planner_node,
            human_node,
            motion_node,
            # --- safety loop: perception -> safety_monitor -> motion scale ---
            perception_node,  # Stream C
            safety_monitor,  # Stream F
            # --- P5 visualisation (only when rviz:=true / hud:=true) ---
            rviz_node,
            hud_node,
        ]
    )
