# cell.launch.py — SafeCollab integrated cell launch (Stream G, §6)
#
# This is the single entry point for running the complete SafeCollab kitting cell.
# It is the integration point that wires together all streams.
#
# What is wired (the full closed-loop cell):
#   1. gz sim            — headless server or full GUI, controlled by headless:=true
#   2. robot_state_publisher — cell.xacro via Command(['xacro ', ...]) (Stream A)
#   3. ros_gz_sim create — spawn the model into gz (Stream A)
#   4. joint_state_broadcaster + arm_controller spawners — ordered via OnProcessExit (Stream A)
#   5. ros_gz_image bridge — gz camera/image -> ROS /camera/image (Stream C input)
#   6. ros_gz_bridge       — gz clock -> ROS /clock (sim time for every node)
#   7. task_node         — kitting state machine + nominal trajectory publisher (Stream E)
#   8. human_node        — ground-truth operator model, broadcasts world->human_gt TF (Stream E)
#   9. motion_node       — fuses nominal trajectory * safety scale, commands arm (Stream D)
#  10. perception_node   — camera -> perceived world->human TF + /human/uncertainty (Stream C)
#  11. safety_monitor    — min-distance -> /safety/scale + /safety/zone, fail-safe (Stream F)
#
# The safety loop is closed: perception_node -> safety_monitor -> /safety/scale -> motion_node.
#
# headless:=true is required by CI (§7 integration and package stages).  See §8:
#   ros2 launch safecollab cell.launch.py headless:=true
#
# Standard interactive run (requires a display):
#   ros2 launch safecollab cell.launch.py
#
# Quick introspection after launch:
#   ros2 control list_controllers          # joint_state_broadcaster + arm_controller active
#   ros2 run tf2_tools view_frames         # world->tcp chain and world->human_gt
#   ros2 topic hz /camera/image            # ~30 Hz from the gz camera bridge
#   ros2 topic echo /task/state            # kitting SM state
#   ros2 topic echo /motion/nominal_trajectory  # trajectory from task_node
#
# Reference: _stream_a_smoke_test.launch.py was the provisional scaffold this file replaces.
# It is kept for Stream A's own smoke-test verification and is not run by CI.

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
            "Integer seed for the operator's random path in human_node.py. "
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

    # ------------------------------------------------------------------
    # Gazebo simulation — §6 item 1 prerequisite
    # Two variants, exactly one runs per invocation (IfCondition / UnlessCondition).
    #   headless=true  -> -s  server-only, no GUI window; safe for CI / Xvfb-less Docker
    #   headless=false -> server + GUI client (default for interactive development)
    # Both use -r (run immediately, not paused) to match the smoke-test invocation.
    # ------------------------------------------------------------------

    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    # The world is the project-owned worlds/cell.sdf, NOT the stock empty.sdf.
    # empty.sdf does not load the gz Sensors system, so the cell camera never
    # renders and /camera/image stays silent (perception goes blind, safety
    # fail-safes to 'lost'). cell.sdf is empty.sdf + the Sensors system; see
    # the header of worlds/cell.sdf and docs/VALIDATE.md.
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

    # ------------------------------------------------------------------
    # robot_state_publisher — §6 item 1
    # Parameterised with Command(['xacro ', cell_xacro]) exactly as documented in §6
    # and as the smoke test (_stream_a_smoke_test.launch.py) already verified works.
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # ros_gz_sim create — §6 item 2 (spawn)
    # Spawns the model from the /robot_description topic into the running gz server.
    # The model name matches the robot element in cell.xacro: name="safecollab_cell".
    # ------------------------------------------------------------------

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "safecollab_cell"],
    )

    # ------------------------------------------------------------------
    # Operator visual body spawn — closes the perception loop (Stream E / C)
    #
    # Spawns urdf/operator.sdf: a static yellow 0.25×0.25×0.30 m box.
    # Yellow RGB (1,1,0) → OpenCV HSV hue ≈ 30, inside detect_human()'s
    # default hue_low=20..hue_high=40 window (perception_node.py line ~192).
    # Initial pose at the operator path start-position (first waypoint).
    #
    # human_node moves the entity at 10 Hz via the gz transport
    # /world/empty/set_pose service so the camera sees the body track the
    # ground-truth operator path.  The world->human_gt TF (ground truth)
    # is broadcast separately at 50 Hz and is unaffected by this spawn.
    #
    # headless-safe: `ros_gz_sim create` uses a gz transport service call
    # internally; it does not require a GUI and works with -s (server-only).
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
            # Start near the first waypoint's X/Y (x=0.9) but at the constant
            # marker height z=_MARKER_Z=0.95 m (human_node holds the marker at
            # this plane height — see human_node._MARKER_Z / operator.sdf).
            # human_node repositions it on the first gz-transport tick (~100 ms).
            "-x",
            "0.9",
            "-y",
            "0.0",
            "-z",
            "0.95",
        ],
    )

    # ------------------------------------------------------------------
    # Controller spawners — §6 item 3
    # Ordered via OnProcessExit to guarantee the gz_ros2_control controller_manager
    # (started by the GazeboSimROS2ControlPlugin in cell.xacro) is already running
    # before we try to load controllers.  Chain: spawn -> jsb_spawner -> arm_spawner.
    # Config is in config/controllers.yaml (§5) loaded by the gz plugin.
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
    # ros_gz_image bridge — §6 item 4 (camera)
    # The gz camera sensor in cell.xacro publishes on gz topic 'camera/image'
    # (set via <topic>camera/image</topic> in urdf/cell.xacro, line ~143).
    # The ros_gz_image image_bridge maps:
    #   gz transport topic:  camera/image
    #   ROS 2 topic:        /camera/image   (sensor_msgs/Image, best-effort, §3 contract)
    #
    # This is the specific piece blocking Stream C live perception wiring:
    # perception_node subscribes /camera/image; without this bridge the topic is dead.
    # ------------------------------------------------------------------

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/image"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # ros_gz_bridge /clock bridge — simulation time
    # gz sim publishes the simulation clock on the gz transport topic 'clock';
    # without bridging it to the ROS '/clock' topic, every node started with
    # use_sim_time:=true (the controllers, task/human/motion_node) has no clock
    # source. The controller_manager then logs "No clock received, using time
    # argument instead!" every cycle, and ROS-side sim time never advances.
    # This one-way (gz -> ROS) bridge publishes /clock so all nodes share sim
    # time. Direction token '[' = gz -> ROS only (see ros_gz_bridge README).
    # ------------------------------------------------------------------

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Application nodes — §6 item 5
    # Nodes present on main (v0.2.0) are launched directly.
    # Nodes not yet on main are left as clearly commented PLACEHOLDER blocks —
    # NOT silently broken references (per §6 ownership note).
    # ------------------------------------------------------------------

    # Stream E: task_node — owns the kitting state machine (GO_TO_BIN -> PICK -> ...),
    # publishes /motion/nominal_trajectory (JointTrajectory) and /task/state (String).
    task_node = Node(
        package="safecollab",
        executable="task_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Stream E: human_node — drives the simulated operator along randomisable paths
    # including tray-reaches (FR-11 / AT-6).  Broadcasts world->human_gt TF
    # (ground truth, sim-internal; never consumed by the safety loop per §3).
    # path_seed: 0 (default) = non-deterministic; >0 = reproducible for AT tests.
    human_node = Node(
        package="safecollab",
        executable="human_node",
        output="screen",
        parameters=[{"use_sim_time": True, "path_seed": path_seed}],
    )

    # Stream D: motion_node — fuses nominal trajectory * /safety/scale, re-times
    # trajectory points (retime()), handles protective stop (scale==0) and clean
    # resume. Publishes /arm_controller/joint_trajectory.
    motion_node = Node(
        package="safecollab",
        executable="motion_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # Stream C: perception_node — closes the input half of the safety loop.
    # Subscribes /camera/image -> classical CV -> broadcasts the *perceived*
    # world->human TF + publishes /human/uncertainty (Float32, σ m). On loss
    # (no detection within the timeout) it stops broadcasting the human TF,
    # which trips safety_monitor's fail-safe (FR-9 / AT-5).
    # NOTE: until the operator is given a camera-visible model in gz (human_node
    # currently only broadcasts the world->human_gt ground-truth TF, no visual),
    # perception sees no blob and reports lost — so the loop sits in the
    # protective-stop fail-safe. That is correct safe behaviour; demonstrating
    # the green/yellow/red ramp needs a visible operator (next task).
    # ------------------------------------------------------------------

    perception_node = Node(
        package="safecollab",
        executable="perception_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # Stream F: safety_monitor — closes the loop. Reads the perceived world->human
    # TF + /human/uncertainty (σ), sweeps min separation over robot frames
    # (tcp, link_6, link_3 — see config/safety.yaml), calls safety_logic.classify()
    # and publishes /safety/scale (Float32 0..1) + /safety/zone (green|yellow|red|
    # lost). Fail-safe: stale/absent human TF -> ("lost", 0.0) protective stop.
    # ------------------------------------------------------------------

    safety_monitor = Node(
        package="safecollab",
        executable="safety_monitor",
        output="screen",
        parameters=[{"use_sim_time": True, "safety_source": safety_source}],
    )

    # ------------------------------------------------------------------
    # P5: RViz — only when rviz:=true (default false keeps CI/headless clean).
    # Loads config/view.rviz (RobotModel + Camera + safety-zone Marker display)
    # so the green/yellow/red/lost state is legible at a glance.
    # ------------------------------------------------------------------

    rviz_config = PathJoinSubstitution([pkg, "config", "view.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            headless_arg,
            safety_source_arg,
            path_seed_arg,
            rviz_arg,
            # gz sim: exactly one of these two runs depending on headless argument
            gz_server,  # headless=true  -> server-only (CI / no display)
            gz_full,  # headless=false -> server + GUI (interactive)
            # /clock bridge first: sim time must be available before the
            # controllers and use_sim_time nodes start, or they warn every cycle.
            clock_bridge,
            robot_state_publisher,
            spawn,
            # Spawn the yellow operator visual body immediately after the robot.
            # ros_gz_sim create retries internally until gz sim is ready, so no
            # explicit ordering constraint is needed.  human_node repositions it
            # via gz transport set_pose on the first tick.
            spawn_operator,
            # controllers are ordered: spawn -> jsb_spawner -> arm_spawner
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
            ),
            camera_bridge,
            # --- application nodes ---
            task_node,
            human_node,
            motion_node,
            # --- safety loop: perception -> safety_monitor -> motion scale ---
            perception_node,  # Stream C
            safety_monitor,  # Stream F
            # --- P5 visualisation (only when rviz:=true) ---
            rviz_node,
        ]
    )
