# _stream_a_smoke_test.launch.py
#
# PROVISIONAL SCAFFOLD - NOT the final launch file.
#
# This exists only to verify the Stream A exit gate (arm + cell spawn, controllers
# active, clean TF) in a headless gz sim. It deliberately does the bare minimum:
#   * robot_state_publisher  (cell.xacro via `xacro` Command)
#   * ros_gz_sim `create`    (spawn the model into a running gz server)
#   * spawner                joint_state_broadcaster
#   * spawner                arm_controller
#
# It intentionally OMITS the camera ros_gz bridge and every application node.
# The real, shared launch/cell.launch.py (camera bridge, headless:=true, all
# application nodes) is owned jointly once the other streams are ready - do not
# grow this file into that.
#
# Run (inside a ROS 2 Jazzy + gz environment):
#   gz sim -s -r empty.sdf            # headless gz server in another shell
#   ros2 launch safecollab _stream_a_smoke_test.launch.py
#   ros2 control list_controllers     # -> joint_state_broadcaster, arm_controller active
#   ros2 run tf2_tools view_frames    # -> single tree rooted at world, reaches tcp

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("safecollab")

    cell_xacro = PathJoinSubstitution([pkg, "urdf", "cell.xacro"])
    # wrap as str so launch does not try to parse the URDF XML as YAML
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

    # spawn the model into the (already running) gz server from /robot_description
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "safecollab_cell"],
    )

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

    # order the controller spawners after the model is created so the
    # gz_ros2_control controller_manager is up before we load controllers
    return LaunchDescription(
        [
            robot_state_publisher,
            spawn,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
            ),
        ]
    )
