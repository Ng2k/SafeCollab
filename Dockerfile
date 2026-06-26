# SafeCollab — perception-driven, human-aware collaborative kitting cell.
# Base image and dependency set are fixed by AGENTS.md §8.
FROM ros:jazzy-ros-base

# ---------------------------------------------------------------------------
# System & ROS dependencies (AGENTS.md §8)
#   - ros-gz-sim ..................... Gazebo (gz) simulation
#   - ros-gz-image ................... gz <-> ros2 camera/image bridge (perception)
#   - ros-gz-bridge .................. gz <-> ros2 /clock bridge (sim time)
#   - gz-ros2-control ................ gz <-> ros2_control bridge
#   - ros2-control / ros2-controllers  the control stack
#   - joint-trajectory-controller .... arm controller used by motion_node
#   - xacro .......................... expands cell.xacro for robot_state_publisher
#   - robot-state-publisher .......... publishes robot_description / TF in cell.launch.py
#   - python3-opencv ................. classical CV for perception_node
#   - python3-pytest(-cov) ........... unit test + coverage gate (>= 90%)
#   - launch-testing(-ros) ........... headless integration bring-up harness (§7.4)
#
# This image is the single source of truth for the application dependency set:
# the CI integration stage runs the headless launch_test INSIDE this image
# (not a separately-maintained apt list), so a package present here but missing
# in CI — or vice-versa — cannot happen.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-image \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-launch-testing \
        ros-jazzy-launch-testing-ros \
        python3-opencv \
        python3-pytest \
        python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

# Colcon workspace root. The repository root *is* the workspace: packages live
# under ./src (see AGENTS.md §5), so COPY . places them at ${ROS_WS}/src.
ENV ROS_WS=/opt/safecollab_ws
WORKDIR ${ROS_WS}

# Copy the whole repository into the workspace and build the overlay from
# scratch. The repo root is the workspace, so packages already sit under
# ./src (e.g. src/safecollab). `colcon build` exits 0 even when src/ has no
# packages yet, so the image builds cleanly before the application streams
# land (AGENTS.md §6, Stream G).
COPY . ${ROS_WS}/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install

# entrypoint sources /opt/ros/jazzy and the overlay, then exec's the CMD.
# Strip any CR (the repo may be checked out on Windows with core.autocrlf=true);
# a CRLF shebang would make the Linux loader fail with "bad interpreter".
COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh \
    && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default: launch the full cell headless (no GUI) — overridable at `docker run`.
CMD ["ros2", "launch", "safecollab", "cell.launch.py", "headless:=true"]
