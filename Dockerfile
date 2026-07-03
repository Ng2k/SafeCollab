# SafeCollab — perception-driven, human-aware collaborative kitting cell.
# Base image and dependency set are fixed by AGENTS.md §8.
FROM ros:jazzy-ros-base

# Package documentation is pruned in the SAME RUN as each apt install below
# (see the trailing `rm -rf /usr/share/{doc,man,info}`). This must be in-layer:
# the ROS/gz/MoveIt debs ship ~156 MB of docs — 152 MB of it duplicated package
# `copyright` files (boost alone bundles its whole 2.1 MB licence set in every
# one of its 20+ packages). The base image's dpkg excludes re-includes copyright
# and out-sorts any config we add, so a config-based exclude doesn't win; and a
# post-hoc `rm` in a later RUN only writes a union-fs whiteout, leaving the bytes
# in the earlier layer. Deleting inside the install RUN is what actually shrinks
# the image. (Source packages / apt retain the full licences.)

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
#   - cv-bridge ...................... sensor_msgs/Image <-> cv2 in perception_node
#   - python3-pytest(-cov) ........... unit test + coverage gate (>= 90%)
#   - launch-testing(-ros) ........... headless integration bring-up harness (§7.4)
#
# This image is the single source of truth for the application dependency set:
# the CI integration stage runs the headless launch_test INSIDE this image
# (not a separately-maintained apt list), so a package present here but missing
# in CI — or vice-versa — cannot happen.
# ---------------------------------------------------------------------------
# gz Harmonic Python bindings (gz.transport13 / gz.msgs10) live in the OSRF gz
# apt repo, not the ROS one (the ROS image ships gz only as C++ vendor packages:
# ros-jazzy-gz-{transport,msgs}-vendor — no Python module). human_node uses them
# to move the yellow operator body via the /world/empty/set_pose service. Add the
# OSRF repo here so the package install below can pull the matching Python debs
# (13.5.0 / 10.x — same versions as the vendored libs, so dpkg reports 0 removals
# and no file conflicts).
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc/* /usr/share/man/* /usr/share/info/*

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
        ros-jazzy-cv-bridge \
        python3-opencv \
        python3-pytest \
        python3-pytest-cov \
        python3-gz-transport13 \
        python3-gz-msgs10 \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# ---------------------------------------------------------------------------
# Universal Robots UR5e model + gz simulation wiring (real manipulator, replaces
# the hand-built cylinder arm). ur_description provides the URDF/meshes and solved
# kinematics; ur_simulation_gz wires the UR into gz via gz_ros2_control (the same
# mechanism this cell already uses) — its ur_gz.ros2_control.xacro is included by
# cell.xacro. Joint names (shoulder_pan_joint … wrist_3_joint) already match
# config/controllers.yaml and the motion pipeline, so the SSM loop is unchanged.
# (MoveIt planning packages are added in a later layer.)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ur-description \
        ros-jazzy-ur-simulation-gz \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# ---------------------------------------------------------------------------
# MoveIt 2 + the deterministic Pilz industrial motion planner + moveit_py, plus
# the UR MoveIt config (SRDF/kinematics/limits reused for our cell). planner_node
# uses MoveItPy + Pilz (PTP/LIN) to generate the nominal kitting trajectory that
# motion_node then retimes for SSM speed scaling. Kept in a separate layer (large)
# so the UR layer above stays cached across rebuilds.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-moveit-py \
        ros-jazzy-pilz-industrial-motion-planner \
        ros-jazzy-ur-moveit-config \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc/* /usr/share/man/* /usr/share/info/*
# python3-gz-transport13 / python3-gz-msgs10 (from the OSRF repo added above)
# give human_node._set_gz_pose() the gz.transport13 / gz.msgs10 modules it needs
# to move the yellow operator body via /world/empty/set_pose, so the overhead
# camera sees the operator track its path and perception_node can detect it.
# The import in human_node stays guarded (try/except) so the node still runs if
# the bindings are ever absent — it just falls back to a static operator body.

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
