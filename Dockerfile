# syntax=docker/dockerfile:1
# SafeCollab kitting cell. Deps fixed by AGENTS.md §8; CI runs its
# integration/scenario stages inside this image (single source of truth).

# ============================== builder ==============================
# Build the colcon overlay in isolation. safecollab is a pure-Python ament
# package, so this needs colcon (already in ros-base) but neither the gz/MoveIt
# runtime deps nor a compiler. Its output (install/ + the tiny build/ hooks) is
# copied into the runtime, which therefore never ships build-essential/git/colcon
# (~80 MB). Uses --symlink-install (as the app expects — nodes resolve config/
# relative to their module, i.e. back into src/, which the runtime also carries).
FROM ros:jazzy-ros-base AS builder
ENV ROS_WS=/opt/safecollab_ws
WORKDIR ${ROS_WS}
COPY . ${ROS_WS}/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install

# ============================== runtime ==============================
# ros-core, not ros-base: it has the ROS runtime (rclpy, ros2 CLI) without the
# build toolchain the overlay above was already built with.
FROM ros:jazzy-ros-core

# Keep downloaded .debs so the BuildKit cache mounts below survive rebuilds (the
# base image's docker-clean hook otherwise wipes them post-install); a rebuilt
# apt layer then unpacks from cache instead of re-fetching.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
        > /etc/apt/apt.conf.d/keep-cache

# Every apt RUN below caches /var/cache/apt + the apt lists (outside the image)
# and prunes doc/man/info IN-LAYER — a later `rm` only whiteouts the bytes. Docs
# are ~156 MB, nearly all duplicated `copyright` files (boost's 2.1 MB per deb).

# OSRF gz repo: the ROS image has gz only as C++ vendor packages, so the
# gz.transport13/gz.msgs10 Python bindings human_node needs (to move the operator
# via /world/empty/set_pose) come from here.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# ROS/gz stack: sim, gz<->ros2 bridges, ros2_control + arm controller,
# robot_state_publisher/xacro, perception (opencv/cv-bridge), test harnesses.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
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
    && rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# UR5e model + its gz_ros2_control wiring (ur_gz.ros2_control.xacro is included
# by cell.xacro); UR joint names already match config/controllers.yaml.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ur-description \
        ros-jazzy-ur-simulation-gz \
    && rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# MoveIt + Pilz + moveit_py + the UR MoveIt config: planner_node plans the
# nominal kitting trajectory (PTP/LIN) that motion_node retimes for SSM scaling.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-moveit-py \
        ros-jazzy-pilz-industrial-motion-planner \
        ros-jazzy-ur-moveit-config \
    && rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/info/*

# Bring the sources (in-image tests run from ${ROS_WS}/src; the symlink overlay
# also resolves back into it) plus the overlay (install/) and its develop hooks
# (build/) from the builder.
ENV ROS_WS=/opt/safecollab_ws
WORKDIR ${ROS_WS}
COPY . ${ROS_WS}/
COPY --from=builder ${ROS_WS}/install ${ROS_WS}/install
COPY --from=builder ${ROS_WS}/build ${ROS_WS}/build

# Strip any CRLF so a Windows checkout can't break the shebang.
COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh \
    && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "safecollab", "cell.launch.py", "headless:=true"]
