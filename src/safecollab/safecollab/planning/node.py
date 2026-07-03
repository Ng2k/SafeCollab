"""Planner ROS runtime — MoveIt/Pilz plans each leg, publishes the trajectory.

Live-only (``# pragma: no cover``): resolves each Cartesian target to a cached
joint config, plans a Pilz PTP, publishes ``/motion/nominal_trajectory`` +
``/task/state``, and paces the cycle closed-loop against ``/joint_states``.

Publishes: /motion/nominal_trajectory (JointTrajectory), /task/state (String).
"""

from __future__ import annotations

import math
import os
import time

from safecollab.planning.plan import KittingLeg, kitting_legs


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory

    from moveit.planning import MoveItPy, PlanRequestParameters
    from moveit.core.robot_state import RobotState

    _PLAN_GROUP = "ur_manipulator"
    _EEF_LINK = "tool0"
    _PLAN_FRAME = "world"
    # tool0 approach axis straight DOWN: 180° about world x → (w,x,y,z)=(0,1,0,0).
    _DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)

    # MoveItPy reads the MoveIt config (robot_description[_semantic], kinematics,
    # planning pipelines) from THIS node's ROS params, which cell.launch.py supplies
    # via MoveItConfigsBuilder(...).to_dict() — that is what declares the nested
    # pipeline params so MoveItCpp can load the Pilz planner.
    rclpy.init(args=args)
    moveit = MoveItPy(node_name="planner_node")
    arm = moveit.get_planning_component(_PLAN_GROUP)
    robot_model = moveit.get_robot_model()

    # A separate lightweight node carries the publishers (MoveItPy owns its node).
    pub = Node("planner_pub")
    traj_pub = pub.create_publisher(JointTrajectory, "/motion/nominal_trajectory", 10)
    state_pub = pub.create_publisher(String, "/task/state", 10)
    log = pub.get_logger()

    # Closed-loop pacing needs the live joint state. /joint_states is alphabetical;
    # index it BY NAME into the planning-group order so the goal comparison is
    # order-correct (same pitfall motion_node handles). The planner waits until the
    # arm reaches each leg goal before advancing, so an SSM slow-down only DELAYS a
    # pick, never abandons it (the old open-loop wall-clock pacing stopped short).
    _JOINT_ORDER = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    _latest_js: dict[str, float] = {}

    def _on_joint_states(msg: "JointState") -> None:
        for name, position in zip(msg.name, msg.position):
            _latest_js[name] = position

    pub.create_subscription(JointState, "/joint_states", _on_joint_states, 10)

    def current_joints() -> list[float] | None:
        if not all(name in _latest_js for name in _JOINT_ORDER):
            return None
        return [_latest_js[name] for name in _JOINT_ORDER]

    def wait_until_reached(goal: list[float], timeout_s: float) -> bool:
        # True once every joint is within _REACH_TOL of the goal, or False on
        # timeout (e.g. a long protective stop). Spins pub so the JS callback fires.
        _REACH_TOL = 0.05  # rad (~3° per joint)
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(pub, timeout_sec=0.05)
            cur = current_joints()
            if (
                cur is not None
                and max(abs(a - b) for a, b in zip(cur, goal)) < _REACH_TOL
            ):
                return True
        return False

    log.info("[planner_node] MoveItPy up; planning group ur_manipulator")

    def pose_goal(xyz: tuple[float, float, float]) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = _PLAN_FRAME
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        (
            p.pose.orientation.w,
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
        ) = _DOWN_QUAT
        return p

    # Deterministic IK: KDL is non-deterministic, so re-solving each cycle would
    # pick different configs — the arm would swing and sometimes reach toward the
    # operator, making the SSM zone reflect arm motion, not operator approach.
    # Resolve each target ONCE (seeded from a fixed tucked "elbow-up, tool-down"
    # reference with shoulder_pan aimed at the target → a compact config over the
    # arm's own workspace) and cache it; every leg is a PTP to the cached goal.
    _UR_BASE_XY = (-0.10, 0.0)  # UR base offset on table_top (cell.xacro mount)
    _SEED_TAIL = [-1.2, 1.4, -1.6, -1.57, 0.0]  # lift, elbow, wrist_1/2/3
    _ik_cache: dict[tuple[float, float, float], list[float]] = {}

    def resolve_joints(xyz: tuple[float, float, float]) -> list[float] | None:
        if xyz in _ik_cache:
            return _ik_cache[xyz]
        # The frame set_from_ik expects for a bare Pose is version-dependent, so try
        # both world AND base_link-shifted coords (world→base_link is a pure (-0.10,
        # 0, 0.74) translation — cell.xacro) and accept whichever yields a solution
        # whose tool0 FK lands on the world target.
        candidates = (
            (xyz[0], xyz[1], xyz[2]),  # world
            (xyz[0] - _UR_BASE_XY[0], xyz[1], xyz[2] - 0.74),  # base_link
        )
        pan = math.atan2(xyz[1] - _UR_BASE_XY[1], xyz[0] - _UR_BASE_XY[0])
        pose = pose_goal(xyz).pose
        rs = RobotState(robot_model)
        for cand in candidates:
            pose.position.x, pose.position.y, pose.position.z = cand
            for k in range(20):
                seed = [pan + 0.1 * ((k % 5) - 2)] + list(_SEED_TAIL)
                if k:
                    seed[2] += 0.10 * ((k % 4) - 1)
                rs.set_joint_group_positions(_PLAN_GROUP, seed)
                if not rs.set_from_ik(_PLAN_GROUP, pose, _EEF_LINK, timeout=0.05):
                    continue
                rs.update()
                tf = rs.get_global_link_transform(_EEF_LINK)
                tool_xyz = (tf[0, 3], tf[1, 3], tf[2, 3])
                err = sum((a - b) ** 2 for a, b in zip(tool_xyz, xyz)) ** 0.5
                if err < 0.02:
                    joints = list(rs.get_joint_group_positions(_PLAN_GROUP))
                    _ik_cache[xyz] = joints
                    log.info(f"[planner_node] IK {xyz} -> tool0 err {err * 1000:.0f}mm")
                    return joints
        return None

    def plan_and_publish(leg: "KittingLeg") -> tuple[float, list[float] | None]:
        # Returns (trajectory duration, goal joints). The caller waits for the arm
        # to reach the goal before advancing, so a slowed leg completes not aborts.
        joints = resolve_joints(leg.xyz)
        if joints is None:
            log.warn(f"[planner_node] {leg.state}: IK unreachable at {leg.xyz}")
            return 0.0, None
        goal_state = RobotState(robot_model)
        goal_state.set_joint_group_positions(_PLAN_GROUP, joints)
        for _ in range(3):
            arm.set_start_state_to_current_state()
            arm.set_goal_state(robot_state=goal_state)
            params = PlanRequestParameters(moveit, "pilz_industrial_motion_planner")
            params.planning_pipeline = "pilz_industrial_motion_planner"
            params.planner_id = "PTP"
            result = arm.plan(single_plan_parameters=params)
            if result and getattr(result, "trajectory", None):
                joint_traj = (
                    result.trajectory.get_robot_trajectory_msg().joint_trajectory
                )
                state_pub.publish(String(data=leg.state))
                traj_pub.publish(joint_traj)
                last_t = (
                    joint_traj.points[-1].time_from_start if joint_traj.points else None
                )
                dur = (last_t.sec + last_t.nanosec * 1e-9) if last_t else 0.0
                log.info(
                    f"[planner_node] {leg.state} PTP -> {len(joint_traj.points)} pts, {dur:.1f}s"
                )
                return dur, joints
        log.warn(f"[planner_node] {leg.state}: PTP plan FAILED at {leg.xyz}")
        return 0.0, None

    try:
        i = 0
        while rclpy.ok():
            for leg in kitting_legs("left" if i % 2 == 0 else "right"):
                if not rclpy.ok():
                    break
                dur, goal = plan_and_publish(leg)
                # Closed-loop pacing: wait until the arm REACHES the leg goal, then
                # apply the settle dwell. A LARGE timeout is free on a clear cycle and
                # only bounds how long a BLOCKED leg waits for the operator to clear.
                if goal is not None:
                    reached = wait_until_reached(
                        goal, timeout_s=max(15.0, dur * 6.0 + 10.0)
                    )
                    if not reached:
                        log.info(
                            f"[planner_node] {leg.state}: goal not reached before timeout"
                        )
                time.sleep(leg.settle_s)
            i += 1
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Only reached after a CLEAN shutdown (a real bring-up exception propagates out
    # of main() before here, so the integration exit-code check still catches
    # crashes). os._exit skips MoveItPy's C++ static-destructor teardown — the
    # "class_loader: attempting to unload library while objects exist" path that
    # intermittently makes the process exit 1 on slower runners.
    os._exit(0)


if __name__ == "__main__":  # pragma: no cover
    main()
