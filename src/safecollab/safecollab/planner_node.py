"""Planner node — MoveIt/Pilz pick-and-place trajectory source (Stream D').

Replaces the hand-waypoint ``task_node``: instead of hand-solved joint poses, the
kitting cycle is expressed as Cartesian tool targets and MoveIt's **Pilz**
industrial planner (deterministic PTP/LIN) generates a smooth, IK-solved
trajectory for the real UR5e. Each planned segment is published as a dense
position ``JointTrajectory`` on ``/motion/nominal_trajectory`` — the exact
message ``motion_node`` already consumes — so the ISO/TS 15066 SSM speed-scaling
loop (``safety_monitor`` → ``/safety/scale`` → ``motion_node`` retime) governs the
motion unchanged. The planner only PLANS; ``motion_node`` executes with SSM.

Architecture (AGENTS.md ground rule 3):
  ``KittingPlan``  — pure-Python leg sequence + Cartesian targets. No ROS/MoveIt,
                     so it is unit-testable without a live graph.
  ``PlannerNode``  — thin MoveItPy wrapper: plans each leg, publishes the
                     trajectory and the leg name on ``/task/state``.

Interface contract (AGENTS.md §3):
  Publishes:  /motion/nominal_trajectory (trajectory_msgs/JointTrajectory)
              /task/state                (std_msgs/String)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

# ---------------------------------------------------------------------------
# Pure-Python kitting plan (unit-testable, no ROS / MoveIt)
# ---------------------------------------------------------------------------

MotionType = Literal["ptp", "lin"]


@dataclass(frozen=True)
class KittingLeg:
    """One planned segment of the kitting cycle.

    Attributes:
        state: leg name published on /task/state (demo/HUD narrative).
        motion: "ptp" (point-to-point joint transit) or "lin" (straight Cartesian).
        xyz: world-frame tool0 target position (metres).
        settle_s: seconds to dwell after the segment before planning the next
                  (lets the arm reach the waypoint; retimed motion may take longer).
    """

    state: str
    motion: MotionType
    xyz: tuple[float, float, float]
    settle_s: float


# World-frame tool0 targets. The tool points straight down at each target; PTP
# transits move at the higher "above" height, LIN dips/lifts straight into and out
# of the bin/tray. Targets sit in the UR5e's comfortable reach envelope and match
# the cell furniture placed in cell.xacro. bin side alternates each cycle.
# Heights (world z). table_top is 0.74; bins are 0.08 m boxes (top 0.82) and the
# tray top is 0.78. _PICK_Z=0.78 makes the tool DIP INTO the bin (4 cm below the
# rim) to grasp, and place ON the tray surface — instead of stopping at the rim /
# hovering above, which read as "the arm just goes on top and never picks". The
# raised _ABOVE_Z gives a clearly visible ~0.20 m vertical pick/place stroke.
_ABOVE_Z = 0.98
_PICK_Z = 0.78


def kitting_legs(bin_side: str) -> list[KittingLeg]:
    """Return the ordered legs for one kitting cycle from the given feeder bin.

    Cycle: transit above the bin (PTP) → dip in (LIN) → lift out (LIN) →
    transit above the tray (PTP) → dip in (LIN) → lift out (LIN).

    Args:
        bin_side: "left" or "right" — which feeder bin to pick from.

    Raises:
        ValueError: if bin_side is not "left" or "right".
    """
    if bin_side not in ("left", "right"):
        raise ValueError(f"bin_side must be 'left' or 'right', got {bin_side!r}")
    # Feeder-bin tool targets mirror cell.xacro: bins flank the base FORWARD of
    # centre at world (0.15, ±0.35) — base-rel (0.25, ±0.35), a well-conditioned
    # ~0.43 m front-side reach clear of the shoulder singularity (pure-side bins
    # at base-rel x≈0 made the +y reach near-singular and unreachable).
    by = 0.35 if bin_side == "left" else -0.35
    bx = 0.15
    tx, ty = 0.35, 0.0
    # settle_s is the dwell AFTER the leg before the next is planned. The DROP
    # dwell keeps the arm at the shared tray so an operator reach there co-occurs
    # with the arm and drives a red protective stop (AT-6 needs the escalation to
    # recur across ≥3 distinct random paths). It no longer has to be LONG: closed-
    # loop pacing makes the arm WAIT at the tray whenever the operator is close, so
    # co-occupancy is reliable even with a shorter dwell — which keeps the demo
    # snappier (the arm isn't parked idle at the tray when the operator is clear).
    return [
        KittingLeg(f"GO_TO_BIN_{bin_side.upper()}", "ptp", (bx, by, _ABOVE_Z), 0.4),
        KittingLeg("PICK", "lin", (bx, by, _PICK_Z), 0.6),
        KittingLeg("LIFT_BIN", "lin", (bx, by, _ABOVE_Z), 0.3),
        KittingLeg("GO_TO_TRAY", "ptp", (tx, ty, _ABOVE_Z), 0.4),
        KittingLeg("DROP", "lin", (tx, ty, _PICK_Z), 2.0),
        KittingLeg("LIFT_TRAY", "lin", (tx, ty, _ABOVE_Z), 0.4),
    ]


def cycle_sequence(n_cycles: int) -> list[KittingLeg]:
    """Flatten ``n_cycles`` kitting cycles, alternating bins left/right."""
    legs: list[KittingLeg] = []
    for i in range(n_cycles):
        legs.extend(kitting_legs("left" if i % 2 == 0 else "right"))
    return legs


# ---------------------------------------------------------------------------
# ROS 2 + MoveItPy node (requires a live graph — excluded from unit coverage)
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    import math
    import time

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
    # Tool0 z-axis (approach axis) pointing straight DOWN into the bin/tray:
    # 180° about world x → quaternion (w, x, y, z) = (0, 1, 0, 0).
    _DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)

    # MoveItPy reads the MoveIt config (robot_description[_semantic], kinematics,
    # planning pipelines) from THIS node's ROS parameters, which cell.launch.py
    # supplies via MoveItConfigsBuilder(...).to_dict(). Passing them as node params
    # (not MoveItPy(config_dict=...)) is what actually declares the nested pipeline
    # params so MoveItCpp can load the Pilz planner.
    rclpy.init(args=args)
    moveit = MoveItPy(node_name="planner_node")
    arm = moveit.get_planning_component(_PLAN_GROUP)
    robot_model = moveit.get_robot_model()

    # A separate lightweight node carries the publishers (MoveItPy owns its node).
    pub = Node("planner_pub")
    traj_pub = pub.create_publisher(JointTrajectory, "/motion/nominal_trajectory", 10)
    state_pub = pub.create_publisher(String, "/task/state", 10)
    log = pub.get_logger()

    # Closed-loop pacing needs the live joint state. /joint_states is published in
    # the broadcaster's (alphabetical) order; we index it BY NAME into the planning
    # group order so the goal comparison is order-correct (same alignment pitfall
    # motion_node handles). The planner waits until the arm actually reaches each
    # leg goal before advancing, so an SSM slow-down/stop near the operator merely
    # DELAYS the pick — it never abandons it half-finished (the "arm stops short of
    # the bin" symptom of the old open-loop wall-clock pacing).
    _JOINT_ORDER = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
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
        # Returns True once every joint is within _REACH_TOL of the goal, or False
        # if timeout_s elapses first (e.g. a long protective stop while the operator
        # lingers). Spins the pub node so the joint-state callback actually fires.
        _REACH_TOL = 0.05  # rad (~3° per joint)
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(pub, timeout_sec=0.05)
            cur = current_joints()
            if cur is not None and max(abs(a - b) for a, b in zip(cur, goal)) < _REACH_TOL:
                return True
        return False

    log.info("[planner_node] MoveItPy up; planning group ur_manipulator")

    def pose_goal(xyz: tuple[float, float, float]) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = _PLAN_FRAME
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        (p.pose.orientation.w, p.pose.orientation.x,
         p.pose.orientation.y, p.pose.orientation.z) = _DOWN_QUAT
        return p

    # Deterministic IK: KDL is a *non-deterministic* numerical solver, so replanning
    # each cycle would pick different joint configs — the arm would swing wildly and
    # sometimes reach toward the operator, making the SSM zone reflect arm motion
    # instead of operator approach. Instead we resolve each Cartesian target to a
    # joint config ONCE, seeded from a fixed tucked "elbow-up, tool-down" reference
    # so the solution is consistent and stays in the arm's own workspace, then cache
    # it. Every leg is a PTP to the cached JOINT goal — deterministic and reliable.
    # Tucked "elbow-up, tool-down" reference; shoulder_pan is aimed at each target
    # so KDL converges to a COMPACT solution (arm over its own workspace) instead of
    # a contorted reach-around that swings a link toward the operator.
    _UR_BASE_XY = (-0.10, 0.0)  # UR base offset on table_top (cell.xacro mount)
    _SEED_TAIL = [-1.2, 1.4, -1.6, -1.57, 0.0]  # lift, elbow, wrist_1/2/3
    _ik_cache: dict[tuple[float, float, float], list[float]] = {}

    def resolve_joints(xyz: tuple[float, float, float]) -> list[float] | None:
        if xyz in _ik_cache:
            return _ik_cache[xyz]
        # The frame set_from_ik expects for a bare Pose is version-dependent, so try
        # both the world coords AND the base_link-shifted coords (world→base_link is
        # pure translation (-0.10, 0, 0.74), no rotation — see cell.xacro), and
        # accept whichever yields a solution whose tool0 FK (in the world/model root)
        # actually lands on the world target. Seed shoulder_pan at the target so KDL
        # converges to a compact, operator-avoiding config.
        candidates = (
            (xyz[0], xyz[1], xyz[2]),                              # world
            (xyz[0] - _UR_BASE_XY[0], xyz[1], xyz[2] - 0.74),      # base_link
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
        # Returns (planned trajectory duration, goal joint config). The caller waits
        # until the arm actually reaches the goal config before advancing, so a
        # slowed/stopped leg is completed rather than abandoned.
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
                joint_traj = result.trajectory.get_robot_trajectory_msg().joint_trajectory
                state_pub.publish(String(data=leg.state))
                traj_pub.publish(joint_traj)
                last_t = joint_traj.points[-1].time_from_start if joint_traj.points else None
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
                # Closed-loop pacing: wait until the arm actually REACHES the leg
                # goal before advancing, then apply the settle dwell. wait_until_reached
                # returns the instant the arm arrives, so a LARGE timeout costs nothing
                # on a clear (green) cycle — it only bounds how long a BLOCKED leg waits
                # for the operator to step clear. The bin on the operator's approach
                # side used to hit the old ~8 s timeout mid-descent and get skipped
                # (the arm "picked" only one bin); a patient ~15 s budget lets that
                # PICK/DROP wait the operator out and actually complete the dip, while
                # green cycles stay just as snappy.
                if goal is not None:
                    reached = wait_until_reached(goal, timeout_s=max(15.0, dur * 6.0 + 10.0))
                    if not reached:
                        log.info(f"[planner_node] {leg.state}: goal not reached before timeout")
                time.sleep(leg.settle_s)
            i += 1
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
