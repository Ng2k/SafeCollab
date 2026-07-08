#!/usr/bin/env bash
# =============================================================================
# record-demo.sh — one-command SafeCollab demo for screen-recording (P5).
#
# Brings up the FULL cell with the GUI (Gazebo + RViz zone marker/label) and the
# console HUD, driven by the PERCEIVED human (the headline path, DoD #5), so you
# can record one green->yellow->red protective-stop->resume cycle AND a
# transient detection-loss fail-safe (lost -> re-acquire).
#
# Everything runs in the `safecollab:dev` Docker image with X11 passthrough
# (same X11 approach as README.md's validation section), so it works under
# Hyprland (XWayland). It never touches git and never pushes anything.
#
# LAYOUT once it is up:
#   * Gazebo window  — the simulated cell (arm kitting, yellow operator moving)
#   * RViz window    — robot model, camera image, the safety-zone sphere + the
#                      floating GREEN/YELLOW/RED/LOST text label
#   * THIS terminal  — the console HUD, one colour-coded line refreshed in place:
#                        SSM | RED    | speed   0% | min-dist 0.38 m
#
# By default gz runs HEADLESS (offscreen) and RViz is the visible 3D window —
# reliable under Wayland. Add --gz-gui for the raw Gazebo window only if your GL
# passthrough is known good (it can hang the sim otherwise; RViz shows everything
# anyway).
#
# USAGE:
#   scripts/record-demo.sh                 # build (if needed) + bring it all up
#   scripts/record-demo.sh --no-build      # reuse an existing safecollab:dev image
#   scripts/record-demo.sh --source ground_truth   # if perception is GPU-flaky
#   scripts/record-demo.sh --seed 7        # a different deterministic operator path
#   scripts/record-demo.sh --gz-gui        # also open the raw Gazebo window
#   scripts/record-demo.sh --auto-failsafe # auto-trigger the loss->recover cue
#
#   # In a SECOND terminal, cue the fail-safe on demand while recording:
#   scripts/record-demo.sh failsafe                # default hold
#   scripts/record-demo.sh failsafe --hold 8       # hold the loss 8 s
#
#   scripts/record-demo.sh down            # tear everything down manually
#
# Ctrl-C in this terminal stops the HUD and tears the whole demo down cleanly.
# Requires: docker, an X server (DISPLAY set — XWayland provides this). The
# image must be built from THIS branch so it contains the P5 RViz + HUD work.
# =============================================================================
set -uo pipefail

# ---- configuration ----------------------------------------------------------
IMAGE="safecollab:dev"        # lean app image (built from the repo Dockerfile)
DEMO_IMAGE="safecollab:demo"  # IMAGE + RViz, for the GUI demo (scripts/demo.Dockerfile)
CONTAINER="safecollab-demo"
SOURCE="perceived"            # perceived (headline) | ground_truth (fallback)
SEED=42                       # deterministic operator path (human_node path_seed)
DO_BUILD=1
RVIZ="true"
# gz runs headless (server-only, offscreen rendering) by default — reliable, and
# RViz already shows the robot, camera feed, and zone marker/label. The gz GUI
# (headless:=false) launches server+GUI as one process and, if the GUI cannot
# initialise (common under Wayland/GL), it takes the server down too and nothing
# spawns. Opt into the raw Gazebo window with --gz-gui only if your GL passthrough
# is known good.
HEADLESS="true"
STARTUP_TIMEOUT=150           # max seconds to wait for the cell to publish /safety
FAILSAFE_HOLD=6               # seconds to hold the detection loss in `failsafe`
AUTO_FAILSAFE=0
AUTO_FAILSAFE_DELAY=35        # seconds after HUD start before the auto cue fires

# Repo root = parent of this script's directory (so it runs from anywhere).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---- pretty logging ---------------------------------------------------------
if [ -t 1 ]; then
    C_B="\033[1m"; C_G="\033[32m"; C_Y="\033[33m"; C_R="\033[31m"; C_0="\033[0m"
else
    C_B=""; C_G=""; C_Y=""; C_R=""; C_0=""
fi
info() { printf "${C_B}[demo]${C_0} %s\n" "$*"; }
ok()   { printf "${C_G}[demo] %s${C_0}\n" "$*"; }
warn() { printf "${C_Y}[demo] %s${C_0}\n" "$*" >&2; }
die()  { printf "${C_R}[demo] ERROR:${C_0} %s\n" "$*" >&2; exit 1; }

# ---- subcommand + flag parsing ----------------------------------------------
CMD="up"
case "${1:-}" in
    up|failsafe|hud|down) CMD="$1"; shift ;;
esac

while [ $# -gt 0 ]; do
    case "$1" in
        --no-build)      DO_BUILD=0 ;;
        --source)        SOURCE="${2:?--source needs perceived|ground_truth}"; shift ;;
        --seed)          SEED="${2:?--seed needs an integer}"; shift ;;
        --no-rviz)       RVIZ="false" ;;
        --gz-gui)        HEADLESS="false" ;;
        --auto-failsafe) AUTO_FAILSAFE=1 ;;
        --hold)          FAILSAFE_HOLD="${2:?--hold needs seconds}"; shift ;;
        --image)         IMAGE="${2:?--image needs a tag}"; shift ;;
        --container)     CONTAINER="${2:?--container needs a name}"; shift ;;
        -h|--help)       sed -n '2,58p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *)               die "unknown argument: $1 (try --help)" ;;
    esac
    shift
done

command -v docker >/dev/null 2>&1 || die "docker not found on PATH."

# =============================================================================
# Helpers
# =============================================================================

container_running() {
    [ -n "$(docker ps -q -f "name=^${CONTAINER}$" 2>/dev/null)" ]
}

# Trigger / release the transient detection loss inside the running container.
# Freezing perception_node stops the PERCEIVED world->human TF, so safety_monitor
# fail-safes to 'lost' after loss_timeout; SIGCONT lets it re-acquire and resume.
# In ground_truth mode the TF comes from human_node, so freeze that instead.
failsafe_target() {
    [ "$SOURCE" = "ground_truth" ] && echo "human_node" || echo "perception_node"
}

do_failsafe() {
    local target; target="$(failsafe_target)"
    container_running || die "demo container '${CONTAINER}' is not running (start it first)."
    warn ">>> detection loss: freezing ${target} for ${FAILSAFE_HOLD}s (zone -> LOST) ..."
    docker exec "${CONTAINER}" pkill -STOP -f "${target}" 2>/dev/null || true
    sleep "${FAILSAFE_HOLD}"
    docker exec "${CONTAINER}" pkill -CONT -f "${target}" 2>/dev/null || true
    ok "<<< ${target} resumed — watch the zone leave LOST and the arm recover."
}

teardown() {
    # Always release a frozen node, stop the container, and revoke the X grant.
    if container_running; then
        docker exec "${CONTAINER}" pkill -CONT -f perception_node 2>/dev/null || true
        docker exec "${CONTAINER}" pkill -CONT -f human_node 2>/dev/null || true
        info "stopping container '${CONTAINER}' ..."
        docker stop -t 5 "${CONTAINER}" >/dev/null 2>&1 || true
    fi
    docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
    xhost -local:docker >/dev/null 2>&1 || true
}

# =============================================================================
# Subcommand: failsafe / down (act on an already-running demo)
# =============================================================================
if [ "$CMD" = "failsafe" ]; then
    do_failsafe
    exit 0
fi
if [ "$CMD" = "down" ]; then
    teardown
    ok "demo torn down."
    exit 0
fi

# =============================================================================
# Subcommand: up (default) / hud
# =============================================================================
[ -n "${DISPLAY:-}" ] || die "DISPLAY is not set. The GUI needs an X server \
(under Hyprland, XWayland sets DISPLAY automatically — start a graphical session)."

# Tear down any leftover from a previous run, and on exit/Ctrl-C.
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
trap teardown EXIT INT TERM

if [ "$CMD" = "up" ]; then
    # ---- 1. image(s) --------------------------------------------------------
    # RUN_IMAGE is the lean app image, or the +RViz demo layer when rviz:=true.
    RUN_IMAGE="${IMAGE}"
    if [ "$DO_BUILD" = "1" ]; then
        info "building ${IMAGE} from $(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?') \
(this includes the P5 RViz + HUD work) ..."
        docker build -t "${IMAGE}" "${REPO_ROOT}" || die "docker build failed."
    else
        docker image inspect "${IMAGE}" >/dev/null 2>&1 \
            || die "image ${IMAGE} not found; drop --no-build to build it."
        warn "reusing existing ${IMAGE} — if the HUD/zone-label are missing, rebuild (drop --no-build)."
    fi
    if [ "$RVIZ" = "true" ]; then
        # rviz2 is not in the lean image (CI never needs it); layer it on here.
        info "building ${DEMO_IMAGE} (= ${IMAGE} + rviz2) for the GUI demo ..."
        docker build -t "${DEMO_IMAGE}" \
            -f "${SCRIPT_DIR}/demo.Dockerfile" \
            --build-arg BASE="${IMAGE}" \
            "${REPO_ROOT}" || die "demo image build failed (rviz2 layer)."
        RUN_IMAGE="${DEMO_IMAGE}"
    fi

    # ---- 2. GPU + X passthrough --------------------------------------------
    DRI_ARGS=()
    if [ -e /dev/dri ]; then
        DRI_ARGS=(--device /dev/dri)
    else
        warn "/dev/dri not present — falling back to software GL (slow camera render)."
        DRI_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
    fi
    info "granting local X access to docker (revoked on exit) ..."
    xhost +local:docker >/dev/null 2>&1 || warn "xhost grant failed; the GUI may not appear."
    [ "$HEADLESS" = "false" ] && warn "--gz-gui: launching the raw Gazebo window; if it \
hangs at 'Requesting list of world names', your GL passthrough can't start the gz GUI — \
drop --gz-gui (RViz still shows everything)."

    # ---- 3. launch the cell (detached: RViz [+ gz GUI] windows appear) -------
    # No --rm: keep the container so 'docker logs' survives a start-up crash;
    # teardown removes it (docker rm -f) on exit.
    info "starting the cell: source=${SOURCE} seed=${SEED} rviz=${RVIZ} gz_gui=$([ "$HEADLESS" = false ] && echo on || echo off) ..."
    # NB: NO --network host. gz-transport discovers the sim server<->spawner over
    # UDP multicast; on the host's real interfaces that discovery fails, so the gz
    # server never serves the world and the spawners loop forever at "Requesting
    # list of world names" (no controllers -> no /joint_states -> the robot's
    # moving frames link_1..6/tcp never connect to world, and the scene is static).
    # An isolated container netns (the default bridge, like CI) makes discovery
    # work on loopback. X11 to RViz goes over the mounted /tmp/.X11-unix socket,
    # which needs no host networking; the HUD and fail-safe cue use `docker exec`.
    docker run -d --init --name "${CONTAINER}" \
        -e DISPLAY="${DISPLAY}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        "${DRI_ARGS[@]}" \
        "${RUN_IMAGE}" \
        ros2 launch safecollab cell.launch.py \
            headless:="${HEADLESS}" rviz:="${RVIZ}" \
            safety_source:="${SOURCE}" path_seed:="${SEED}" \
        >/dev/null || die "docker run failed."

    # ---- 4. wait until the cell is genuinely up -----------------------------
    # Two gates, both bounded by the same deadline:
    #   a) /safety/zone publishes            -> the safety loop is closed.
    #   b) world->tool0 TF resolves          -> the ARM is up and renderable.
    # Gate (b) matters because /safety/zone can publish (even 'lost') before the
    # controllers activate and the arm's transforms exist — at which point RViz
    # shows the zone sphere but NO robot, which reads as "the arm is missing".
    # Waiting for the arm TF makes "READY TO RECORD" mean the arm is on screen.
    info "waiting for the cell to come up (up to ${STARTUP_TIMEOUT}s; software GL is slow) ..."
    deadline=$(( $(date +%s) + STARTUP_TIMEOUT ))

    # cell_ready CMD... — run a readiness probe inside the container; true if it
    # exits 0. Dies with the container's last logs if it crashed during start-up.
    wait_for() {
        local what="$1"; shift
        until docker exec "${CONTAINER}" /entrypoint.sh "$@" >/dev/null 2>&1; do
            if ! container_running; then
                warn "the cell container exited during start-up — last 40 log lines:"
                docker logs --tail 40 "${CONTAINER}" 2>&1 | sed 's/^/    /' || true
                die "cell start-up failed (see logs above)."
            fi
            [ "$(date +%s)" -lt "$deadline" ] || die "timed out waiting for ${what}."
            sleep 2
        done
    }

    wait_for "the safety loop (/safety/zone)" ros2 topic echo /safety/zone --once
    info "safety loop up; waiting for the arm (world->tool0 TF) ..."
    wait_for "the arm (world->tool0 TF)" \
        bash -c 'timeout 5 ros2 run tf2_ros tf2_echo world tool0 2>/dev/null | grep -q Translation'
    ok "cell is up — arm TF resolved; Gazebo and RViz windows should be open."
fi

# ---- 5. recording cues -------------------------------------------------------
cat <<EOF

  ${C_B}READY TO RECORD${C_0}
  -----------------------------------------------------------------------------
  Windows:  RViz (robot + camera feed + zone sphere & label)  +  this terminal
  (the HUD).  [Gazebo's own window only with --gz-gui.]
  Record the RViz window and this terminal. A good ~30 s take captures, in order:
    1. the operator approaching:     zone GREEN -> YELLOW -> RED, HUD speed -> 0%
    2. the operator retreating:      arm resumes, zone back to GREEN
    3. a transient detection loss:   run the cue below in ANOTHER terminal:

        ${C_B}scripts/record-demo.sh failsafe${C_0}

       -> zone goes grey LOST, HUD shows LOST / 0%, then re-acquires and resumes.

  Ctrl-C here stops the HUD and tears the whole demo down.
  -----------------------------------------------------------------------------

EOF

# Optional: fire the fail-safe cue automatically after a delay (for hands-free
# recording). Runs in the background while the HUD holds the foreground.
if [ "$AUTO_FAILSAFE" = "1" ]; then
    info "auto fail-safe armed: will fire in ${AUTO_FAILSAFE_DELAY}s, holding ${FAILSAFE_HOLD}s."
    ( sleep "${AUTO_FAILSAFE_DELAY}"; do_failsafe ) &
fi

# ---- 6. HUD in the foreground of THIS terminal ------------------------------
info "attaching the console HUD (Ctrl-C to end the demo) ..."
docker exec "${CONTAINER}" /entrypoint.sh ros2 run safecollab hud_node || true

# When the HUD exits (Ctrl-C), the EXIT trap tears everything down.
