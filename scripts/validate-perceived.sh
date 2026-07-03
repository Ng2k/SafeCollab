#!/usr/bin/env bash
# =============================================================================
# validate-perceived.sh — live end-to-end check of the SafeCollab PERCEIVED
# safety path (camera -> perception_node -> safety_monitor).
#
# This is the validation that the headless CI jobs do NOT perform: CI proves
# the AT-1..AT-5 behaviour via safety_source:=ground_truth (which bypasses
# perception). Here we run with safety_source:=perceived and confirm that the
# camera -> classical-CV detector -> TF(world->human) -> SSM loop actually
# closes, by comparing the PERCEIVED human pose against the GROUND-TRUTH pose
# the simulator publishes (world->human_gt).
#
# See the "Validating the perceived path live" section of README.md for the
# walkthrough and the four checks. Running this script touches Docker only — it
# never touches git and never pushes anything.
#
# Usage:
#   scripts/validate-perceived.sh                 # build image (if needed) + validate
#   scripts/validate-perceived.sh --no-build      # reuse an existing safecollab:dev
#   scripts/validate-perceived.sh --window 60     # observe /safety for 60 s (default 40)
#   scripts/validate-perceived.sh --keep          # leave the container running for poking
#
# Exit code: 0 if every check passes, 1 otherwise.
# =============================================================================
set -uo pipefail

# ---- configuration ----------------------------------------------------------
IMAGE="safecollab:dev"
CONTAINER="safecollab-validate"
PATH_SEED=42                # deterministic operator path (human_node path_seed)
STARTUP_TIMEOUT=90          # max seconds to wait for the cell to come up
OBSERVE_WINDOW=40           # seconds to watch /safety/zone and /safety/scale
TF_AGREEMENT_TOL=0.15       # max accepted ||perceived - ground_truth|| in metres
DO_BUILD=1
KEEP=0

while [ $# -gt 0 ]; do
    case "$1" in
        --no-build) DO_BUILD=0 ;;
        --keep)     KEEP=1 ;;
        --window)   shift; OBSERVE_WINDOW="${1:?--window needs a value}" ;;
        --seed)     shift; PATH_SEED="${1:?--seed needs a value}" ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -40
            exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

# Resolve the repository root from this script's location so it works no matter
# where it is invoked from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PASS=0
FAIL=0
ok()   { printf '  \033[32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS + 1)); }
bad()  { printf '  \033[31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL + 1)); }
info() { printf '\033[1m==> %s\033[0m\n' "$1"; }

# Run a command inside the already-running container with ROS + the overlay
# sourced. `docker exec` does NOT go through the image ENTRYPOINT, so we invoke
# the entrypoint explicitly: `/entrypoint.sh <cmd>` sources jazzy + the colcon
# overlay and then exec's <cmd>.
dexec() { docker exec "${CONTAINER}" /entrypoint.sh bash -lc "$*"; }

cleanup() {
    if [ "${KEEP}" -eq 1 ]; then
        info "Leaving container '${CONTAINER}' running (--keep). Inspect with:"
        echo "    docker exec -it ${CONTAINER} /entrypoint.sh bash"
        echo "    docker rm -f ${CONTAINER}   # when done"
        return
    fi
    docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# ---- 0. preflight -----------------------------------------------------------
command -v docker >/dev/null 2>&1 || { echo "docker not found on PATH" >&2; exit 2; }
docker info >/dev/null 2>&1 || {
    echo "Cannot talk to the Docker daemon. Start it with: sudo systemctl start docker" >&2
    echo "(and make sure your user is in the 'docker' group, or run with sudo)." >&2
    exit 2
}

# A leftover container from a previous aborted run would make `docker run` fail.
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

# ---- 1. build the image -----------------------------------------------------
if [ "${DO_BUILD}" -eq 1 ]; then
    info "Building ${IMAGE} (this is the CI-identical environment)…"
    docker build -t "${IMAGE}" "${REPO_ROOT}" || { echo "image build failed" >&2; exit 1; }
else
    info "Reusing existing ${IMAGE} (--no-build)"
    docker image inspect "${IMAGE}" >/dev/null 2>&1 || {
        echo "${IMAGE} not found — drop --no-build to build it first." >&2; exit 1; }
fi

# ---- 2. start the cell, headless, with the PERCEIVED safety source ----------
# --device /dev/dri (when present) gives the gz camera sensor hardware GL so it
# actually renders frames; without any GL the sensor may publish nothing. If
# /dev/dri is absent we fall back to software rendering via LIBGL_ALWAYS_SOFTWARE.
DRI_ARGS=()
GL_ENV=()
if [ -e /dev/dri ]; then
    DRI_ARGS=(--device /dev/dri)
else
    GL_ENV=(-e LIBGL_ALWAYS_SOFTWARE=1)
    info "/dev/dri not present — using software GL (slower; camera may lag)"
fi

info "Starting the cell headless (safety_source:=perceived, path_seed:=${PATH_SEED})…"
docker run -d --init --name "${CONTAINER}" \
    "${DRI_ARGS[@]}" "${GL_ENV[@]}" \
    "${IMAGE}" \
    ros2 launch safecollab cell.launch.py \
        headless:=true safety_source:=perceived "path_seed:=${PATH_SEED}" \
    >/dev/null || { echo "failed to start container" >&2; exit 1; }

# ---- 3. wait for bring-up ---------------------------------------------------
# The cell is "up" once safety_monitor is publishing /safety/scale. We poll for
# a single message rather than sleeping a fixed amount, so a slow software-GL
# start still works and a fast machine isn't penalised.
info "Waiting for the cell to come up (up to ${STARTUP_TIMEOUT}s)…"
up=0
deadline=$(( $(date +%s) + STARTUP_TIMEOUT ))
while [ "$(date +%s)" -lt "${deadline}" ]; do
    if dexec "timeout 5 ros2 topic echo /safety/scale --once" >/dev/null 2>&1; then
        up=1; break
    fi
    # If the container died (e.g. launch crashed), stop waiting and show why.
    if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null)" != "true" ]; then
        echo "container exited during start-up — last log lines:" >&2
        docker logs --tail 40 "${CONTAINER}" >&2
        exit 1
    fi
    sleep 3
done
if [ "${up}" -ne 1 ]; then
    echo "cell did not publish /safety/scale within ${STARTUP_TIMEOUT}s — logs:" >&2
    docker logs --tail 40 "${CONTAINER}" >&2
    exit 1
fi
ok "cell is up (/safety/scale is live)"

echo
info "Running perceived-path checks"

# ---- CHECK 1: camera frames are flowing -------------------------------------
# The gz camera sensor -> ros_gz_image bridge -> /camera/image. If this is dead,
# perception has no input and everything downstream is meaningless.
cam_hz="$(dexec "timeout 8 ros2 topic hz /camera/image 2>/dev/null | grep -m1 'average rate' | awk '{print \$3}'" 2>/dev/null || true)"
if [ -n "${cam_hz}" ]; then
    ok "/camera/image is publishing (~${cam_hz} Hz)"
else
    bad "/camera/image produced no frames — gz camera sensor / ros_gz_image bridge not rendering (see VALIDATE.md troubleshooting)"
fi

# ---- CHECK 2: perception is broadcasting a perceived human ------------------
# perception_node broadcasts TF world->human and publishes /human/uncertainty.
if dexec "timeout 6 ros2 run tf2_ros tf2_echo world human 2>/dev/null | grep -q Translation"; then
    ok "perceived TF world->human is present"
    have_perceived=1
else
    bad "no world->human TF — perception_node is not detecting the operator (out of FOV or HSV miss)"
    have_perceived=0
fi

if dexec "timeout 6 ros2 topic echo /human/uncertainty --once" >/dev/null 2>&1; then
    ok "/human/uncertainty is publishing (perception uncertainty σ)"
else
    bad "/human/uncertainty silent — perception_node not running its estimate"
fi

# ---- CHECK 3: perceived pose agrees with ground truth -----------------------
# This is the heart of the validation: the perceived world->human must track the
# simulator's world->human_gt.
#
# The two poses MUST be sampled near-simultaneously. The operator moves up to
# ~0.6 m/s, so reading 'human' and then 'human_gt' a second apart (e.g. two
# back-to-back `tf2_echo` calls) measures how far the operator travelled in that
# gap, NOT the perception error — it reports a spurious 0.2-0.7 m "divergence"
# even when perception is spot-on. Instead we subscribe to /tf directly and, on
# each tick, compare the latest 'human' against the latest 'human_gt' only when
# both were received within 0.2 s of each other (a true instantaneous snapshot),
# tracking the minimum separation over a short window.
if [ "${have_perceived}" -eq 1 ]; then
    best="$(dexec "python3 - <<'PYEOF'
import math, time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

WINDOW = 12.0          # seconds to sample
FRESH = 0.2            # max age difference (s) to treat two TFs as simultaneous

rclpy.init()
nd = Node('vcheck3')
cur = {}
def cb(msg):
    now = time.time()
    for tr in msg.transforms:
        c = tr.child_frame_id
        if c in ('human', 'human_gt'):
            t = tr.transform.translation
            cur[c] = (t.x, t.y, t.z)
            cur[c + '_t'] = now
nd.create_subscription(TFMessage, '/tf', cb, 50)
best = 999.0
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(nd, timeout_sec=0.05)
    if 'human' in cur and 'human_gt' in cur:
        if abs(cur['human_t'] - cur['human_gt_t']) < FRESH:
            best = min(best, math.dist(cur['human'], cur['human_gt']))
print('%.4f' % best)
rclpy.shutdown()
PYEOF
" 2>/dev/null | tail -1)"
    best="${best:-999}"
    if python3 -c "import sys; sys.exit(0 if ${best} <= ${TF_AGREEMENT_TOL} else 1)" 2>/dev/null; then
        ok "perceived vs ground-truth agree (min separation ${best} m <= ${TF_AGREEMENT_TOL} m)"
    else
        bad "perceived pose diverges from ground truth (min separation ${best} m > ${TF_AGREEMENT_TOL} m) — calibration/detection issue"
    fi
else
    bad "skipped perceived-vs-truth comparison (no perceived TF)"
fi

# ---- CHECK 4: the SSM loop reacts to the perceived human --------------------
# Over OBSERVE_WINDOW seconds, observe /safety/zone and /safety/scale. As the
# operator approaches, the scale must drop below 1.0 and the zone must leave
# green. (If perception were dead, fail-safe-on-lost would pin scale at 0 and
# the zone at 'lost' — which CHECK 2 would already have flagged.)
#
# zone and scale MUST be observed in the SAME window. They are two views of one
# instant (classify() emits 'red'/0.0 and 'yellow'/<1.0 together), so capturing
# them in back-to-back windows can show a red zone in the first window yet a
# flat 1.0 scale in the second if the operator's (deterministic-in-sim-time)
# path lands a green-heavy phase there under variable wall-clock load. A single
# concurrent subscriber removes that skew and reports what actually co-occurred.
info "Observing /safety for ${OBSERVE_WINDOW}s while the operator moves…"
obs="$(dexec "python3 - <<PYEOF
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

WINDOW = ${OBSERVE_WINDOW}
rclpy.init()
nd = Node('vcheck4')
zones = set()
min_scale = [1.0e9]
nd.create_subscription(Float32, '/safety/scale',
                       lambda m: min_scale.__setitem__(0, min(min_scale[0], m.data)), 10)
qz = QoSProfile(depth=1)
qz.durability = DurabilityPolicy.TRANSIENT_LOCAL
qz.reliability = ReliabilityPolicy.RELIABLE
nd.create_subscription(String, '/safety/zone', lambda m: zones.add(m.data), qz)
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(nd, timeout_sec=0.1)
# line 1: space-separated zones; line 2: min scale seen
print(' '.join(sorted(zones)))
print('%.4f' % (min_scale[0] if min_scale[0] < 1.0e9 else 9.99))
rclpy.shutdown()
PYEOF
" 2>/dev/null)"
zones="$(printf '%s\n' "${obs}" | sed -n '1p')"
min_scale="$(printf '%s\n' "${obs}" | sed -n '2p')"

if [ -n "${zones}" ]; then
    info "zones observed: ${zones}"
    if echo "${zones}" | grep -Eq 'yellow|red'; then
        ok "/safety/zone left green (reached yellow/red as the operator approached)"
    else
        bad "/safety/zone never left green — operator never drove the perceived zone in"
    fi
    if echo "${zones}" | grep -qw lost; then
        bad "/safety/zone hit 'lost' — perception dropped out during the run (fail-safe engaged)"
    fi
else
    bad "no /safety/zone messages captured in the observation window"
fi

if [ -n "${min_scale}" ]; then
    if python3 -c "import sys; sys.exit(0 if ${min_scale} < 0.99 else 1)" 2>/dev/null; then
        ok "/safety/scale responded to the human (min ${min_scale} < 1.0)"
    else
        bad "/safety/scale stayed at ~1.0 — SSM never slowed for the perceived human"
    fi
else
    bad "no /safety/scale messages captured in the observation window"
fi

# ---- summary ----------------------------------------------------------------
echo
info "Result: ${PASS} passed, ${FAIL} failed"
if [ "${FAIL}" -eq 0 ]; then
    printf '\033[32mPERCEIVED PATH VALIDATED — camera -> perception -> SSM loop closes live.\033[0m\n'
    exit 0
else
    printf '\033[31mPERCEIVED PATH NOT VALIDATED — see failures above and README.md.\033[0m\n'
    exit 1
fi
