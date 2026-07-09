#!/usr/bin/env bash
# Shared helpers for the SafeCollab demo launchers (demo-robot/ssm/failsafe.sh).
# Source this, then: dc_build_images; dc_start <name> <launch args...>; dc_wait_*.
# DO_BUILD=0 skips the app-image build and reuses the existing one.

IMAGE="${IMAGE:-safecollab:dev}"          # lean app image (repo Dockerfile)
DEMO_IMAGE="${DEMO_IMAGE:-safecollab:demo}"  # IMAGE + rviz2 (scripts/demo.Dockerfile)
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-150}" # seconds to wait for the cell to come up
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -t 1 ]; then
    C_B="\033[1m"; C_G="\033[32m"; C_Y="\033[33m"; C_R="\033[31m"; C_0="\033[0m"
else
    C_B=""; C_G=""; C_Y=""; C_R=""; C_0=""
fi
info() { printf "${C_B}[demo]${C_0} %s\n" "$*"; }
ok()   { printf "${C_G}[demo] %s${C_0}\n" "$*"; }
warn() { printf "${C_Y}[demo] %s${C_0}\n" "$*" >&2; }
die()  { printf "${C_R}[demo] ERROR:${C_0} %s\n" "$*" >&2; exit 1; }

dc_running() { [ -n "$(docker ps -q -f "name=^$1$" 2>/dev/null)" ]; }

dc_teardown() {
    local name="$1"
    if dc_running "$name"; then
        docker exec "$name" pkill -CONT -f perception_node 2>/dev/null || true
        docker stop -t 5 "$name" >/dev/null 2>&1 || true
    fi
    docker rm -f "$name" >/dev/null 2>&1 || true
    xhost -local:docker >/dev/null 2>&1 || true
}

dc_build_images() {
    command -v docker >/dev/null 2>&1 || die "docker not found on PATH."
    if [ "${DO_BUILD:-1}" = "1" ]; then
        info "building ${IMAGE} from $(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?') ..."
        docker build -t "${IMAGE}" "${REPO_ROOT}" || die "docker build failed."
    else
        docker image inspect "${IMAGE}" >/dev/null 2>&1 \
            || die "image ${IMAGE} not found; drop --no-build to build it."
    fi
    info "building ${DEMO_IMAGE} (= ${IMAGE} + rviz2) ..."
    docker build -t "${DEMO_IMAGE}" -f "${REPO_ROOT}/scripts/demo.Dockerfile" \
        --build-arg BASE="${IMAGE}" "${REPO_ROOT}" || die "rviz demo image build failed."
}

# dc_start <name> <extra launch args...> — run the cell detached (gz headless +
# RViz), with X11 + GPU passthrough, and arm the startup deadline.
dc_start() {
    local name="$1"; shift
    [ -n "${DISPLAY:-}" ] || die "DISPLAY is not set (the GUI needs an X server)."
    local dri=()
    if [ -e /dev/dri ]; then
        dri=(--device /dev/dri)
    else
        warn "/dev/dri absent — falling back to software GL (slow)."
        dri=(-e LIBGL_ALWAYS_SOFTWARE=1)
    fi
    docker rm -f "$name" >/dev/null 2>&1 || true
    xhost +local:docker >/dev/null 2>&1 || warn "xhost grant failed; the GUI may not appear."
    info "starting cell '${name}': $* ..."
    docker run -d --init --name "$name" \
        -e DISPLAY="${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix "${dri[@]}" \
        "${DEMO_IMAGE}" \
        ros2 launch safecollab cell.launch.py headless:=true rviz:=true "$@" \
        >/dev/null || die "docker run failed."
    DC_DEADLINE=$(( $(date +%s) + STARTUP_TIMEOUT ))
}

dc_exec() { docker exec "$1" /entrypoint.sh "${@:2}"; }  # dc_exec <name> <cmd...>

# dc_wait <name> <label> <probe cmd...> — poll the probe until it exits 0, or die
# on timeout / container crash.
dc_wait() {
    local name="$1" label="$2"; shift 2
    until dc_exec "$name" "$@" >/dev/null 2>&1; do
        if ! dc_running "$name"; then
            warn "container exited during start-up — last 40 log lines:"
            docker logs --tail 40 "$name" 2>&1 | sed 's/^/    /' || true
            die "cell start-up failed (see logs above)."
        fi
        [ "$(date +%s)" -lt "${DC_DEADLINE}" ] || die "timed out waiting for ${label}."
        sleep 2
    done
}

dc_wait_arm() {
    dc_wait "$1" "the arm (world->tool0 TF)" \
        bash -c 'timeout 5 ros2 run tf2_ros tf2_echo world tool0 2>/dev/null | grep -q Translation'
}
dc_wait_safety() {
    dc_wait "$1" "the safety loop (/safety/zone)" ros2 topic echo /safety/zone --once
}
