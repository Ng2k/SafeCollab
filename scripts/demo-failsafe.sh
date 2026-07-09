#!/usr/bin/env bash
# Fail-safe demo: the SSM cell plus detection losses triggered at random times.
# Perception is frozen for a few seconds so the zone drops to LOST (fail-safe
# protective stop), then resumes — showing the safety loop degrade safely and
# recover.
#
#   scripts/demo-failsafe.sh              # build + run
#   scripts/demo-failsafe.sh --no-build   # reuse the existing safecollab:dev image
set -uo pipefail

DO_BUILD=1
for a in "$@"; do [ "$a" = "--no-build" ] && DO_BUILD=0; done
CONTAINER="safecollab-failsafe"

source "$(dirname "${BASH_SOURCE[0]}")/demo-common.sh"

FAILSAFE_PID=""
cleanup() {
    [ -n "$FAILSAFE_PID" ] && kill "$FAILSAFE_PID" 2>/dev/null || true
    dc_teardown "$CONTAINER"
}
trap cleanup EXIT INT TERM

dc_build_images
dc_start "$CONTAINER" human:=true safety:=true path_seed:=42
info "waiting for the cell to come up (up to ${STARTUP_TIMEOUT}s) ..."
dc_wait_safety "$CONTAINER"
dc_wait_arm "$CONTAINER"
ok "cell up — random detection losses will trip the LOST fail-safe. Ctrl-C to stop."

# Background: every 15-30 s, freeze perception for 5-8 s (zone -> LOST) then thaw.
(
    while dc_running "$CONTAINER"; do
        sleep $(( RANDOM % 16 + 15 ))
        dc_running "$CONTAINER" || break
        hold=$(( RANDOM % 4 + 5 ))
        warn ">>> detection loss: freezing perception ${hold}s (zone -> LOST) ..."
        docker exec "$CONTAINER" pkill -STOP -f perception_node 2>/dev/null || true
        sleep "$hold"
        docker exec "$CONTAINER" pkill -CONT -f perception_node 2>/dev/null || true
        ok "<<< perception resumed — zone leaves LOST, arm recovers."
    done
) &
FAILSAFE_PID=$!

# Foreground: the console HUD (zone / speed scale / min-distance).
dc_exec "$CONTAINER" ros2 run safecollab hud_node || true
