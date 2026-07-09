#!/usr/bin/env bash
# ISO/TS 15066 SSM demo: the UR5e kits while the operator approaches, and the
# safety loop scales its speed green -> yellow -> red (protective stop), then
# resumes as the operator retreats.
#
#   scripts/demo-ssm.sh              # build + run
#   scripts/demo-ssm.sh --no-build   # reuse the existing safecollab:dev image
set -uo pipefail

DO_BUILD=1
for a in "$@"; do [ "$a" = "--no-build" ] && DO_BUILD=0; done
CONTAINER="safecollab-ssm"

source "$(dirname "${BASH_SOURCE[0]}")/demo-common.sh"
trap 'dc_teardown "$CONTAINER"' EXIT INT TERM

dc_build_images
# path_seed:=42 is the deterministic operator path that reliably drives a full
# green -> yellow -> red -> resume cycle.
dc_start "$CONTAINER" human:=true safety:=true path_seed:=42
info "waiting for the cell to come up (up to ${STARTUP_TIMEOUT}s) ..."
dc_wait_safety "$CONTAINER"
dc_wait_arm "$CONTAINER"
ok "cell up — watch the zone go green -> yellow -> red as the operator approaches. Ctrl-C to stop."

# Foreground: the console HUD (zone / speed scale / min-distance).
dc_exec "$CONTAINER" ros2 run safecollab hud_node || true
