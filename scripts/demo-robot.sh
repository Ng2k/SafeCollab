#!/usr/bin/env bash
# Robot-only demo: the UR5e runs its full MoveIt/Pilz pick-and-place at full
# speed — no operator, no SSM scaling. Shows the nominal kitting cycle end to end.
#
#   scripts/demo-robot.sh              # build + run
#   scripts/demo-robot.sh --no-build   # reuse the existing safecollab:dev image
set -uo pipefail

DO_BUILD=1
for a in "$@"; do [ "$a" = "--no-build" ] && DO_BUILD=0; done
CONTAINER="safecollab-robot"

source "$(dirname "${BASH_SOURCE[0]}")/demo-common.sh"
trap 'dc_teardown "$CONTAINER"' EXIT INT TERM

dc_build_images
# human:=false safety:=false -> no operator and no safety loop, so motion_node
# runs every planned leg at scale 1.0.
dc_start "$CONTAINER" human:=false safety:=false
info "waiting for the cell to come up (up to ${STARTUP_TIMEOUT}s) ..."
dc_wait_arm "$CONTAINER"
# Wait for the planner to publish the first leg before echoing /task/state: the
# arm TF is up (controllers active) well before planner_node finishes loading
# MoveItPy and planning, and `ros2 topic echo` on a not-yet-advertised topic
# exits immediately — which would tear the whole demo down before it starts.
info "arm up; waiting for the planner to start the kitting cycle ..."
dc_wait "$CONTAINER" "the planner (/task/state)" ros2 topic echo /task/state --once
ok "robot up — RViz shows the UR5e running its kitting cycle. Ctrl-C to stop."

# Foreground: stream the leg names so the terminal tracks the pick-and-place.
dc_exec "$CONTAINER" ros2 topic echo /task/state || true
