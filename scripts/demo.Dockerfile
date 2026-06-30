# SafeCollab — DEMO image: the lean app image (safecollab:dev) + RViz.
#
# RViz (and its Qt/OGRE stack) is a VISUALISATION tool, not an application
# dependency: no node requires it, and CI never launches it (rviz:=false is the
# default; the headless integration/scenario jobs never set rviz:=true). The main
# Dockerfile is the single source of truth for the *application* dependency set
# that CI runs and the deliver job ships as the Release tarball — keeping rviz2
# (hundreds of MB of Qt/OGRE) out of it keeps CI builds and that artifact lean.
#
# So RViz is layered on here, only for `scripts/record-demo.sh`. The base already
# has the full app dependency set, so this image is a strict visualisation
# superset — no app-dependency divergence from what CI exercises.
#
# Built automatically by scripts/record-demo.sh (FROM the base it builds first):
#   docker build -t safecollab:demo -f scripts/demo.Dockerfile --build-arg BASE=safecollab:dev .
ARG BASE=safecollab:dev
FROM ${BASE}

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
