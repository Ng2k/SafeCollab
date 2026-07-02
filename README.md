# SafeCollab

Project for the course of Smart Robotics of the Master's degree in Artificial Intelligence Engineering.

A ROS 2 (Jazzy) + Gazebo (Harmonic) demonstration of an **ISO/TS 15066** Speed &
Separation Monitoring (SSM) safety loop: a **UR5e** runs a MoveIt/Pilz-planned
pick-and-place while a `safety_monitor` scales its speed (green → yellow → red →
protective stop) from the perceived distance to a human operator, and fails safe
on detection loss.

## Demo

![SafeCollab SSM demo — UR5e kitting under ISO/TS 15066 speed scaling, with a protective stop and a fail-safe re-acquire](docs/media/safecollab-demo.gif)

See [`docs/DEMO.md`](docs/DEMO.md) for how to run and record it.
