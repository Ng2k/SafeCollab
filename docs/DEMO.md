# SafeCollab — Demo & Recording Guide (P5)

How to run the **headline demo** and capture the GIF/video for the README: one
green → yellow → red **protective stop → resume** cycle, **plus** a transient
detection-loss **fail-safe** (`lost` → re-acquire), driven by the *perceived*
operator (not ground truth — DoD #5).

It runs in the `safecollab:dev` Docker image with X11 passthrough, the same way
as [`VALIDATE.md`](VALIDATE.md) §5, so it works on Arch / Omarchy under Hyprland
(XWayland). One script brings up everything.

---

## 1. What you will see

It is a live experiment, not a log dump — three things on screen:

```
┌──────────────────────────┐   ┌────────────────────────────────────┐
│  Gazebo (the simulation) │   │  RViz                              │
│  • robot arm kitting     │   │  • robot model + camera image      │
│  • yellow operator box   │   │  • safety-zone SPHERE at operator  │
│    moving along its path │   │  • floating GREEN/YELLOW/RED/LOST   │
│                          │   │    text label (same colour)        │
└──────────────────────────┘   └────────────────────────────────────┘
┌────────────────────────────────────────────────────────────────────┐
│  This terminal — console HUD, one colour-coded line, in place:       │
│     SSM | RED    | speed   0% | min-dist 0.38 m                      │
└────────────────────────────────────────────────────────────────────┘
```

As the operator approaches the tray, the sphere/label and HUD go
`GREEN → YELLOW → RED`, the arm slows then stops (`speed 0%`); as the operator
retreats it resumes. Freezing perception makes it fail-safe to grey `LOST`, then
re-acquire.

---

## 2. Prerequisites

- Docker, and your user in the `docker` group.
- A graphical session so `DISPLAY` is set (XWayland provides this under Hyprland).
- **Build the image from the P5 branch.** The released `safecollab:dev` predates
  the RViz zone label and the HUD; the script rebuilds by default so they are
  present. (If you pass `--no-build`, make sure your image was built from a commit
  that contains `feat/p5-polish`.)
- **RViz is layered on a demo image.** `rviz2` (Qt/OGRE) is not in the lean
  `safecollab:dev` (CI never needs it). The script builds `safecollab:demo`
  (= `safecollab:dev` + `ros-jazzy-rviz2`, see `scripts/demo.Dockerfile`)
  automatically when RViz is enabled — no manual step. `--no-rviz` skips it and
  runs the lean image (Gazebo + HUD only).

---

## 3. Quick start (the script)

```bash
# Build (if needed) and bring up Gazebo + RViz + the HUD, perceived-driven:
scripts/record-demo.sh
```

Wait for `cell is up`. The Gazebo and RViz windows open; this terminal becomes
the HUD. Start your screen recorder now (see §5).

**Cue the fail-safe** when you want it on camera — in a *second* terminal:

```bash
scripts/record-demo.sh failsafe            # freezes perception ~6 s, then resumes
scripts/record-demo.sh failsafe --hold 8   # hold the loss longer
```

You will see the zone go grey `LOST`, the HUD show `LOST / 0%`, then the zone
leave `LOST` and the arm resume.

**Stop**: `Ctrl-C` in the HUD terminal tears the whole demo down (stops the
container, releases any frozen node, revokes the X grant). Or, from elsewhere:

```bash
scripts/record-demo.sh down
```

### Useful flags

| Flag | Effect |
|------|--------|
| `--no-build` | Reuse the existing `safecollab:dev` (must already contain P5). |
| `--source ground_truth` | Drive the loop from the ground-truth TF if perception is GPU-flaky. The fail-safe cue then freezes `human_node` instead. |
| `--seed N` | A different deterministic operator path (default `42`). |
| `--no-rviz` | Gazebo + HUD only (lighter, no RViz window). |
| `--auto-failsafe` | Fire the loss→recover cue automatically ~35 s in (hands-free). |

---

## 4. A good ~30-second take

1. **Approach** — operator nears the tray: `GREEN → YELLOW → RED`, HUD `speed`
   ramps to `0%` (protective stop).
2. **Retreat** — operator withdraws: arm resumes, zone returns to `GREEN`.
3. **Fail-safe** — run `record-demo.sh failsafe`: grey `LOST`, then re-acquire
   and resume.

Recording all three (the ramp/stop, the resume, and the fail-safe) covers the
acceptance story end to end.

---

## 5. Recording → GIF

Capture the Gazebo + RViz windows and the HUD terminal together.

```bash
# Wayland (Hyprland): record a region to mp4
wf-recorder -g "$(slurp)" -f demo.mp4        # needs wf-recorder + slurp
# or use OBS Studio for a framed multi-window capture.

# Convert to a compressed, looping GIF for the README:
ffmpeg -i demo.mp4 -vf "fps=12,scale=900:-1:flags=lanczos" -loop 0 docs/media/safecollab-demo.gif
# (gifski usually gives a smaller, cleaner GIF if you have it:)
#   ffmpeg -i demo.mp4 -vf scale=900:-1 -f yuv4mpegpipe - | gifski -o docs/media/safecollab-demo.gif -
```

Drop the result at `docs/media/safecollab-demo.gif` and embed it:

```markdown
![SafeCollab SSM demo](docs/media/safecollab-demo.gif)
```

---

## 6. The 5-second legibility check

The polish goal (AGENTS §10 P5) is that a viewer can name the safety state within
**five seconds**. After recording, glance at a still frame: the zone word
(`GREEN/YELLOW/RED/LOST`) and its colour should be unmistakable in both RViz (the
floating label) and the HUD (the leading token), with the speed `%` confirming
the stop. If it is not instantly readable, raise `marker_label_height`
(`config/safety.yaml`) or keep HUD colour on (`config/hud.yaml`).

---

## 7. Doing it by hand (no script)

```bash
# Build the demo image once (lean app image + rviz2):
docker build -t safecollab:dev .
docker build -t safecollab:demo -f scripts/demo.Dockerfile --build-arg BASE=safecollab:dev .

xhost +local:docker
# Terminal 1 — the cell with GUI + RViz, perceived source:
docker run --rm --init --name safecollab-demo \
    -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host --device /dev/dri \
    safecollab:demo \
    ros2 launch safecollab cell.launch.py headless:=false rviz:=true safety_source:=perceived

# Terminal 2 — the HUD (its own pane keeps the in-place line clean):
docker exec -it safecollab-demo /entrypoint.sh ros2 run safecollab hud_node

# Terminal 3 — cue the fail-safe:
docker exec safecollab-demo pkill -STOP -f perception_node   # -> LOST
docker exec safecollab-demo pkill -CONT -f perception_node   # -> re-acquire

xhost -local:docker   # when done
```

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Gazebo/RViz window is black or GL errors | X/DRI passthrough isn't reaching the GPU. Confirm `/dev/dri` exists; try `--no-rviz` to lighten the load; software GL is slow — give it more start-up time. |
| HUD shows `LOST` the whole time | Perception isn't detecting the operator (out of FOV, or GPU render too slow). Use `--source ground_truth` to demo the SSM behaviour without perception. |
| `record-demo.sh failsafe` says "not running" | The `up` container isn't started, or it exited — check `docker logs safecollab-demo`. |
| RViz window never opens / `rviz2: command not found` | The demo image lacks RViz. Rebuild it (drop `--no-build`) so `safecollab:demo` is layered from `scripts/demo.Dockerfile`, or run `--no-rviz`. |
| The HUD line is overwritten by logs | Run the HUD in its own terminal (the script already does); avoid `hud:=true` in the launch terminal for recording. |
| `Cannot talk to the Docker daemon` | `sudo systemctl start docker`; add yourself to the `docker` group (`newgrp docker`). |

---

*See also [`VALIDATE.md`](VALIDATE.md) (headless perceived-path validation) and
[`ROADMAP-P5.md`](ROADMAP-P5.md) (the polish & demo sprint plan).*
