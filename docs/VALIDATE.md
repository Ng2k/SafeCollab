# Validating the perceived safety path live

This guide walks through validating the **perceived** Speed-and-Separation
Monitoring (SSM) path of SafeCollab end-to-end on a real machine:

```
gz camera sensor ──(ros_gz_image bridge)──▶ /camera/image
        │
        ▼
perception_node ── classical CV (yellow HSV) ──▶ TF world→human  +  /human/uncertainty
        │
        ▼
safety_monitor (safety_source:=perceived) ──▶ /safety/zone, /safety/scale
```

**Why this is needed.** The CI pipeline proves the acceptance behaviour
(AT‑1..AT‑5) using `safety_source:=ground_truth`, which feeds the safety monitor
the simulator's exact operator pose and therefore **bypasses perception
entirely**. The geometry of `perception_node` is unit‑tested (sub‑mm
pixel↔world round‑trips), but nothing in CI runs the *live* camera → detector →
SSM loop. This procedure does, by comparing the **perceived** human pose against
the simulator's **ground‑truth** pose (`world→human_gt`) while the cell runs.

> Running anything here touches **Docker only**. It never touches git and never
> pushes. The container is the same image CI builds from `Dockerfile`.

---

## 1. Prerequisites (Arch Linux / Omarchy)

Omarchy is Arch‑based and ships Hyprland (Wayland). You do **not** need to
install ROS 2 or Gazebo natively — they live inside the Docker image. You only
need Docker, plus a couple of X utilities if you want the GUI (§5).

```bash
# Docker engine
sudo pacman -S --needed docker
sudo systemctl enable --now docker

# Let your user talk to the daemon without sudo (log out / back in afterwards,
# or run `newgrp docker` in the current shell).
sudo usermod -aG docker "$USER"

# Optional, only for the GUI mode in §5:
sudo pacman -S --needed xorg-xhost
```

Verify Docker is healthy:

```bash
docker info >/dev/null && echo "docker OK"
```

> **GPU note.** The gz camera sensor must render to produce frames. If your
> laptop exposes `/dev/dri` (almost all do — Intel/AMD/NVIDIA with the open
> stack), the script passes it through for hardware GL. If not, it falls back to
> software rendering (`LIBGL_ALWAYS_SOFTWARE=1`), which works but is slower and
> the camera may lag. On NVIDIA‑proprietary setups you may instead want the
> NVIDIA Container Toolkit (`nvidia-container-toolkit`) and `--gpus all`; the
> open `nouveau`/`/dev/dri` path needs nothing extra.

---

## 2. Get the code onto the laptop

```bash
git clone <your-remote-or-copy-the-repo> SafeCollab
cd SafeCollab
```

If you are moving it from the Windows box without a remote, just copy the
working tree across; the validation needs no git history.

---

## 3. The one‑command validation (recommended)

```bash
scripts/validate-perceived.sh
```

This builds the image (first run only takes a few minutes), starts the cell
**headless** with `safety_source:=perceived` and a fixed `path_seed`, waits for
bring‑up, runs four checks, prints a `PASS`/`FAIL` summary, and tears the
container down. It exits `0` only if every check passes.

Useful flags:

| Flag | Effect |
|------|--------|
| `--no-build` | reuse an existing `safecollab:dev` image (skip the rebuild) |
| `--window N` | observe `/safety` for `N` seconds (default 40) |
| `--seed N` | use a different deterministic operator `path_seed` |
| `--keep` | leave the container running afterwards so you can poke at it |
| `-h` | help |

### What the four checks mean

1. **`/camera/image` is publishing** — the gz camera sensor and `ros_gz_image`
   bridge are alive. If this fails, perception has no input (see §6).
2. **Perceived TF `world→human` exists** — `perception_node` detected the yellow
   operator and is broadcasting its estimated pose.
3. **Perceived ≈ ground truth** — the heart of the test. The perceived
   `world→human` is compared against the simulator's `world→human_gt`; the
   minimum separation over several samples must be ≤ `0.15 m`. This is what
   actually proves the camera calibration *and* the detector are correct live,
   not just in unit tests.
4. **The SSM loop reacts** — over the observation window the operator's path
   carries it toward the arm; `/safety/zone` must leave `green` (reach
   `yellow`/`red`) and `/safety/scale` must drop below `1.0`. A `lost` zone here
   means perception dropped out mid‑run (fail‑safe engaged) and is reported as a
   failure.

A green run prints:

```
PERCEIVED PATH VALIDATED — camera -> perception -> SSM loop closes live.
```

---

## 4. Doing it by hand (to understand / debug)

If you want to watch it yourself instead of (or alongside) the script:

```bash
# Build once.
docker build -t safecollab:dev .

# Terminal 1 — run the cell headless with the PERCEIVED source.
docker run --rm --init --name safecollab --device /dev/dri \
    safecollab:dev \
    ros2 launch safecollab cell.launch.py \
        headless:=true safety_source:=perceived path_seed:=42
```

```bash
# Terminal 2 — introspect. `docker exec` does NOT run the image entrypoint, so
# invoke it explicitly to source ROS + the colcon overlay, then exec bash:
docker exec -it safecollab /entrypoint.sh bash

# 1. camera frames flowing?  expect roughly the sensor's rate
ros2 topic hz /camera/image

# 2. perception producing a perceived human?
ros2 topic echo /human/uncertainty --once
ros2 run tf2_ros tf2_echo world human        # PERCEIVED  (from perception_node)
ros2 run tf2_ros tf2_echo world human_gt     # GROUND TRUTH (from human_node)
#    ^ the two translations should agree within a few centimetres

# 3. the SSM loop reacting to the perceived human as the operator approaches
ros2 topic echo /safety/zone     # green -> yellow -> red
ros2 topic echo /safety/scale    # 1.0 -> down toward s_min
```

**Pass criteria, restated:** `/camera/image` ticks; `world→human` exists and
stays within a few cm of `world→human_gt`; `/safety/zone` transitions and
`/safety/scale` drops below `1.0` as the operator nears the arm. If
`world→human` is missing or `/safety/scale` is pinned at `0`, perception is not
detecting — operator out of FOV, or the HSV thresholds aren't catching the
yellow body (calibration is already fixed; detection is a separate stage).

---

## 5. Watching it in the Gazebo GUI (optional)

Headless is enough to *validate*, but seeing it helps intuition. Under Hyprland,
GUI apps run through XWayland, so X11 passthrough is the simplest route.

```bash
# Allow local containers to reach your X server (XWayland under Hyprland).
xhost +local:docker

docker run --rm --init \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri \
    --network host \
    safecollab:dev \
    ros2 launch safecollab cell.launch.py headless:=false safety_source:=perceived

# When finished, revoke the X grant:
xhost -local:docker
```

You should see the cell, the yellow operator body moving along its path, and —
if you also open RViz / echo the topics from a second `docker exec` — the zones
changing as it approaches. If the gz window is black or GL errors appear, your
X/DRI passthrough isn't reaching the GPU; fall back to the headless validation
in §3, which doesn't need a display.

---

## 6. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `Cannot talk to the Docker daemon` | `sudo systemctl start docker`; ensure your user is in the `docker` group (`newgrp docker`). |
| `/camera/image` shows 0 Hz | The gz camera sensor isn't rendering. Pass `--device /dev/dri` (the script does this automatically when present); otherwise the script sets `LIBGL_ALWAYS_SOFTWARE=1`. Give it more start‑up time — software GL is slow. |
| No `world→human` TF | Perception isn't detecting. Confirm `/camera/image` is live first; the yellow operator may be outside the camera FOV, or the HSV hue band isn't matching the body colour. |
| `/safety/scale` pinned at `0`, zone `lost` | Fail‑safe‑on‑lost: the perceived `world→human` is missing/stale beyond `loss_timeout` (see `config/safety.yaml`). This means CHECK 2 failed — perception isn't producing a pose. |
| Perceived vs ground‑truth far apart | Camera pose / projection mismatch. The calibration lives in `perception_node.py` (`cam_to_world_transform`, `project_pixel_to_plane`); the plane height is `plane_z_m` (hand reach ≈ 0.82 m). |
| GUI window is black / GL errors | X/DRI passthrough isn't reaching the GPU under XWayland. Re‑check `xhost +local:docker`, `DISPLAY`, the `/tmp/.X11-unix` mount, and `--device /dev/dri`. Headless (§3) avoids this entirely. |
| Container exits immediately | `docker logs safecollab` (or the tail the script prints) shows the launch error. |

---

## 7. What a successful validation establishes

A green run is the evidence the project's perception milestone (v0.3.0, P2) is
actually earned: the **perceived** human — not the ground‑truth shortcut — drives
the SSM loop, satisfying the "perceived human in the headline demo" Definition
of Done. Until this passes live, the perception path is proven only at the unit
(geometry) level.
