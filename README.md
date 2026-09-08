# rtcosmik_ros

ROS 2 overlay for [RT-COSMIK](https://gitlab.laas.fr/msabbah/rt-cosmik): runs the
live pipeline and publishes the result for RViz and for control.

The node is a thin wrapper. Calibration and IK live in
`rtcosmik.pipeline.solver`, capture in `rtcosmik.camera`, so this package tracks
changes to the toolbox without being edited.

## Build and run

```bash
cd <ws> && colcon build --packages-select rtcosmik_ros
source install/setup.bash
ros2 launch rtcosmik_ros start.launch.py
```

RT-COSMIK must be importable (`pip install -e .` in the rt-cosmik checkout), its
models fetched (`scripts/bash/fetch_models.sh`), and — for `ik_type = "mhe"` —
its OCP generated (`scripts/python/core/run_ocp_codegen.py`). `ffmpeg` must be on
`PATH`: cameras are opened through it, not OpenCV.

With the acados backend, `ACADOS_SOURCE_DIR` and the acados `lib/` directory
must be visible to the node. `ros2 launch` inherits the shell environment, so a
normal session is fine; a service started by systemd is not, and will fail at
solver construction.

## Launch arguments

| argument | default | |
|---|---|---|
| `use_rviz` | `true` | start RViz alongside the bridge |
| `cam_calib_path` | *(settings)* | camera calibration directory, COMFI layout |
| `replay_dir` | *(none)* | replay recordings instead of opening cameras |
| `robot_description_topic` | | topic for the scaled URDF |

Everything else is configuration and lives in RT-COSMIK's `settings.py` --
cameras, backend, solver profile, recording. Only paths are arguments here.

## Published topics

| topic | type | |
|---|---|---|
| `/rtcosmik/q` | `Float64MultiArray` | full configuration vector |
| `/rtcosmik/joint_states` | `JointState` | for `robot_state_publisher` |
| `/rtcosmik/markers` | `MarkerArray` | estimated markers |
| `/rtcosmik/collision_poses` | `PoseArray` | collision capsule poses |
| `/rtcosmik/collision_markers` | `MarkerArray` | the same, drawable |

The node also writes a scaled URDF for the calibrated subject and hands it to
`robot_state_publisher`, so RViz shows a body the right size rather than the
default one.

## Testing without a rig

Recordings can be replayed through the live path, paced at their own frame rate:

```bash
ros2 launch rtcosmik_ros start.launch.py \
    replay_dir:=<dataset>/videos/<participant>/<task> \
    cam_calib_path:=<dataset>/cam_params/<participant>
```

Files stand in for `/dev/video*`; the camera processes, barrier, shared buffers
and pipeline are the real ones. The sources hold their first frame until the
model is calibrated, matching a subject who stands still for it -- otherwise the
trial runs on during calibration and the model is scaled from whatever pose it
lands on.

This exercises the software path, not the capture hardware: every file source is
always ready, so the barrier never waits and real inter-camera skew is invisible.

## Recording

Driven by RT-COSMIK's `settings.py`, not by launch arguments:

- `SAVE_CSV` -- markers and joint angles, with a frame counter per camera
- `SAVE_VID` -- one `camera_<id>.mkv` per camera, a stream copy of the camera's
  own MJPEG, so no decode and no re-encode
- `record_on_start` -- begin immediately; otherwise recording starts off
- `SAVE_DIR` -- where they go

The keyboard toggle belongs to the standalone pipeline, which owns a terminal.
A launched node does not, so set `record_on_start = True` here.
