# rtcosmik_ros

ROS 2 overlay for [RT-COSMIK](https://github.com/Gepetto/rt-cosmik): runs the
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

RT-COSMIK's Docker image has all of this, and ROS 2 Humble. Clone this
repository next to `rt-cosmik`: `rt-cosmik/docker/run.sh` mounts it at
`/root/workspace/ros_ws/src/rtcosmik_ros`, so `<ws>` above is
`/root/workspace/ros_ws`. See RT-COSMIK's
[installation guide](https://github.com/Gepetto/rt-cosmik/blob/main/docs/installation.md).

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

## Cameras

The node opens the cameras of `settings.cameras`, which are calibrated camera
ids: the `camera_<id>` of the calibration files. Each attached device is
recognised by its USB port when the calibration has a `cameras.yaml` (as
cams_calibration writes), so a recabled rig keeps its calibration; anything
else plugged in, such as a laptop webcam, stays closed. A requested camera that
is not attached stops the node with the list of what is attached. If the first
camera has no world pose, a requested camera that has one becomes the
reference, so positions stay in room coordinates. Recordings are named after
the calibrated ids. This is RT-COSMIK's own selection
(`rtcosmik.camera.cam_utils.select_live_cameras`), the same as
`run_pipeline.py --online`.

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

RT-COSMIK's sample trial (`scripts/bash/fetch_sample.sh` in rt-cosmik) replays as
is with the default `settings.cameras = (0, 2, 4, 6)`. The subject comes from
`settings.py`: set `human_height = 1.77`, `human_weight = 62.0` and
`human_gender = 'f'` to match its participant.

```bash
ros2 launch rtcosmik_ros start.launch.py \
    replay_dir:=<rt-cosmik>/data/comfi_sample/videos/2112/RobotWelding \
    cam_calib_path:=<rt-cosmik>/data/comfi_sample/cam_params/2112
```

## Recording

Driven by RT-COSMIK's `settings.py`, not by launch arguments:

- `SAVE_CSV` -- markers and joint angles, with a frame counter per camera
- `SAVE_VID` -- one `camera_<id>.mkv` per camera, a stream copy of the camera's
  own MJPEG, so no decode and no re-encode
- `record_on_start` -- begin immediately; otherwise recording starts off
- `SAVE_DIR` -- where they go

The keyboard toggle belongs to the standalone pipeline, which owns a terminal.
A launched node does not, so set `record_on_start = True` here.
