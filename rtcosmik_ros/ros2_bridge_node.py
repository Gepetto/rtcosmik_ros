#!/usr/bin/env python3
"""ROS 2 bridge: publish RT-COSMIK outputs for visualization and control."""

from datetime import datetime
from multiprocessing import Event as MPEvent
import os
import shutil
import threading
import time
import traceback
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time as TimeMsg

from ament_index_python.packages import get_package_share_directory

from rtcosmik.camera.camera import Camera
from rtcosmik.camera.cam_utils import (
    anchor_first,
    load_camera_parameters,
    load_world_transformation,
    select_live_cameras,
)
from rtcosmik.config_loader import settings
from rtcosmik.filtering.iir import MarkerFilter
from rtcosmik.pipeline.solver import HumanSolver
from rtcosmik.saver.recorder import Recorder
from rtcosmik.model_weights import resolve_detector_engine
from rtcosmik.nlf.nlf import NLFEstimator, extract_views
from rtcosmik.triangulation.triangulation import reconstruct_3d
from rtcosmik.utils.mp_utils import create_camera_shared_ressources
from rtcosmik_ros.urdf_export import save_scaled_urdf

INIT_DONE_TOKEN = "RTCOSMIK_INIT_DONE"


class RTCosmikMarkerBridge(Node):
    def __init__(self):
        super().__init__('rtcosmik_marker_bridge')

        self.enable_markers = True
        self.publish_base_tf = True
        # RT-COSMIK output is expressed in calibration world frame.
        self.world_frame_id = 'world'
        self.scaled_urdf_output_path = self._default_human_urdf_path().with_name('human_scaled.urdf')
        self.joint_names = self._load_joint_names_from_urdf(self._default_human_urdf_path())
        self.base_frame_id = self._load_root_link_from_urdf(self._default_human_urdf_path())
        self._source_scaled_urdf_output_path = self._find_workspace_source_scaled_urdf_path()
        self._reliable_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.q_publisher_ = self.create_publisher(Float64MultiArray, '/rtcosmik/q', self._reliable_qos)
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            '/rtcosmik/joint_states',
            self._reliable_qos,
        )
        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/rtcosmik/markers',
            self._reliable_qos,
        )
        self.collision_pose_array_publisher_ = self.create_publisher(
            PoseArray,
            '/rtcosmik/collision_poses',
            self._reliable_qos,
        )
        self.collision_marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/rtcosmik/collision_markers',
            self._reliable_qos,
        )
        self._collision_segment_pose_publishers = []
        self.tf_broadcaster_ = TransformBroadcaster(self) if self.publish_base_tf else None

        self.stop_event = None
        self.processes = []
        self._runtime_thread = None
        self._init_done_announced = False
        self._frame_shape = (settings.height, settings.width, 3)
        self._num_cameras = 0
        self._camera_buffers = []
        self._camera_timestamps = []
        self._camera_locks = []
        self._frame_counters = []
        self._last_frame_counters = []
        self._mtxs = None
        self._dists = None
        self._projections = None
        self._world_r1_cam = np.eye(3, dtype=float)
        self._world_t1_cam = np.zeros(3, dtype=float)
        self._max_frame_skew_s = 0.050
        self._last_skew_warn_t = 0.0
        self._has_freeflyer_model = False
        self._last_invalid_warn_t = 0.0
        self._ff_rotation_correction = pin.utils.rotate('x', np.pi / 2.0)
        self._collision_capsule_specs = [
            {
                'name': 'right_upperarm_capsule',
                'segment_name': 'right_upperarm',
                'start_frame': 'right_upperarm',
                'end_frame': 'right_lowerarm',
                'radius': 0.045,
                'length_scale': 0.95,
                'min_length': 0.08,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.22,
            },
            {
                'name': 'right_lowerarm_capsule',
                'segment_name': 'right_lowerarm',
                'start_frame': 'right_lowerarm',
                'end_frame': 'right_hand',
                'radius': 0.035,
                'length_scale': 0.95,
                'min_length': 0.07,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.20,
            },
            {
                'name': 'right_hand_capsule',
                'segment_name': 'right_hand',
                'start_frame': 'right_hand',
                'end_frame': None,
                'radius': 0.03,
                'length_scale': 1.0,
                'min_length': 0.06,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.12,
            },
            {
                'name': 'left_upperarm_capsule',
                'segment_name': 'left_upperarm',
                'start_frame': 'left_upperarm',
                'end_frame': 'left_lowerarm',
                'radius': 0.045,
                'length_scale': 0.95,
                'min_length': 0.08,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.22,
            },
            {
                'name': 'left_lowerarm_capsule',
                'segment_name': 'left_lowerarm',
                'start_frame': 'left_lowerarm',
                'end_frame': 'left_hand',
                'radius': 0.035,
                'length_scale': 0.95,
                'min_length': 0.07,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.20,
            },
            {
                'name': 'left_hand_capsule',
                'segment_name': 'left_hand',
                'start_frame': 'left_hand',
                'end_frame': None,
                'radius': 0.03,
                'length_scale': 1.0,
                'min_length': 0.06,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.12,
            },
            {
                'name': 'middle_thorax_capsule',
                'segment_name': 'middle_thorax',
                'start_frame': 'middle_thorax',
                'end_frame': None,
                'radius': 0.12,
                'length_scale': 1.0,
                'min_length': 0.12,
                'fallback_local_axis': [0.0, -1.0, 0.0],
                'default_length': 0.40,
            },
            {
                'name': 'middle_thorax_capsule2',
                'segment_name': 'middle_thorax',
                'start_frame': 'middle_thorax',
                'end_frame': None,
                'radius': 0.14,
                'length_scale': 1.0,
                'min_length': 0.12,
                'fallback_local_axis': [0.0, 1.0, 0.0],
                'default_length': 0.24,
            },
            {
                'name': 'head_capsule',
                'segment_name': 'head',
                'start_frame': 'middle_head',
                'end_frame': None,
                'radius': 0.09,
                'length_scale': 1.0,
                'min_length': 0.10,
                'fallback_local_axis': [0.0, 1.0, 0.0],
                'default_length': 0.25,
            },

        ]
        self._collision_segment_pose_publishers = [
            self.create_publisher(
                Pose,
                f"/rtcosmik/collision_pose/{spec['segment_name']}",
                self._reliable_qos,
            )
            for spec in self._collision_capsule_specs
        ]
        self._collision_capsule_frame_ids = []

        self._start_rtcosmik_runtime()

        self._runtime_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._runtime_thread.start()

        self.get_logger().info(
            'RT-COSMIK bridge started in initialization mode. '
            'Publishers and RViz are deferred until calibration is finished.'
        )

    def _default_human_urdf_path(self):
        try:
            share_dir = Path(get_package_share_directory('rtcosmik_ros'))
            return share_dir / 'urdf' / 'human.urdf'
        except Exception:
            return Path(__file__).resolve().parents[1] / 'urdf' / 'human.urdf'

    def _load_joint_names_from_urdf(self, urdf_path: Path):
        if not urdf_path.exists():
            self.get_logger().warning(f'Could not find URDF at {urdf_path} to auto-load joint names.')
            return []

        root = ET.fromstring(urdf_path.read_text())
        names = []
        for joint in root.findall('joint'):
            joint_type = joint.attrib.get('type', '')
            if joint_type in {'revolute', 'continuous', 'prismatic'}:
                names.append(joint.attrib['name'])
        return names

    def _load_root_link_from_urdf(self, urdf_path: Path):
        if not urdf_path.exists():
            self.get_logger().warning(f'Could not find URDF at {urdf_path} to auto-load root link.')
            return 'middle_pelvis'

        root = ET.fromstring(urdf_path.read_text())
        link_names = [link.attrib['name'] for link in root.findall('link') if 'name' in link.attrib]
        child_links = set()
        for joint in root.findall('joint'):
            child = joint.find('child')
            if child is not None and 'link' in child.attrib:
                child_links.add(child.attrib['link'])

        roots = [name for name in link_names if name not in child_links]
        if roots:
            return roots[0]
        return 'middle_pelvis'

    def _find_workspace_source_scaled_urdf_path(self):
        """
        Locate workspace source URDF path to mirror generated scaled model for visibility.
        Expected layout: <ws>/src/rtcosmik_ros/urdf/human_scaled.urdf
        """
        here = Path(__file__).resolve()
        for ancestor in here.parents:
            candidate_dir = ancestor / 'src' / 'rtcosmik_ros' / 'urdf'
            if candidate_dir.is_dir():
                return candidate_dir / 'human_scaled.urdf'
        return None

    def _start_rtcosmik_runtime(self):
        width, height = settings.width, settings.height
        self._frame_shape = (height, width, 3)

        # Replaying recordings through the live path is how this node is tested
        # without a rig. A path, so it is a node parameter; recording itself is
        # configuration and stays in settings.py.
        self.declare_parameter('replay_dir', '')
        replay_dir = self.get_parameter('replay_dir').value or ''

        # The launch file always forwards this argument, so an override that was
        # not given arrives as an empty string rather than as an unset variable.
        # Treating that as a path silently rebases the whole calibration on the
        # working directory, where it is only found by accident.
        cam_calib_path = os.getenv('RTCOSMIK_CAM_CALIB_PATH') or settings.cam_calib_path
        if cam_calib_path != settings.cam_calib_path:
            self.get_logger().info(
                f'Overriding RT-COSMIK camera calibration path from environment: {cam_calib_path}'
            )
            settings.cam_calib_path = cam_calib_path

        # settings.cameras names calibrated cameras: the camera_<id> of the
        # calibration files. Only the reference camera's world pose is read, and
        # a rig calibration anchors a single camera, so the reference must be
        # that one.
        cam_ids = anchor_first(cam_calib_path, settings.cameras)
        if not cam_ids:
            raise RuntimeError('settings.cameras is empty; at least one camera is required.')
        if replay_dir:
            sources = [os.path.join(replay_dir, f'camera_{c}.mp4') for c in cam_ids]
            missing = [s for s in sources if not os.path.isfile(s)]
            if missing:
                raise FileNotFoundError(
                    f'missing recordings for replay: {missing}')
            self.get_logger().info(
                f'Replaying {len(sources)} recordings from {replay_dir} as live cameras')
        else:
            # Open the device behind each requested camera -- recognised by its
            # USB port when the calibration records one, so a recabled rig keeps
            # its calibration -- and nothing else that happens to be plugged in.
            sources = [f'/dev/video{index}'
                       for index in select_live_cameras(cam_calib_path, cam_ids)]
        self._camera_ids = cam_ids
        self._num_cameras = len(cam_ids)
        self.get_logger().info(
            f'Using {self._num_cameras} camera(s): calibrated cameras {cam_ids} '
            f'from {", ".join(sources)}'
        )

        self._mtxs, self._dists, self._projections, _, _ = load_camera_parameters(
            cam_calib_path, camera_ids=cam_ids)
        try:
            self._world_r1_cam, self._world_t1_cam = load_world_transformation(
                cam_calib_path, ref_camera=cam_ids[0])
            self._world_r1_cam = np.asarray(self._world_r1_cam, dtype=float).reshape(3, 3)
            self._world_t1_cam = np.asarray(self._world_t1_cam, dtype=float).reshape(3)
        except Exception as exc:
            self.get_logger().warning(
                f'Could not load world transform, falling back to identity: {exc}'
            )
            self._world_r1_cam = np.eye(3, dtype=float)
            self._world_t1_cam = np.zeros(3, dtype=float)

        (
            self._camera_buffers,
            self._camera_timestamps,
            self._camera_locks,
            self._frame_counters,
            camera_barrier,
            self.stop_event,
        ) = create_camera_shared_ressources(self._num_cameras, self._frame_shape)

        # Replay sources hold their first frame until the model is calibrated:
        # a real subject stands still for it, so a recording must not run on.
        self._calibrated_event = MPEvent()

        # Video recording is a stream copy alongside capture, so it costs no
        # decode and no re-encode.
        record_paths = [None] * self._num_cameras
        if settings.SAVE_VID:
            os.makedirs(settings.SAVE_DIR, exist_ok=True)
            record_paths = [os.path.join(settings.SAVE_DIR, f'camera_{c}.mkv')
                            for c in self._camera_ids]
            self.get_logger().info(f'Recording video to {settings.SAVE_DIR}')

        camera_processes = [
            # Named by calibrated camera; the device (or recording) it reads
            # is its source.
            Camera(
                cam_id=self._camera_ids[i],
                shared_buffer=self._camera_buffers[i],
                timestamp_buffer=self._camera_timestamps[i],
                lock=self._camera_locks[i],
                frame_counter=self._frame_counters[i],
                barrier=camera_barrier,
                stop_event=self.stop_event,
                frame_shape=self._frame_shape,
                cam_fps=settings.fs,
                cam_fourcc=settings.fourcc,
                source=sources[i],
                record_path=record_paths[i],
                realtime=bool(replay_dir),
                calibrated_event=self._calibrated_event if replay_dir else None,
            )
            for i in range(self._num_cameras)
        ]

        self._last_frame_counters = [0] * self._num_cameras
        self.processes = camera_processes
        for process in self.processes:
            process.start()

    def _frame_counter_values(self):
        """Per-camera frame counters, for the recorded rows."""
        return [int(c.value) for c in self._frame_counters]

    def _assert_camera_processes_alive(self):
        dead = []
        for process in self.processes:
            if not process.is_alive() and process.exitcode is not None:
                dead.append((process.name, process.exitcode))

        if dead:
            if self.stop_event is not None:
                self.stop_event.set()
            raise RuntimeError(f'Camera process failure detected: {dead}')

    def _timestamps_are_coherent(self, timestamps):
        parsed = []
        for ts in timestamps:
            try:
                parsed.append(datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f"))
            except Exception:
                return True

        if len(parsed) < 2:
            return True

        skew_s = (max(parsed) - min(parsed)).total_seconds()
        if skew_s <= self._max_frame_skew_s:
            return True

        now_t = time.monotonic()
        if now_t - self._last_skew_warn_t > 1.0:
            self._last_skew_warn_t = now_t
            self.get_logger().warning(
                f'Skipping desynchronized camera bundle (skew={skew_s*1000.0:.1f} ms).'
            )
        return False

    def _build_bundle_stamp(self, timestamps):
        parsed = []
        for ts in timestamps:
            try:
                parsed.append(datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f"))
            except Exception:
                continue

        if not parsed:
            return self.get_clock().now().to_msg()

        avg_epoch_s = sum(dt.timestamp() for dt in parsed) / len(parsed)
        sec = int(avg_epoch_s)
        nanosec = int((avg_epoch_s - sec) * 1e9)
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _read_synchronized_frames(self):
        frames = [None] * self._num_cameras
        new_counters = self._last_frame_counters.copy()
        timestamps = [None] * self._num_cameras
        for i, (lock, buffer, cam_ts, frame_counter) in enumerate(
            zip(
                self._camera_locks,
                self._camera_buffers,
                self._camera_timestamps,
                self._frame_counters,
            )
        ):
            with lock:
                if frame_counter.value <= self._last_frame_counters[i]:
                    continue

                timestamp = bytes(cam_ts[:]).decode().strip('\x00')
                if timestamp == '':
                    continue

                arr = np.frombuffer(buffer, dtype=np.uint8)
                frame = arr.reshape(self._frame_shape).copy()
                frames[i] = frame
                new_counters[i] = frame_counter.value
                timestamps[i] = timestamp

        if any(frame is None for frame in frames):
            return None

        self._last_frame_counters = new_counters
        return frames, self._build_bundle_stamp(timestamps)

    def _write_scaled_urdf_from_pin_model(
        self,
        human_model,
        urdf_output_path: Path,
        human_visual_model=None,
        human_collision_model=None,
    ):
        """Serialize calibrated Pinocchio model to URDF used by robot_state_publisher."""
        output_path = save_scaled_urdf(
            new_model_name='human_scaled',
            new_model_path=urdf_output_path,
            scaled_model=human_model,
            visual_model=human_visual_model,
            collision_model=human_collision_model,
        )
        self.get_logger().info(f'Scaled URDF generated at {output_path}')

        if (
            self._source_scaled_urdf_output_path is not None
            and self._source_scaled_urdf_output_path.resolve() != output_path.resolve()
        ):
            self._source_scaled_urdf_output_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(output_path, self._source_scaled_urdf_output_path)
            self.get_logger().info(
                f'Scaled URDF mirrored to source tree at {self._source_scaled_urdf_output_path}'
            )

    def _model_has_freeflyer(self, model):
        if model is None or model.njoints <= 1:
            return False
        shortname_j1 = model.joints[1].shortname()
        return ("FreeFlyer" in shortname_j1) or ("FF" in shortname_j1)

    def _announce_initialization_done(self):
        if self._init_done_announced:
            return

        loaded_joint_names = self._load_joint_names_from_urdf(self.scaled_urdf_output_path)
        if loaded_joint_names:
            self.joint_names = loaded_joint_names
        self.base_frame_id = self._load_root_link_from_urdf(self.scaled_urdf_output_path)

        self._init_done_announced = True
        self.get_logger().info('Calibration and model initialization completed.')
        # Plain stdout token used by launch OnProcessIO to trigger RViz and robot_state_publisher.
        print(INIT_DONE_TOKEN, flush=True)

    def _warn_throttled(self, message: str, period_s: float = 1.0):
        now_t = time.monotonic()
        if now_t - self._last_invalid_warn_t > period_s:
            self._last_invalid_warn_t = now_t
            self.get_logger().warning(message)

    def _setup_collision_capsules(self, model):
        self._collision_capsule_frame_ids = []
        for spec in self._collision_capsule_specs:
            start_id = model.getFrameId(spec['start_frame'])
            end_name = spec['end_frame']
            end_id = model.getFrameId(end_name) if end_name else None
            if start_id >= len(model.frames):
                self.get_logger().warning(
                    f"Collision capsule frame not found: {spec['start_frame']}. Skipping."
                )
                self._collision_capsule_frame_ids.append((None, None))
                continue
            if end_id is not None and end_id >= len(model.frames):
                self.get_logger().warning(
                    f"Collision capsule frame not found: {end_name}. Using fallback axis."
                )
                end_id = None
            self._collision_capsule_frame_ids.append((start_id, end_id))

    def _quat_xyzw_from_two_vectors(self, vec_from, vec_to):
        a = np.asarray(vec_from, dtype=float)
        b = np.asarray(vec_to, dtype=float)
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na < 1e-12 or nb < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        a /= na
        b /= nb
        dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
        if dot > 1.0 - 1e-10:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        if dot < -1.0 + 1e-10:
            ortho = np.array([1.0, 0.0, 0.0], dtype=float)
            if abs(a[0]) > 0.9:
                ortho = np.array([0.0, 1.0, 0.0], dtype=float)
            axis = np.cross(a, ortho)
            axis /= np.linalg.norm(axis)
            return np.array([axis[0], axis[1], axis[2], 0.0], dtype=float)

        axis = np.cross(a, b)
        s = np.sqrt((1.0 + dot) * 2.0)
        invs = 1.0 / s
        q = np.array([axis[0] * invs, axis[1] * invs, axis[2] * invs, 0.5 * s], dtype=float)
        q /= np.linalg.norm(q)
        return q

    def _publish_collision_capsules(self, model, data, q_array, stamp):
        if model is None or data is None:
            return
        if q_array.size < model.nq or not np.all(np.isfinite(q_array[:model.nq])):
            return
        if not self._collision_capsule_frame_ids:
            return

        q_model = np.array(q_array[:model.nq], dtype=float, copy=True)
        try:
            q_model = pin.normalize(model, q_model)
            pin.forwardKinematics(model, data, q_model)
            pin.updateFramePlacements(model, data)
        except Exception as exc:
            self._warn_throttled(f'Skipping collision capsule update: {exc}')
            return

        pose_array = PoseArray()
        pose_array.header = Header(stamp=stamp, frame_id=self.world_frame_id)
        marker_array = MarkerArray()
        axis_z = np.array([0.0, 0.0, 1.0], dtype=float)
        for idx, spec in enumerate(self._collision_capsule_specs):
            start_id, end_id = self._collision_capsule_frame_ids[idx]
            if start_id is None:
                continue
            start_pose = data.oMf[start_id]
            start_pos = np.asarray(start_pose.translation, dtype=float).reshape(3)

            if end_id is not None:
                end_pos = np.asarray(data.oMf[end_id].translation, dtype=float).reshape(3)
                direction = end_pos - start_pos
                direction_norm = np.linalg.norm(direction)
                if direction_norm < 1e-9:
                    local_axis = np.asarray(spec['fallback_local_axis'], dtype=float)
                    direction = start_pose.rotation @ local_axis
                    direction_norm = np.linalg.norm(direction)
                if direction_norm < 1e-9:
                    continue
                axis_dir = direction / direction_norm
                length = max(spec['min_length'], direction_norm * spec['length_scale'])
                center = 0.5 * (start_pos + end_pos)
            else:
                local_axis = np.asarray(spec['fallback_local_axis'], dtype=float)
                axis_dir = start_pose.rotation @ local_axis
                axis_dir_norm = np.linalg.norm(axis_dir)
                if axis_dir_norm < 1e-9:
                    continue
                axis_dir /= axis_dir_norm
                length = spec['default_length']
                center = start_pos + 0.5 * length * axis_dir

            quat_xyzw = self._quat_xyzw_from_two_vectors(axis_z, axis_dir)

            pose_msg = Pose()
            pose_msg.position.x = float(center[0])
            pose_msg.position.y = float(center[1])
            pose_msg.position.z = float(center[2])
            pose_msg.orientation.x = float(quat_xyzw[0])
            pose_msg.orientation.y = float(quat_xyzw[1])
            pose_msg.orientation.z = float(quat_xyzw[2])
            pose_msg.orientation.w = float(quat_xyzw[3])
            pose_array.poses.append(pose_msg)

            marker = Marker()
            marker.header = Header(stamp=stamp, frame_id=self.world_frame_id)
            marker.ns = 'rtcosmik_collision'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose_msg
            marker.scale.x = float(2.0 * spec['radius'])
            marker.scale.y = float(2.0 * spec['radius'])
            marker.scale.z = float(length)
            marker.color.r = 0.15
            marker.color.g = 0.55
            marker.color.b = 1.0
            marker.color.a = 0.65
            marker_array.markers.append(marker)

            if idx < len(self._collision_segment_pose_publishers):
                self._collision_segment_pose_publishers[idx].publish(pose_msg)

        self.collision_pose_array_publisher_.publish(pose_array)
        self.collision_marker_publisher_.publish(marker_array)

    def _processing_loop(self):
        first_sample = True
        recorder = Recorder(settings, self._num_cameras,
                            logger=self.get_logger()).start()
        self._recorder = recorder
        solver = HumanSolver(settings, logger=self.get_logger())
        human_model = None
        human_data = None
        last_health_check_t = 0.0

        try:
            estimator = NLFEstimator(
                yolo_path=resolve_detector_engine(settings.yolo_path, self._num_cameras),
                nlf_path=settings.nlf_path,
                cano_path=settings.cano_path,
                image_size=(self._frame_shape[1], self._frame_shape[0]),
                cam_Ks=self._mtxs,
                indices=settings.nlf_indices,
                conf=settings.yolo_conf,
                imgsz=settings.yolo_imgsz,
                device=settings.device,
            )

            marker_filter = MarkerFilter(len(settings.marker_names), settings)

            while self.stop_event is not None and not self.stop_event.is_set():
                now_t = time.monotonic()
                if now_t - last_health_check_t > 1.0:
                    self._assert_camera_processes_alive()
                    last_health_check_t = now_t

                frame_bundle = self._read_synchronized_frames()
                if frame_bundle is None:
                    time.sleep(0.001)
                    continue
                frames, frame_stamp = frame_bundle

                nlf_out, _, _, _ = estimator.estimate_from_frames(frames)
                views = extract_views(nlf_out, self._num_cameras)
                p3d = reconstruct_3d(views, self._projections)
                if len(p3d) == 0:
                    continue

                # Points come back in the reference camera frame; convert to world.
                p3d_cam0 = np.asarray(p3d, dtype=np.float32)
                p3d_in_world = np.array(
                    [self._world_r1_cam @ point + self._world_t1_cam for point in p3d_cam0],
                    dtype=np.float32,
                )

                # One frame in, one filtered frame out. IIR.filter is stateful,
                # so it must see each sample exactly once.
                augmented_markers = marker_filter(p3d_in_world)
                mks_dict = dict(zip(settings.marker_names, augmented_markers))

                if first_sample:
                    # Calibration and IK live in rtcosmik.pipeline.solver, so this
                    # node tracks changes to either without being edited.
                    q = solver.calibrate(mks_dict)
                    human_model, human_data = solver.model, solver.data
                    self._has_freeflyer_model = self._model_has_freeflyer(human_model)

                    self._write_scaled_urdf_from_pin_model(
                        human_model,
                        self.scaled_urdf_output_path,
                        human_visual_model=solver.visual_model,
                        human_collision_model=solver.collision_model,
                    )
                    self._setup_collision_capsules(human_model)
                    self._announce_initialization_done()
                    # Lets the replay sources advance past their held frame.
                    self._calibrated_event.set()
                    first_sample = False
                    continue

                q = solver.step(mks_dict)
                recorder.record(self._frame_counter_values(), mks_dict, q)

                self._publish_outputs(
                    mks_dict,
                    q,
                    stamp=frame_stamp,
                    human_model=human_model,
                    human_data=human_data,
                )

        except Exception as exc:
            if self.stop_event is not None:
                self.stop_event.set()
            self.get_logger().error(
                f'RT-COSMIK runtime loop crashed: {exc}\n{traceback.format_exc()}'
            )
        finally:
            self.get_logger().info('RT-COSMIK runtime loop terminated.')

    def _publish_outputs(self, markers, q_values, stamp=None, human_model=None, human_data=None):
        now = stamp if stamp is not None else self.get_clock().now().to_msg()

        try:
            q_array = np.asarray(q_values, dtype=float).flatten()
        except Exception as exc:
            self._warn_throttled(f'Skipping q publish, invalid q payload type: {exc}')
            q_array = np.array([], dtype=float)
        q_is_valid = q_array.size > 0 and np.all(np.isfinite(q_array))
        if q_array.size > 0:
            if q_is_valid:
                q_msg = Float64MultiArray()
                q_msg.data = [float(v) for v in q_array]
                self.q_publisher_.publish(q_msg)

                if self.joint_names:
                    q_joints = self._extract_joint_positions(q_array)
                    joint_count = min(len(self.joint_names), len(q_joints))
                    js_msg = JointState()
                    js_msg.header = Header(stamp=now, frame_id=self.world_frame_id)
                    js_msg.name = self.joint_names[:joint_count]
                    js_msg.position = q_joints[:joint_count]
                    self.joint_state_publisher_.publish(js_msg)

                if self.publish_base_tf:
                    self._publish_base_transform(q_array, now)
            else:
                self._warn_throttled('Skipping non-finite q sample.')

        if self.enable_markers:
            marker_array = MarkerArray()
            for idx, (name, xyz) in enumerate(markers.items()):
                if not np.all(np.isfinite(xyz)):
                    continue
                marker = Marker()
                marker.header = Header(stamp=now, frame_id=self.world_frame_id)
                marker.ns = 'rtcosmik_markers'
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(xyz[0])
                marker.pose.position.y = float(xyz[1])
                marker.pose.position.z = float(xyz[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.r = 1.0
                marker.color.g = 0.2
                marker.color.b = 0.2
                marker.color.a = 1.0
                marker.text = name
                marker_array.markers.append(marker)
            self.marker_publisher_.publish(marker_array)

        if q_is_valid:
            self._publish_collision_capsules(human_model, human_data, q_array, now)

    def _extract_joint_positions(self, q_array):
        """
        Map full configuration to articulated joint values expected by JointState.
        If a free-flyer is present, RT-COSMIK q layout is [x y z qx qy qz qw joints...].
        """
        if not self.joint_names:
            return []

        expected = len(self.joint_names)
        if self._has_freeflyer_model and q_array.size >= expected + 7:
            return [float(v) for v in q_array[7:7 + expected]]
        return [float(v) for v in q_array[:expected]]

    def _publish_base_transform(self, q_array, stamp):
        if self.tf_broadcaster_ is None:
            return
        if not self._has_freeflyer_model:
            return
        if q_array.size < 7:
            return

        quat = np.asarray(q_array[3:7], dtype=float)
        if not np.all(np.isfinite(quat)) or not np.all(np.isfinite(q_array[:3])):
            self._warn_throttled('Skipping non-finite base transform sample.')
            return
        quat_norm = np.linalg.norm(quat)
        if quat_norm < 1e-12:
            return
        quat /= quat_norm

        # Apply a fixed freeflyer correction so RViz axes follow project convention.
        t_current = np.asarray(q_array[:3], dtype=float).reshape(3)
        q_current = pin.Quaternion(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2]))
        t_current_se3 = pin.SE3(q_current.matrix(), t_current)
        t_correction = pin.SE3(self._ff_rotation_correction, np.zeros(3))
        t_corrected = t_correction * t_current_se3
        q_corrected = pin.Quaternion(t_corrected.rotation)

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.transform.translation.x = float(t_corrected.translation[0])
        msg.transform.translation.y = float(t_corrected.translation[1])
        msg.transform.translation.z = float(t_corrected.translation[2])
        msg.transform.rotation.x = float(q_corrected.x)
        msg.transform.rotation.y = float(q_corrected.y)
        msg.transform.rotation.z = float(q_corrected.z)
        msg.transform.rotation.w = float(q_corrected.w)
        self.tf_broadcaster_.sendTransform(msg)

    def destroy_node(self):
        recorder = getattr(self, '_recorder', None)
        if recorder is not None:
            recorder.close()
            self._recorder = None
        if self.stop_event is not None:
            self.stop_event.set()

        if self._runtime_thread is not None and self._runtime_thread.is_alive():
            self._runtime_thread.join(timeout=2.0)

        for process in self.processes:
            process.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTCosmikMarkerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
