#!/usr/bin/env python3
"""ROS 2 bridge: publish RT-COSMIK outputs for visualization and control."""

from collections import deque
from datetime import datetime
import os
import shutil
import threading
import time
import traceback
from pathlib import Path
import xml.etree.ElementTree as ET

import example_robot_data as robex
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

from ament_index_python.packages import get_package_share_directory

from rtcosmik.camera.camera import Camera
from rtcosmik.camera.cam_utils import (
    list_cameras,
    load_camera_parameters,
)
from rtcosmik.config_loader import settings
from rtcosmik.filtering.iir import IIR
from rtcosmik.human_model.model_utils import (
    mks_registration,
    recalibrate_marker_frames_in_joint_space,
    scale_human_model,
)
from rtcosmik.ik.ik import RT_IK, RT_SWIKA
from rtcosmik.nlf.nlf import NLFEstimator
from rtcosmik.triangulation.triangulation import triangulate_points
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

        self.pose_publisher_ = self.create_publisher(PoseArray, '/rtcosmik/body_poses', self._reliable_qos)
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
        self._max_frame_skew_s = 0.050
        self._last_skew_warn_t = 0.0
        self._has_freeflyer_model = False
        self._last_invalid_warn_t = 0.0

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

        cam_calib_path = os.getenv('RTCOSMIK_CAM_CALIB_PATH', settings.cam_calib_path)
        if cam_calib_path != settings.cam_calib_path:
            self.get_logger().info(
                f'Overriding RT-COSMIK camera calibration path from environment: {cam_calib_path}'
            )
            settings.cam_calib_path = cam_calib_path

        self._mtxs, self._dists, self._projections, _, _ = load_camera_parameters(cam_calib_path)

        cameras = list_cameras()
        self._num_cameras = len(cameras)
        if self._num_cameras < 2:
            raise RuntimeError('At least 2 cameras are required for triangulation.')

        (
            self._camera_buffers,
            self._camera_timestamps,
            self._camera_locks,
            self._frame_counters,
            camera_barrier,
            self.stop_event,
        ) = create_camera_shared_ressources(self._num_cameras, self._frame_shape)

        cam_ids = list(cameras.keys())
        camera_processes = [
            Camera(
                cam_id=cam_ids[i],
                shared_buffer=self._camera_buffers[i],
                timestamp_buffer=self._camera_timestamps[i],
                lock=self._camera_locks[i],
                frame_counter=self._frame_counters[i],
                barrier=camera_barrier,
                stop_event=self.stop_event,
                frame_shape=self._frame_shape,
                cam_fps=settings.fs,
                cam_fourcc=settings.fourcc,
            )
            for i in range(self._num_cameras)
        ]

        self._last_frame_counters = [0] * self._num_cameras
        self.processes = camera_processes
        for process in self.processes:
            process.start()

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

        if not self._timestamps_are_coherent(timestamps):
            return None

        self._last_frame_counters = new_counters
        return frames

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

    def _compute_body_poses(self, model, data, q):
        if model is None or data is None:
            return {}
        q_array = np.asarray(q, dtype=float).flatten()
        if q_array.size < model.nq or not np.all(np.isfinite(q_array[:model.nq])):
            return {}

        # Ensure freeflyer quaternion stays normalized before kinematics.
        q_model = np.array(q_array[:model.nq], dtype=float, copy=True)
        try:
            q_model = pin.normalize(model, q_model)
            pin.forwardKinematics(model, data, q_model)
            pin.updateFramePlacements(model, data)
        except Exception as exc:
            self._warn_throttled(f'Body pose computation skipped for invalid model state: {exc}')
            return {}

        poses = {}
        for frame_id, frame in enumerate(model.frames):
            if frame.type != pin.FrameType.BODY:
                continue
            if "virtual" in frame.name:
                continue
            oMf = data.oMf[frame_id]
            xyz = np.asarray(oMf.translation, dtype=float).reshape(3)
            if not np.all(np.isfinite(xyz)):
                continue
            quat = pin.Quaternion(oMf.rotation)
            poses[frame.name] = {
                "position": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                "orientation": [float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
            }
        return poses

    def _processing_loop(self):
        first_sample = True
        p3d_buffer = deque(maxlen=settings.N)
        ik_class = None
        human_model = None
        human_data = None
        human_visual_model = None
        human_collision_model = None
        deque_lstm_dict = None
        x_array = None
        u_array = None
        last_health_check_t = 0.0

        try:
            estimator = NLFEstimator(
                yolo_path=settings.yolo_path,
                nlf_path=settings.nlf_path,
                cano_path=settings.cano_path,
                image_size=(self._frame_shape[1], self._frame_shape[0]),
                cam_Ks=self._mtxs,
                indices=settings.nlf_indices,
                conf=settings.yolo_conf,
                imgsz=settings.yolo_imgsz,
                device=settings.device,
            )

            num_channel = 3 * len(settings.marker_names)
            iir_filter = IIR(num_channel=num_channel, sampling_frequency=settings.fs)
            iir_filter.add_filter(
                order=settings.order,
                cutoff=settings.cutoff_freq,
                filter_type=settings.filter_type,
            )

            while self.stop_event is not None and not self.stop_event.is_set():
                now_t = time.monotonic()
                if now_t - last_health_check_t > 1.0:
                    self._assert_camera_processes_alive()
                    last_health_check_t = now_t

                frames = self._read_synchronized_frames()
                if frames is None:
                    time.sleep(0.001)
                    continue

                nlf_out, _, _, _ = estimator.estimate_from_frames(frames)
                nlf_out_2d = nlf_out["poses2d"]
                if nlf_out_2d is None or len(nlf_out_2d) < self._num_cameras:
                    continue

                keypoints_list = [None] * self._num_cameras
                valid_cam_ids = []
                for ii in range(self._num_cameras):
                    poses2d = nlf_out_2d[ii]
                    if poses2d is None or len(poses2d) == 0 or poses2d[0] is None:
                        continue
                    keypoints_list[ii] = poses2d[0].detach().float().cpu().numpy()
                    valid_cam_ids.append(ii)

                if len(valid_cam_ids) < 2:
                    continue

                p3d = triangulate_points(
                    keypoints_list=keypoints_list,
                    mtxs=self._mtxs,
                    dists=self._dists,
                    projections=self._projections,
                )
                # Triangulated markers are already in the calibrated frame.
                p3d_in_world = np.asarray(p3d, dtype=np.float32)

                if first_sample:
                    for _ in range(settings.N):
                        p3d_buffer.append(p3d_in_world)
                else:
                    p3d_buffer.append(p3d_in_world)

                if len(p3d_buffer) != settings.N:
                    continue

                p3d_buffer_array = np.array(p3d_buffer)
                filtered_p3d_buffer = iir_filter.filter(
                    np.reshape(p3d_buffer_array, (settings.N, 3 * len(settings.marker_names)))
                )
                filtered_p3d_buffer = np.reshape(
                    filtered_p3d_buffer, (settings.N, len(settings.marker_names), 3)
                )
                augmented_markers = filtered_p3d_buffer[-1]
                mks_dict = dict(zip(settings.marker_names, augmented_markers))

                if first_sample:
                    human = robex.human.HumanLoader(
                        height=settings.human_height,
                        weight=settings.human_weight,
                        gender=settings.human_gender,
                    ).robot
                    human_model = human.model
                    human_collision_model = human.collision_model
                    human_visual_model = human.visual_model
                    self._has_freeflyer_model = self._model_has_freeflyer(human_model)

                    human_model = scale_human_model(
                        human_model,
                        mks_dict,
                        gender=settings.human_gender,
                        subject_height=settings.human_height,
                    )
                    human_model = mks_registration(
                        human_model,
                        mks_dict,
                        gender=settings.human_gender,
                        subject_height=settings.human_height,
                    )

                    if settings.ik_type == 'sbs':
                        omega = {key: 1 for key in settings.keys_to_track_list}
                        q = pin.neutral(human_model)
                        ik_class = RT_IK(
                            human_model,
                            mks_dict,
                            q,
                            settings.keys_to_track_list,
                            settings.dt,
                            omega,
                        )
                        q = ik_class.solve_ik_sample_casadi()
                        ik_class._q0 = q

                        human_model = recalibrate_marker_frames_in_joint_space(
                            human_model,
                            q,
                            mks_dict,
                            settings.marker_names,
                        )
                        ik_class = RT_IK(
                            human_model,
                            mks_dict,
                            q,
                            settings.keys_to_track_list,
                            settings.dt,
                            omega,
                        )

                    elif settings.ik_type == 'mhe':
                        ik_class = RT_SWIKA(
                            human_model,
                            settings.keys_to_track_list,
                            settings.N,
                            code=settings.ik_code,
                        )
                        x_array = np.zeros((human_model.nq + human_model.nv, settings.N))
                        x_array[6, :] = 1
                        u_array = np.zeros((human_model.nv, settings.N))
                        deque_lstm_dict = deque(maxlen=settings.N)
                        for _ in range(settings.N):
                            deque_lstm_dict.append(mks_dict)

                        array_data = np.array(
                            [
                                np.hstack([d[marker] for marker in settings.keys_to_track_list])
                                for d in deque_lstm_dict
                            ]
                        ).T
                        x_array, u_array = ik_class.solve(
                            x_array,
                            u_array,
                            array_data,
                            x_array[:, -1],
                            settings.cost_weights,
                            settings.dt,
                        )
                        q = pin.neutral(human_model)
                        q[:] = np.array(x_array[:human_model.nq, -1]).flatten()

                        human_model = recalibrate_marker_frames_in_joint_space(
                            human_model,
                            q,
                            mks_dict,
                            settings.marker_names,
                        )
                        ik_class = RT_SWIKA(
                            human_model,
                            settings.keys_to_track_list,
                            settings.N,
                            code=settings.ik_code,
                        )
                    else:
                        raise ValueError(
                            "Invalid ik type, should be sbs (sample by sample) or mhe (moving horizon estimation)."
                        )

                    self._write_scaled_urdf_from_pin_model(
                        human_model,
                        self.scaled_urdf_output_path,
                        human_visual_model=human_visual_model,
                        human_collision_model=human_collision_model,
                    )
                    human_data = human_model.createData()
                    self._announce_initialization_done()
                    first_sample = False
                    continue

                if settings.ik_type == 'sbs':
                    ik_class._dict_m = mks_dict
                    q = ik_class.solve_ik_sample_quadprog()
                    ik_class._q0 = q
                elif settings.ik_type == 'mhe':
                    deque_lstm_dict.append(mks_dict)
                    array_data = np.array(
                        [
                            np.hstack([d[marker] for marker in settings.keys_to_track_list])
                            for d in deque_lstm_dict
                        ]
                    ).T
                    x_array, u_array = ik_class.solve(
                        x_array,
                        u_array,
                        array_data,
                        x_array[:, -1],
                        settings.cost_weights,
                        settings.dt,
                    )
                    q = pin.neutral(human_model)
                    q[:] = np.array(x_array[:human_model.nq, -1]).flatten()
                else:
                    raise ValueError(
                        "Invalid ik type, should be sbs (sample by sample) or mhe (moving horizon estimation)."
                    )

                body_poses = self._compute_body_poses(human_model, human_data, q)
                self._publish_outputs(mks_dict, q, body_poses=body_poses)

        except Exception as exc:
            if self.stop_event is not None:
                self.stop_event.set()
            self.get_logger().error(
                f'RT-COSMIK runtime loop crashed: {exc}\n{traceback.format_exc()}'
            )
        finally:
            self.get_logger().info('RT-COSMIK runtime loop terminated.')

    def _publish_outputs(self, markers, q_values, body_poses=None):
        now = self.get_clock().now().to_msg()

        try:
            q_array = np.asarray(q_values, dtype=float).flatten()
        except Exception as exc:
            self._warn_throttled(f'Skipping q publish, invalid q payload type: {exc}')
            q_array = np.array([], dtype=float)
        if q_array.size > 0:
            if np.all(np.isfinite(q_array)):
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

        pose_array = PoseArray()
        pose_array.header = Header(stamp=now, frame_id=self.world_frame_id)
        if body_poses:
            for pose_data in body_poses.values():
                pos = pose_data["position"]
                ori = pose_data["orientation"]
                if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(ori)):
                    continue
                pose = Pose()
                pose.position.x = float(pos[0])
                pose.position.y = float(pos[1])
                pose.position.z = float(pos[2])
                pose.orientation.x = float(ori[0])
                pose.orientation.y = float(ori[1])
                pose.orientation.z = float(ori[2])
                pose.orientation.w = float(ori[3])
                pose_array.poses.append(pose)
        else:
            for xyz in markers.values():
                if not np.all(np.isfinite(xyz)):
                    continue
                pose = Pose()
                pose.position.x = float(xyz[0])
                pose.position.y = float(xyz[1])
                pose.position.z = float(xyz[2])
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        self.pose_publisher_.publish(pose_array)

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

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.transform.translation.x = float(q_array[0])
        msg.transform.translation.y = float(q_array[1])
        msg.transform.translation.z = float(q_array[2])
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster_.sendTransform(msg)

    def destroy_node(self):
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
