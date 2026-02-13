#!/usr/bin/env python3
"""ROS 2 bridge: publish RT-COSMIK outputs for visualization and control."""

import os
import threading
from pathlib import Path
from queue import Empty
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64MultiArray, Header
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

from rtcosmik.config_loader import settings
from rtcosmik.camera.cam_utils import list_cameras, load_camera_parameters, load_world_transformation
from rtcosmik.camera.camera import Camera
from rtcosmik.pipeline.pipeline import PipelineProcess
from rtcosmik.utils.mp_utils import create_camera_shared_ressources, create_pipeline_shared_ressources


class RTCosmikMarkerBridge(Node):
    def __init__(self):
        super().__init__('rtcosmik_marker_bridge')

        self.declare_parameter('enable_markers', True)
        self.declare_parameter('queue_timeout_s', 0.03)
        self.declare_parameter('joint_names', [])

        self.enable_markers = bool(self.get_parameter('enable_markers').value)
        self.queue_timeout_s = float(self.get_parameter('queue_timeout_s').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        if not self.joint_names:
            self.joint_names = self._load_joint_names_from_urdf()

        self.pose_publisher_ = self.create_publisher(PoseArray, '/rtcosmik/body_poses', qos_profile_sensor_data)
        self.q_publisher_ = self.create_publisher(Float64MultiArray, '/rtcosmik/q', qos_profile_sensor_data)
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            '/rtcosmik/joint_states',
            qos_profile_sensor_data,
        )
        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/rtcosmik/markers',
            qos_profile_sensor_data,
        )

        self.stop_event = None
        self.processes = []
        self.result_queues = []
        self._consumer_thread = None
        self._start_rtcosmik_processes()

        self._consumer_thread = threading.Thread(target=self._consume_and_publish_loop, daemon=True)
        self._consumer_thread.start()

        self.get_logger().info(
            f'RT-COSMIK bridge started. Publishing q ({len(self.joint_names)} joints), '
            '/rtcosmik/body_poses and '
            + ('/rtcosmik/markers.' if self.enable_markers else 'markers disabled.')
        )

    def _load_joint_names_from_urdf(self):
        try:
            share_dir = Path(get_package_share_directory('rtcosmik_ros'))
            urdf_path = share_dir / 'urdf' / 'human.urdf'
        except Exception:
            urdf_path = Path(__file__).resolve().parents[1] / 'urdf' / 'human.urdf'

        if not urdf_path.exists():
            self.get_logger().warning('Could not find human.urdf to auto-load joint names.')
            return []

        root = ET.fromstring(urdf_path.read_text())
        names = []
        for joint in root.findall('joint'):
            joint_type = joint.attrib.get('type', '')
            if joint_type in {'revolute', 'continuous', 'prismatic'}:
                names.append(joint.attrib['name'])
        return names

    def _start_rtcosmik_processes(self):
        width, height = settings.width, settings.height
        frame_shape = (height, width, 3)

        cam_calib_path = os.getenv('RTCOSMIK_CAM_CALIB_PATH', settings.cam_calib_path)
        if cam_calib_path != settings.cam_calib_path:
            self.get_logger().info(
                f'Overriding RT-COSMIK camera calibration path from environment: {cam_calib_path}'
            )
            settings.cam_calib_path = cam_calib_path

        mtxs, dists, projections, _, _ = load_camera_parameters(cam_calib_path)
        world_r1_cam, world_t1_cam = load_world_transformation(cam_calib_path)

        cameras = list_cameras()
        num_cameras = len(cameras)
        if num_cameras < 2:
            raise RuntimeError('At least 2 cameras are required for triangulation.')

        (
            camera_buffers,
            camera_timestamps,
            camera_locks,
            frame_counters,
            camera_barrier,
            self.stop_event,
        ) = create_camera_shared_ressources(num_cameras, frame_shape)

        self.result_queues = create_pipeline_shared_ressources()

        cam_ids = list(cameras.keys())
        camera_processes = [
            Camera(
                cam_id=cam_ids[i],
                shared_buffer=camera_buffers[i],
                timestamp_buffer=camera_timestamps[i],
                lock=camera_locks[i],
                frame_counter=frame_counters[i],
            )
            for i in range(num_cameras)
        ]

        pipeline_process = PipelineProcess(
            settings=settings,
            frame_counters=frame_counters,
            camera_buffers=camera_buffers,
            camera_locks=camera_locks,
            timestamp_buffers=camera_timestamps,
            results_queues=self.result_queues,
            stop_event=self.stop_event,
            mtxs=mtxs,
            dists=dists,
            projections=projections,
            world_R1_cam=world_r1_cam,
            world_T1_cam=world_t1_cam,
            frame_shape=frame_shape,
            num_cameras=num_cameras,
        )

        self.processes = [*camera_processes, pipeline_process]
        for process in self.processes:
            process.start()

    def _consume_and_publish_loop(self):
        if not self.result_queues:
            return

        queue = self.result_queues[0]
        while self.stop_event is not None and not self.stop_event.is_set():
            try:
                item = queue.get(timeout=self.queue_timeout_s)
            except Empty:
                continue

            while True:
                try:
                    item = queue.get_nowait()
                except Empty:
                    break

            markers, q_values, body_poses = self._extract_outputs(item)
            self._publish_outputs(markers, q_values, body_poses)

    def _extract_outputs(self, item):
        markers = {}
        q_values = []
        body_poses = {}

        if isinstance(item, dict):
            markers = item.get('markers') or item.get('mks_dict') or {}
            q_values = item.get('q') or []
            body_poses = item.get('body_poses') or item.get('segment_poses') or {}
        elif isinstance(item, (tuple, list)):
            for elem in item:
                if isinstance(elem, dict):
                    if not markers and elem and all(isinstance(v, (tuple, list)) and len(v) >= 3 for v in elem.values()):
                        markers = elem
                    elif not body_poses:
                        body_poses = elem
                elif isinstance(elem, (list, tuple)) and elem and isinstance(elem[0], (int, float)):
                    q_values = elem
            if not markers and len(item) >= 2 and isinstance(item[1], dict):
                markers = item[1]
        return markers, list(q_values), body_poses

    def _publish_outputs(self, markers, q_values, body_poses):
        now = self.get_clock().now().to_msg()

        if q_values:
            q_msg = Float64MultiArray()
            q_msg.data = [float(v) for v in q_values]
            self.q_publisher_.publish(q_msg)

            if self.joint_names:
                joint_count = min(len(self.joint_names), len(q_msg.data))
                js_msg = JointState()
                js_msg.header = Header(stamp=now, frame_id='world')
                js_msg.name = self.joint_names[:joint_count]
                js_msg.position = q_msg.data[:joint_count]
                self.joint_state_publisher_.publish(js_msg)

        pose_array = PoseArray()
        pose_array.header = Header(stamp=now, frame_id='world')

        if body_poses:
            for _, pose_data in body_poses.items():
                pose = Pose()
                if isinstance(pose_data, dict):
                    pos = pose_data.get('position', [0.0, 0.0, 0.0])
                    ori = pose_data.get('orientation', [0.0, 0.0, 0.0, 1.0])
                else:
                    pos = pose_data[:3] if len(pose_data) >= 3 else [0.0, 0.0, 0.0]
                    ori = pose_data[3:7] if len(pose_data) >= 7 else [0.0, 0.0, 0.0, 1.0]
                pose.position.x, pose.position.y, pose.position.z = map(float, pos[:3])
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, ori[:4])
                pose_array.poses.append(pose)
        else:
            for xyz in markers.values():
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
                marker = Marker()
                marker.header = Header(stamp=now, frame_id='world')
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

    def destroy_node(self):
        if self.stop_event is not None:
            self.stop_event.set()

        if self._consumer_thread is not None and self._consumer_thread.is_alive():
            self._consumer_thread.join(timeout=1.0)

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
