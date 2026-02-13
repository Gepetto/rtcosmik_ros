#!/usr/bin/env python3
"""ROS 2 bridge: publish RT-COSMIK markers."""

import os
from queue import Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from rtcosmik.config_loader import settings
from rtcosmik.camera.cam_utils import list_cameras, load_camera_parameters, load_world_transformation
from rtcosmik.camera.camera import Camera
from rtcosmik.pipeline.pipeline import PipelineProcess
from rtcosmik.utils.mp_utils import create_camera_shared_ressources, create_pipeline_shared_ressources


class RTCosmikMarkerBridge(Node):
    def __init__(self):
        super().__init__('rtcosmik_marker_bridge')
        self.publisher_ = self.create_publisher(
            MarkerArray,
            '/rtcosmik/markers',
            qos_profile_sensor_data,
        )

        # Start RT-COSMIK producer stack (camera + pipeline) and consume queue[0] markers.
        self.stop_event = None
        self.processes = []
        self.result_queues = []
        self._start_rtcosmik_processes()

        self.timer = self.create_timer(0.02, self._poll_and_publish_markers)  # ~50 Hz polling
        self.get_logger().info('RT-COSMIK marker bridge started. Publishing /rtcosmik/markers.')

    def _start_rtcosmik_processes(self):
        width, height = settings.width, settings.height
        frame_shape = (height, width, 3)

        cam_calib_path = os.getenv('RTCOSMIK_CAM_CALIB_PATH', settings.cam_calib_path)
        if cam_calib_path != settings.cam_calib_path:
            self.get_logger().info(
                f'Overriding RT-COSMIK camera calibration path from environment: {cam_calib_path}'
            )
            # Keep settings object coherent for downstream RT-COSMIK code.
            settings.cam_calib_path = cam_calib_path
        else:
            self.get_logger().info(f'Using RT-COSMIK camera calibration path: {cam_calib_path}')

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
                barrier=camera_barrier,
                stop_event=self.stop_event,
                frame_shape=frame_shape,
                cam_fps=settings.fs,
                cam_fourcc=settings.fourcc,
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

        self.get_logger().info(f'Started {len(camera_processes)} camera processes and 1 pipeline process.')

    def _poll_and_publish_markers(self):
        if not self.result_queues:
            return

        try:
            _, mks_dict = self.result_queues[0].get_nowait()
        except Empty:
            return

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for idx, (name, xyz) in enumerate(mks_dict.items()):
            marker = Marker()
            marker.header = Header(stamp=now, frame_id='world')
            marker.ns = 'rtcosmik_markers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            point = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
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

        self.publisher_.publish(marker_array)

    def destroy_node(self):
        self.get_logger().info('Shutting down RT-COSMIK processes...')
        if self.stop_event is not None:
            self.stop_event.set()
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
