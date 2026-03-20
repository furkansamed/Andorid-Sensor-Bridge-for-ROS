from __future__ import annotations

import queue
import socket
import struct
import threading
import time
from copy import deepcopy
from dataclasses import dataclass

import av
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu


VIDEO_HEADER_STRUCT = struct.Struct(">qqfffffffffffffI")
IMU_STRUCT = struct.Struct(">qfffffffffffff")
MAX_VIDEO_PAYLOAD_BYTES = 2 * 1024 * 1024


@dataclass
class ImuSample:
    timestamp_ns: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    qx: float
    qy: float
    qz: float
    qw: float
    pitch_rad: float
    yaw_rad: float
    roll_rad: float


@dataclass
class VideoPacketHeader:
    frame_timestamp_ns: int
    synced_sample: ImuSample
    payload_size: int


@dataclass
class DecodedFrame:
    timestamp_ns: int
    width: int
    height: int
    encoding: str
    step: int
    data: bytes
    synced_imu: ImuSample | None


@dataclass
class PublishDiagnosticsWindow:
    image_published: int = 0
    camera_info_published: int = 0
    imu_published: int = 0
    synced_imu_published: int = 0
    video_frames_coalesced: int = 0
    video_queue_drops: int = 0
    imu_queue_drops: int = 0
    max_video_batch: int = 0
    max_imu_batch: int = 0


class ClockMapper:
    def __init__(self, now_provider) -> None:
        self._now_provider = now_provider
        self._offset_ns: int | None = None
        self._lock = threading.Lock()

    def to_ros_time(self, phone_timestamp_ns: int) -> Time:
        if phone_timestamp_ns <= 0:
            return Time(nanoseconds=self._now_provider())
        with self._lock:
            if self._offset_ns is None:
                self._offset_ns = self._now_provider() - phone_timestamp_ns
            mapped_ns = phone_timestamp_ns + self._offset_ns
        return Time(nanoseconds=max(mapped_ns, 0))


class VioStreamBridge(Node):
    def __init__(self) -> None:
        super().__init__("vio_stream_bridge")

        self._declare_parameters()
        self.bind_address = self.get_parameter("bind_address").get_parameter_value().string_value
        self.video_port = self.get_parameter("video_port").get_parameter_value().integer_value
        self.imu_port = self.get_parameter("imu_port").get_parameter_value().integer_value
        self.camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value
        self.imu_frame_id = self.get_parameter("imu_frame_id").get_parameter_value().string_value
        self.publish_camera_info = (
            self.get_parameter("publish_camera_info").get_parameter_value().bool_value
        )
        self.diagnostics_enabled = (
            self.get_parameter("diagnostics_enabled").get_parameter_value().bool_value
        )
        self.diagnostics_period_sec = (
            self.get_parameter("diagnostics_period_sec").get_parameter_value().double_value
        )
        self.publish_period_sec = max(
            0.001,
            self.get_parameter("publish_period_sec").get_parameter_value().double_value,
        )
        self.max_video_messages_per_cycle = max(
            1,
            self.get_parameter("max_video_messages_per_cycle").get_parameter_value().integer_value,
        )
        self.max_imu_messages_per_cycle = max(
            1,
            self.get_parameter("max_imu_messages_per_cycle").get_parameter_value().integer_value,
        )
        self.orientation_covariance = self._get_double_list("orientation_covariance")
        self.angular_velocity_covariance = self._get_double_list("angular_velocity_covariance")
        self.linear_acceleration_covariance = self._get_double_list("linear_acceleration_covariance")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_publisher = self.create_publisher(Image, "camera/image_raw", qos)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "camera/camera_info", qos)
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", qos)
        self.synced_imu_publisher = self.create_publisher(Imu, "imu/image_sync", qos)

        self.video_queue: queue.Queue[DecodedFrame] = queue.Queue(maxsize=4)
        self.imu_queue: queue.Queue[ImuSample] = queue.Queue(maxsize=1024)
        self.stop_event = threading.Event()
        self._diagnostics_lock = threading.Lock()
        self._diagnostics_window = PublishDiagnosticsWindow()
        self._diagnostics_window_started_at = time.monotonic()
        self.clock_mapper = ClockMapper(lambda: self.get_clock().now().nanoseconds)
        self.base_camera_info = self._build_base_camera_info()
        self.video_listener: socket.socket | None = None
        self.imu_socket: socket.socket | None = None
        self.video_thread = threading.Thread(target=self._video_server_loop, name="ros-video", daemon=True)
        self.imu_thread = threading.Thread(target=self._imu_server_loop, name="ros-imu", daemon=True)
        self.video_thread.start()
        self.imu_thread.start()
        self.publish_timer = self.create_timer(
            self.publish_period_sec,
            self._publish_pending_messages,
        )
        self.diagnostics_timer = None
        if self.diagnostics_enabled:
            self.diagnostics_timer = self.create_timer(
                max(self.diagnostics_period_sec, 0.1),
                self._log_publish_diagnostics,
            )

        self.get_logger().info(
            f"Listening for Android streamer on {self.bind_address}:{self.video_port} (TCP) "
            f"and {self.bind_address}:{self.imu_port} (UDP)"
        )
        if self.diagnostics_enabled:
            self.get_logger().info(
                f"Publish diagnostics enabled with a {max(self.diagnostics_period_sec, 0.1):.2f}s window"
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("video_port", 5000)
        self.declare_parameter("imu_port", 5001)
        self.declare_parameter("camera_frame_id", "camera_optical_frame")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("publish_camera_info", False)
        self.declare_parameter("diagnostics_enabled", False)
        self.declare_parameter("diagnostics_period_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.002)
        self.declare_parameter("max_video_messages_per_cycle", 1)
        self.declare_parameter("max_imu_messages_per_cycle", 1)
        self.declare_parameter("camera_name", "phone_camera")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("distortion_model", "plumb_bob")
        self.declare_parameter(
            "distortion_coefficients",
            [0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "camera_matrix",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "rectification_matrix",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "projection_matrix",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "orientation_covariance",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "angular_velocity_covariance",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "linear_acceleration_covariance",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def _build_base_camera_info(self) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id
        camera_info.width = self.get_parameter("image_width").get_parameter_value().integer_value
        camera_info.height = self.get_parameter("image_height").get_parameter_value().integer_value
        camera_info.distortion_model = (
            self.get_parameter("distortion_model").get_parameter_value().string_value
        )
        camera_info.d = self._get_double_list("distortion_coefficients")
        camera_info.k = self._get_double_list("camera_matrix")
        camera_info.r = self._get_double_list("rectification_matrix")
        camera_info.p = self._get_double_list("projection_matrix")
        return camera_info

    def _get_double_list(self, name: str) -> list[float]:
        return [float(value) for value in self.get_parameter(name).value]

    def _video_server_loop(self) -> None:
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.bind_address, self.video_port))
            listener.listen(1)
            listener.settimeout(1.0)
            self.video_listener = listener
        except OSError as error:
            self.get_logger().error(f"Failed to bind TCP video port {self.video_port}: {error}")
            self.stop_event.set()
            return

        while not self.stop_event.is_set():
            try:
                client, address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f"Video client connected from {address[0]}:{address[1]}")
            decoder = av.CodecContext.create("h264", "r")
            last_header: VideoPacketHeader | None = None
            try:
                with client:
                    client.settimeout(1.0)
                    while not self.stop_event.is_set():
                        raw_header = self._recv_exact(client, VIDEO_HEADER_STRUCT.size)
                        if raw_header is None:
                            break
                        header = self._parse_video_header(raw_header)
                        if header.payload_size <= 0 or header.payload_size > MAX_VIDEO_PAYLOAD_BYTES:
                            self.get_logger().warning(
                                f"Ignoring suspicious video payload size {header.payload_size}"
                            )
                            break
                        payload = self._recv_exact(client, header.payload_size)
                        if payload is None:
                            break
                        if header.frame_timestamp_ns > 0:
                            last_header = header
                        for packet in decoder.parse(payload):
                            for frame in decoder.decode(packet):
                                effective_header = last_header or header
                                pixel_data = frame.to_ndarray(format="bgr24").tobytes()
                                self._put_latest(
                                    self.video_queue,
                                    DecodedFrame(
                                        timestamp_ns=effective_header.frame_timestamp_ns,
                                        width=frame.width,
                                        height=frame.height,
                                        encoding="bgr8",
                                        step=frame.width * 3,
                                        data=pixel_data,
                                        synced_imu=self._valid_synced_sample(
                                            effective_header.synced_sample
                                        ),
                                    ),
                                    queue_name="video",
                                )
            except OSError as error:
                self.get_logger().warning(f"Video connection dropped: {error}")
            except av.AVError as error:
                self.get_logger().warning(f"H.264 decoder error: {error}")
            finally:
                self.get_logger().info("Video client disconnected")

    def _imu_server_loop(self) -> None:
        try:
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_socket.bind((self.bind_address, self.imu_port))
            udp_socket.settimeout(1.0)
            self.imu_socket = udp_socket
        except OSError as error:
            self.get_logger().error(f"Failed to bind UDP IMU port {self.imu_port}: {error}")
            self.stop_event.set()
            return

        logged_first_sender = False
        while not self.stop_event.is_set():
            try:
                payload, address = udp_socket.recvfrom(IMU_STRUCT.size)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(payload) != IMU_STRUCT.size:
                self.get_logger().warning(f"Ignoring IMU packet of size {len(payload)}")
                continue
            if not logged_first_sender:
                self.get_logger().info(f"IMU packets arriving from {address[0]}:{address[1]}")
                logged_first_sender = True

            self._put_latest(
                self.imu_queue,
                ImuSample(*IMU_STRUCT.unpack(payload)),
                queue_name="imu",
            )

    def _publish_pending_messages(self) -> None:
        imu_samples = self._drain_queue_snapshot(
            self.imu_queue,
            self.max_imu_messages_per_cycle,
        )
        imu_batch_count = len(imu_samples)
        for sample in imu_samples:
            self._publish_imu(sample, self.imu_publisher)

        video_frames, video_drained_count = self._drain_latest_from_queue(
            self.video_queue,
            self.max_video_messages_per_cycle,
        )
        if video_frames:
            for frame in video_frames:
                self._publish_image(frame)

        self._record_publish_batches(video_drained_count, imu_batch_count)

    def _publish_image(self, frame: DecodedFrame) -> None:
        stamp = self.clock_mapper.to_ros_time(frame.timestamp_ns).to_msg()
        image_message = Image()
        image_message.header.stamp = stamp
        image_message.header.frame_id = self.camera_frame_id
        image_message.height = frame.height
        image_message.width = frame.width
        image_message.encoding = frame.encoding
        image_message.is_bigendian = False
        image_message.step = frame.step
        image_message.data = frame.data
        self.image_publisher.publish(image_message)
        self._record_publish("image")

        if self.publish_camera_info:
            camera_info = deepcopy(self.base_camera_info)
            camera_info.header.stamp = stamp
            camera_info.header.frame_id = self.camera_frame_id
            camera_info.width = frame.width
            camera_info.height = frame.height
            self.camera_info_publisher.publish(camera_info)
            self._record_publish("camera_info")

        if frame.synced_imu is not None:
            self._publish_imu(
                frame.synced_imu,
                self.synced_imu_publisher,
                stamp_override=stamp,
                diagnostics_key="synced_imu",
            )

    def _publish_imu(
        self,
        sample: ImuSample,
        publisher,
        stamp_override=None,
        diagnostics_key: str = "imu",
    ) -> None:
        stamp = stamp_override or self.clock_mapper.to_ros_time(sample.timestamp_ns).to_msg()
        imu_message = Imu()
        imu_message.header.stamp = stamp
        imu_message.header.frame_id = self.imu_frame_id
        imu_message.orientation.x = sample.qx
        imu_message.orientation.y = sample.qy
        imu_message.orientation.z = sample.qz
        imu_message.orientation.w = sample.qw
        imu_message.linear_acceleration.x = sample.ax
        imu_message.linear_acceleration.y = sample.ay
        imu_message.linear_acceleration.z = sample.az
        imu_message.angular_velocity.x = sample.gx
        imu_message.angular_velocity.y = sample.gy
        imu_message.angular_velocity.z = sample.gz
        imu_message.orientation_covariance = self.orientation_covariance
        imu_message.angular_velocity_covariance = self.angular_velocity_covariance
        imu_message.linear_acceleration_covariance = self.linear_acceleration_covariance
        publisher.publish(imu_message)
        self._record_publish(diagnostics_key)

    def _parse_video_header(self, payload: bytes) -> VideoPacketHeader:
        unpacked = VIDEO_HEADER_STRUCT.unpack(payload)
        synced_sample = ImuSample(
            timestamp_ns=unpacked[1],
            ax=unpacked[2],
            ay=unpacked[3],
            az=unpacked[4],
            gx=unpacked[5],
            gy=unpacked[6],
            gz=unpacked[7],
            qx=unpacked[8],
            qy=unpacked[9],
            qz=unpacked[10],
            qw=unpacked[11],
            pitch_rad=unpacked[12],
            yaw_rad=unpacked[13],
            roll_rad=unpacked[14],
        )
        return VideoPacketHeader(
            frame_timestamp_ns=unpacked[0],
            synced_sample=synced_sample,
            payload_size=unpacked[15],
        )

    def _valid_synced_sample(self, sample: ImuSample) -> ImuSample | None:
        if sample.timestamp_ns <= 0:
            return None
        return sample

    def _recv_exact(self, client: socket.socket, size: int) -> bytes | None:
        received = bytearray()
        while len(received) < size and not self.stop_event.is_set():
            try:
                chunk = client.recv(size - len(received))
            except socket.timeout:
                continue
            if not chunk:
                return None
            received.extend(chunk)
        return bytes(received)

    def _put_latest(self, target_queue: queue.Queue, item, queue_name: str) -> None:
        while True:
            try:
                target_queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    target_queue.get_nowait()
                    self._record_queue_drop(queue_name)
                except queue.Empty:
                    return

    def _drain_queue_snapshot(self, target_queue: queue.Queue, max_items: int) -> list:
        drained: list = []
        drain_limit = min(max(target_queue.qsize(), 0), max_items)
        for _ in range(drain_limit):
            try:
                drained.append(target_queue.get_nowait())
            except queue.Empty:
                break
        return drained

    def _drain_latest_from_queue(
        self,
        target_queue: queue.Queue,
        max_publish_items: int,
    ) -> tuple[list, int]:
        drained = self._drain_queue_snapshot(
            target_queue,
            max(target_queue.qsize(), 0),
        )
        drained_count = len(drained)
        if len(drained) > max_publish_items:
            self._record_video_coalesced(len(drained) - max_publish_items)
            drained = drained[-max_publish_items:]
        return drained, drained_count

    def _record_publish(self, diagnostics_key: str) -> None:
        if not self.diagnostics_enabled:
            return

        with self._diagnostics_lock:
            if diagnostics_key == "image":
                self._diagnostics_window.image_published += 1
            elif diagnostics_key == "camera_info":
                self._diagnostics_window.camera_info_published += 1
            elif diagnostics_key == "imu":
                self._diagnostics_window.imu_published += 1
            elif diagnostics_key == "synced_imu":
                self._diagnostics_window.synced_imu_published += 1

    def _record_queue_drop(self, queue_name: str) -> None:
        if not self.diagnostics_enabled:
            return

        with self._diagnostics_lock:
            if queue_name == "video":
                self._diagnostics_window.video_queue_drops += 1
            elif queue_name == "imu":
                self._diagnostics_window.imu_queue_drops += 1

    def _record_video_coalesced(self, count: int) -> None:
        if not self.diagnostics_enabled or count <= 0:
            return

        with self._diagnostics_lock:
            self._diagnostics_window.video_frames_coalesced += count

    def _record_publish_batches(self, video_batch_count: int, imu_batch_count: int) -> None:
        if not self.diagnostics_enabled or (video_batch_count == 0 and imu_batch_count == 0):
            return

        with self._diagnostics_lock:
            self._diagnostics_window.max_video_batch = max(
                self._diagnostics_window.max_video_batch,
                video_batch_count,
            )
            self._diagnostics_window.max_imu_batch = max(
                self._diagnostics_window.max_imu_batch,
                imu_batch_count,
            )

    def _log_publish_diagnostics(self) -> None:
        now = time.monotonic()
        with self._diagnostics_lock:
            window = self._diagnostics_window
            elapsed = max(now - self._diagnostics_window_started_at, 1e-6)
            self._diagnostics_window = PublishDiagnosticsWindow()
            self._diagnostics_window_started_at = now

        self.get_logger().info(
            "Publish stats %.2fs: image_raw=%.1fHz (%d), camera_info=%.1fHz (%d), "
            "imu_raw=%.1fHz (%d), imu_image_sync=%.1fHz (%d), coalesced(video=%d), "
            "drops(video=%d, imu=%d), "
            "max_batch(video=%d, imu=%d), queue_depth(video=%d, imu=%d)"
            % (
                elapsed,
                window.image_published / elapsed,
                window.image_published,
                window.camera_info_published / elapsed,
                window.camera_info_published,
                window.imu_published / elapsed,
                window.imu_published,
                window.synced_imu_published / elapsed,
                window.synced_imu_published,
                window.video_frames_coalesced,
                window.video_queue_drops,
                window.imu_queue_drops,
                window.max_video_batch,
                window.max_imu_batch,
                self.video_queue.qsize(),
                self.imu_queue.qsize(),
            )
        )

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.video_listener is not None:
            try:
                self.video_listener.close()
            except OSError:
                pass
            self.video_listener = None
        if self.imu_socket is not None:
            try:
                self.imu_socket.close()
            except OSError:
                pass
            self.imu_socket = None
        if self.video_thread.is_alive():
            self.video_thread.join(timeout=1.0)
        if self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)

    def destroy_node(self) -> bool:
        self.shutdown()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VioStreamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
