from __future__ import annotations

import math
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import av
    from PIL import Image, ImageTk
except ImportError as error:
    missing = error.name if hasattr(error, "name") else str(error)
    raise SystemExit(
        "Missing dependency: "
        f"{missing}. Run `python -m pip install -r pc-receiver/requirements.txt` first."
    ) from error


VIDEO_PORT = 5000
IMU_PORT = 5001
VIDEO_HEADER_STRUCT = struct.Struct(">qqfffffffffffffI")
IMU_STRUCT = struct.Struct(">qfffffffffffff")
BLACK_FRAME_RGB = bytes([12, 16, 20]) * 640 * 480
RAD_TO_DEG = 180.0 / math.pi


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


class ReceiverApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("VIO Stream Receiver")
        self.root.geometry("1160x820")
        self.root.minsize(980, 720)
        self.log_file_path = Path(__file__).with_name("receiver.log")

        self.stop_event = threading.Event()
        self.frame_queue: queue.Queue[Image.Image] = queue.Queue(maxsize=2)
        self.log_queue: queue.Queue[str] = queue.Queue()

        self.video_listener: socket.socket | None = None
        self.imu_socket: socket.socket | None = None
        self.video_thread: threading.Thread | None = None
        self.imu_thread: threading.Thread | None = None
        self.decoder = av.CodecContext.create("h264", "r")

        self.video_connected = False
        self.imu_connected = False
        self.video_peer = "-"
        self.imu_peer = "-"
        self.video_packets = 0
        self.imu_packets = 0
        self.video_fps = 0.0
        self.imu_hz = 0.0
        self.video_last_ts = 0
        self.synced_imu_last_ts = 0
        self.imu_last_ts = 0
        self.last_synced_sample: ImuSample | None = None
        self.last_imu_sample: ImuSample | None = None
        self.first_video_ts: int | None = None
        self.first_sync_ts: int | None = None
        self.first_imu_ts: int | None = None
        self.logged_first_frame = False
        self.logged_first_imu = False
        self.last_video_packet_count = 0
        self.last_imu_packet_count = 0
        self.last_rate_time = time.monotonic()

        self.status_var = tk.StringVar(value="Stopped")
        self.video_status_var = tk.StringVar(value="Video: waiting")
        self.imu_status_var = tk.StringVar(value="IMU: waiting")
        self.local_ips_var = tk.StringVar(value=", ".join(self.discover_ipv4_addresses()))
        self.video_stats_var = tk.StringVar(
            value=(
                "Video packets: 0\n"
                "Video FPS: 0.0\n"
                "Frame timestamp: -\n"
                "Frame T+: -\n"
                "Synced IMU timestamp: -\n"
                "Sync delta: -\n"
                "Frame accel: -\n"
                "Frame gyro: -\n"
                "Frame Euler(deg): -"
            )
        )
        self.imu_stats_var = tk.StringVar(
            value=(
                "IMU packets: 0\n"
                "IMU Hz: 0.0\n"
                "Timestamp: -\n"
                "IMU T+: -\n"
                "Accel: -\n"
                "Gyro: -\n"
                "Quat: -\n"
                "Euler(deg): -"
            )
        )

        self._build_ui()
        self._show_placeholder()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.process_ui_updates)

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=12)
        container.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(container)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Phone should connect to:").pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.local_ips_var).pack(side=tk.LEFT, padx=(6, 18))
        ttk.Label(top, text=f"TCP {VIDEO_PORT} / UDP {IMU_PORT}").pack(side=tk.LEFT)

        self.start_button = ttk.Button(top, text="Start Receiver", command=self.start_receiver)
        self.start_button.pack(side=tk.RIGHT)
        self.stop_button = ttk.Button(top, text="Stop", command=self.stop_receiver, state=tk.DISABLED)
        self.stop_button.pack(side=tk.RIGHT, padx=(0, 8))

        status_row = ttk.Frame(container, padding=(0, 12, 0, 12))
        status_row.pack(fill=tk.X)
        ttk.Label(status_row, textvariable=self.status_var).pack(anchor=tk.W)
        ttk.Label(status_row, textvariable=self.video_status_var).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(status_row, textvariable=self.imu_status_var).pack(anchor=tk.W, pady=(4, 0))

        main = ttk.Panedwindow(container, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)

        video_panel = ttk.Frame(main, padding=8)
        stats_panel = ttk.Frame(main, padding=8)
        main.add(video_panel, weight=3)
        main.add(stats_panel, weight=2)

        self.video_label = tk.Label(
            video_panel,
            width=640,
            height=480,
            bg="#0f1418",
            bd=0,
            highlightthickness=0
        )
        self.video_label.pack(fill=tk.BOTH, expand=True)

        ttk.Label(stats_panel, text="Video / Frame-Synced IMU").pack(anchor=tk.W)
        ttk.Label(
            stats_panel,
            textvariable=self.video_stats_var,
            justify=tk.LEFT,
            font=("Consolas", 11)
        ).pack(anchor=tk.W, fill=tk.X, pady=(4, 12))

        ttk.Label(stats_panel, text="Raw IMU (UDP)").pack(anchor=tk.W)
        ttk.Label(
            stats_panel,
            textvariable=self.imu_stats_var,
            justify=tk.LEFT,
            font=("Consolas", 11)
        ).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(container, text="Receiver log").pack(anchor=tk.W, pady=(12, 4))
        self.log_text = tk.Text(container, height=10, wrap=tk.WORD, font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=False)
        self.log_text.configure(state=tk.DISABLED)

    def start_receiver(self) -> None:
        if self.video_listener or self.imu_socket:
            return
        self.stop_event.clear()
        self.decoder = av.CodecContext.create("h264", "r")
        self.video_connected = False
        self.imu_connected = False
        self.video_peer = "-"
        self.imu_peer = "-"
        self.video_packets = 0
        self.imu_packets = 0
        self.video_fps = 0.0
        self.imu_hz = 0.0
        self.video_last_ts = 0
        self.synced_imu_last_ts = 0
        self.imu_last_ts = 0
        self.last_synced_sample = None
        self.last_imu_sample = None
        self.first_video_ts = None
        self.first_sync_ts = None
        self.first_imu_ts = None
        self.logged_first_frame = False
        self.logged_first_imu = False
        self.last_video_packet_count = 0
        self.last_imu_packet_count = 0
        self.last_rate_time = time.monotonic()
        self._show_placeholder()
        self.log_file_path.write_text("", encoding="utf-8")

        try:
            self.video_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.video_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.video_listener.bind(("0.0.0.0", VIDEO_PORT))
            self.video_listener.listen(1)
            self.video_listener.settimeout(1.0)

            self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.imu_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.imu_socket.bind(("0.0.0.0", IMU_PORT))
            self.imu_socket.settimeout(1.0)
        except OSError as error:
            self.stop_receiver()
            messagebox.showerror("Receiver error", f"Failed to bind ports: {error}")
            return

        self.video_thread = threading.Thread(target=self.video_loop, name="pc-video", daemon=True)
        self.imu_thread = threading.Thread(target=self.imu_loop, name="pc-imu", daemon=True)
        self.video_thread.start()
        self.imu_thread.start()
        self.status_var.set("Listening for phone")
        self.video_status_var.set("Video: waiting for TCP client")
        self.imu_status_var.set("IMU: waiting for UDP packets")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self.log("Receiver started")

    def stop_receiver(self) -> None:
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
        self.video_connected = False
        self.imu_connected = False
        self.video_peer = "-"
        self.imu_peer = "-"
        self.status_var.set("Stopped")
        self.video_status_var.set("Video: waiting")
        self.imu_status_var.set("IMU: waiting")
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)
        self.log("Receiver stopped")

    def video_loop(self) -> None:
        listener = self.video_listener
        if listener is None:
            return
        while not self.stop_event.is_set():
            try:
                client, address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return

            self.video_connected = True
            self.video_peer = f"{address[0]}:{address[1]}"
            self.video_status_var.set(f"Video: TCP connected from {self.video_peer}")
            self.log(f"Video connected from {self.video_peer}")
            client.settimeout(2.0)

            try:
                with client:
                    while not self.stop_event.is_set():
                        raw_header = self.recv_exact(client, VIDEO_HEADER_STRUCT.size)
                        if not raw_header:
                            break
                        header = self.parse_video_header(raw_header)
                        payload = self.recv_exact(client, header.payload_size)
                        if payload is None:
                            break
                        self.video_packets += 1
                        self.video_last_ts = header.frame_timestamp_ns
                        self.synced_imu_last_ts = header.synced_sample.timestamp_ns
                        self.last_synced_sample = header.synced_sample
                        if self.first_video_ts is None and header.frame_timestamp_ns > 0:
                            self.first_video_ts = header.frame_timestamp_ns
                        if self.first_sync_ts is None and header.synced_sample.timestamp_ns > 0:
                            self.first_sync_ts = header.synced_sample.timestamp_ns
                        self.decode_video_payload(payload, header)
            except OSError as error:
                self.log(f"Video socket closed: {error}")

            self.video_connected = False
            self.video_peer = "-"
            self.video_status_var.set("Video: waiting for TCP client")
            self.log("Video disconnected")

    def imu_loop(self) -> None:
        udp_socket = self.imu_socket
        if udp_socket is None:
            return
        while not self.stop_event.is_set():
            try:
                payload, address = udp_socket.recvfrom(IMU_STRUCT.size)
            except socket.timeout:
                continue
            except OSError:
                return

            if len(payload) != IMU_STRUCT.size:
                self.log(f"Ignored IMU packet with size {len(payload)}")
                continue

            sample = ImuSample(*IMU_STRUCT.unpack(payload))
            self.last_imu_sample = sample
            self.imu_packets += 1
            self.imu_last_ts = sample.timestamp_ns
            if self.first_imu_ts is None and sample.timestamp_ns > 0:
                self.first_imu_ts = sample.timestamp_ns
            if not self.imu_connected:
                self.imu_connected = True
                self.imu_peer = address[0]
                self.log(f"IMU packets arriving from {self.imu_peer}")
            if not self.logged_first_imu:
                self.logged_first_imu = True
                self.log(f"First IMU sample ts={sample.timestamp_ns}")
            self.imu_status_var.set(f"IMU: UDP packets from {address[0]}:{address[1]}")

    def recv_exact(self, client: socket.socket, size: int) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size and not self.stop_event.is_set():
            try:
                chunk = client.recv(size - len(chunks))
            except socket.timeout:
                continue
            if not chunk:
                return None
            chunks.extend(chunk)
        return bytes(chunks)

    def parse_video_header(self, payload: bytes) -> VideoPacketHeader:
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

    def decode_video_payload(self, payload: bytes, header: VideoPacketHeader) -> None:
        try:
            for packet in self.decoder.parse(payload):
                for frame in self.decoder.decode(packet):
                    rgb_frame = frame.to_ndarray(format="rgb24")
                    image = Image.fromarray(rgb_frame)
                    if not self.logged_first_frame:
                        self.logged_first_frame = True
                        delta_ms = (
                            (header.synced_sample.timestamp_ns - header.frame_timestamp_ns) / 1_000_000.0
                        )
                        self.log(
                            "First video frame decoded "
                            f"{frame.width}x{frame.height}, sync delta={delta_ms:+.3f} ms"
                        )
                    self.push_frame(image)
        except av.AVError as error:
            self.log(f"Decoder error: {error}")

    def push_frame(self, image: Image.Image) -> None:
        while self.frame_queue.qsize() >= 1:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        self.frame_queue.put_nowait(image)

    def process_ui_updates(self) -> None:
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            self.update_video_widget(frame)

        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.append_log(line)

        self.refresh_stats()
        self.root.after(100, self.process_ui_updates)

    def refresh_stats(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_rate_time
        if elapsed >= 1.0:
            self.video_fps = (self.video_packets - self.last_video_packet_count) / elapsed
            self.imu_hz = (self.imu_packets - self.last_imu_packet_count) / elapsed
            self.last_video_packet_count = self.video_packets
            self.last_imu_packet_count = self.imu_packets
            self.last_rate_time = now

        synced_sample = self.last_synced_sample
        frame_ts = "-" if self.video_last_ts == 0 else str(self.video_last_ts)
        frame_relative = self.format_relative_ns(self.video_last_ts, self.first_video_ts)
        synced_ts = "-" if self.synced_imu_last_ts == 0 else str(self.synced_imu_last_ts)
        sync_delta = self.format_delta_ms(self.video_last_ts, self.synced_imu_last_ts)
        frame_accel = self.format_triplet(
            synced_sample.ax,
            synced_sample.ay,
            synced_sample.az,
        ) if synced_sample else "-"
        frame_gyro = self.format_triplet(
            synced_sample.gx,
            synced_sample.gy,
            synced_sample.gz,
        ) if synced_sample else "-"
        frame_euler = self.format_euler_deg(synced_sample) if synced_sample else "-"
        self.video_stats_var.set(
            f"Video packets: {self.video_packets}\n"
            f"Video FPS: {self.video_fps:.1f}\n"
            f"Frame timestamp: {frame_ts}\n"
            f"Frame T+: {frame_relative}\n"
            f"Synced IMU timestamp: {synced_ts}\n"
            f"Sync delta: {sync_delta}\n"
            f"Frame accel: {frame_accel}\n"
            f"Frame gyro: {frame_gyro}\n"
            f"Frame Euler(deg): {frame_euler}"
        )

        sample = self.last_imu_sample
        imu_ts = "-" if self.imu_last_ts == 0 else str(self.imu_last_ts)
        imu_relative = self.format_relative_ns(self.imu_last_ts, self.first_imu_ts)
        accel = self.format_triplet(sample.ax, sample.ay, sample.az) if sample else "-"
        gyro = self.format_triplet(sample.gx, sample.gy, sample.gz) if sample else "-"
        quat = self.format_quaternion(sample) if sample else "-"
        euler = self.format_euler_deg(sample) if sample else "-"
        self.imu_stats_var.set(
            f"IMU packets: {self.imu_packets}\n"
            f"IMU Hz: {self.imu_hz:.1f}\n"
            f"Timestamp: {imu_ts}\n"
            f"IMU T+: {imu_relative}\n"
            f"Accel: {accel}\n"
            f"Gyro: {gyro}\n"
            f"Quat: {quat}\n"
            f"Euler(deg): {euler}"
        )

        if self.video_connected or self.imu_connected:
            self.status_var.set("Receiving data")
        elif self.video_listener or self.imu_socket:
            self.status_var.set("Listening for phone")

    def update_video_widget(self, image: Image.Image) -> None:
        render = image.resize((640, 480), Image.Resampling.BILINEAR)
        photo = ImageTk.PhotoImage(render)
        self.video_label.configure(image=photo)
        self.video_label.image = photo

    def _show_placeholder(self) -> None:
        image = Image.frombytes("RGB", (640, 480), BLACK_FRAME_RGB)
        self.update_video_widget(image)

    def append_log(self, line: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}"
        self.log_queue.put(line)
        print(line, flush=True)
        with self.log_file_path.open("a", encoding="utf-8") as log_file:
            log_file.write(line + "\n")

    def discover_ipv4_addresses(self) -> list[str]:
        addresses: set[str] = set()
        host_name = socket.gethostname()
        try:
            for address in socket.gethostbyname_ex(host_name)[2]:
                if address and not address.startswith("127."):
                    addresses.add(address)
        except socket.gaierror:
            pass
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect(("8.8.8.8", 80))
                addresses.add(probe.getsockname()[0])
        except OSError:
            pass
        return sorted(addresses) or ["127.0.0.1"]

    def format_triplet(self, x: float, y: float, z: float) -> str:
        return f"{x:+.3f}, {y:+.3f}, {z:+.3f}"

    def format_quaternion(self, sample: ImuSample) -> str:
        return (
            f"x={sample.qx:+.4f}, y={sample.qy:+.4f}, "
            f"z={sample.qz:+.4f}, w={sample.qw:+.4f}"
        )

    def format_euler_deg(self, sample: ImuSample) -> str:
        return (
            f"P={sample.pitch_rad * RAD_TO_DEG:+.1f}, "
            f"Y={sample.yaw_rad * RAD_TO_DEG:+.1f}, "
            f"R={sample.roll_rad * RAD_TO_DEG:+.1f}"
        )

    def format_relative_ns(self, timestamp_ns: int, base_ns: int | None) -> str:
        if timestamp_ns <= 0 or base_ns is None or base_ns <= 0:
            return "-"
        return f"{(timestamp_ns - base_ns) / 1_000_000_000.0:+.3f} s"

    def format_delta_ms(self, left_ns: int, right_ns: int) -> str:
        if left_ns <= 0 or right_ns <= 0:
            return "-"
        return f"{(right_ns - left_ns) / 1_000_000.0:+.3f} ms"

    def on_close(self) -> None:
        self.stop_receiver()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ttk.Style(root).theme_use("clam")
    app = ReceiverApp(root)
    app.start_receiver()
    root.mainloop()


if __name__ == "__main__":
    main()
