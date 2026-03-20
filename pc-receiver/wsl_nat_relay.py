from __future__ import annotations

import argparse
import select
import socket
import subprocess
import threading
from contextlib import suppress
from dataclasses import dataclass


BUFFER_SIZE = 64 * 1024


def detect_wsl_ip(distro: str) -> str:
    command = [
        "wsl.exe",
        "-d",
        distro,
        "bash",
        "-lc",
        r"ip -4 addr show eth0 | sed -n 's/.*inet \([0-9.]*\)\/.*/\1/p'",
    ]
    result = subprocess.run(
        command,
        capture_output=True,
        check=True,
        text=True,
    )
    ip_address = result.stdout.strip()
    if not ip_address:
        raise RuntimeError(f"Failed to detect WSL eth0 IPv4 address for distro {distro}")
    return ip_address


def pipe_stream(source: socket.socket, destination: socket.socket) -> None:
    try:
        while True:
            data = source.recv(BUFFER_SIZE)
            if not data:
                break
            destination.sendall(data)
    except OSError:
        pass
    finally:
        with suppress(OSError):
            destination.shutdown(socket.SHUT_WR)


@dataclass
class TcpRelayServer:
    listen_host: str
    listen_port: int
    target_host: str
    target_port: int

    def serve_forever(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.listen_host, self.listen_port))
        server.listen(5)
        print(
            f"[tcp] listening on {self.listen_host}:{self.listen_port} -> "
            f"{self.target_host}:{self.target_port}",
            flush=True,
        )
        try:
            while True:
                client_socket, client_address = server.accept()
                threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_address),
                    daemon=True,
                ).start()
        finally:
            server.close()

    def _handle_client(self, client_socket: socket.socket, client_address: tuple[str, int]) -> None:
        print(f"[tcp] client connected from {client_address[0]}:{client_address[1]}", flush=True)
        try:
            upstream = socket.create_connection((self.target_host, self.target_port), timeout=5.0)
        except OSError as error:
            print(f"[tcp] failed to connect to WSL target: {error}", flush=True)
            client_socket.close()
            return

        with client_socket, upstream:
            upstream.settimeout(None)
            threads = [
                threading.Thread(target=pipe_stream, args=(client_socket, upstream), daemon=True),
                threading.Thread(target=pipe_stream, args=(upstream, client_socket), daemon=True),
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

        print(f"[tcp] client disconnected from {client_address[0]}:{client_address[1]}", flush=True)


@dataclass
class UdpRelayServer:
    listen_host: str
    listen_port: int
    target_host: str
    target_port: int

    def serve_forever(self) -> None:
        relay_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        relay_socket.bind((self.listen_host, self.listen_port))
        relay_socket.setblocking(False)
        target_endpoint = (self.target_host, self.target_port)
        last_client: tuple[str, int] | None = None
        print(
            f"[udp] listening on {self.listen_host}:{self.listen_port} -> "
            f"{self.target_host}:{self.target_port}",
            flush=True,
        )
        try:
            while True:
                readable, _, _ = select.select([relay_socket], [], [], 1.0)
                if not readable:
                    continue
                payload, address = relay_socket.recvfrom(BUFFER_SIZE)
                if address == target_endpoint:
                    if last_client is not None:
                        relay_socket.sendto(payload, last_client)
                    continue
                last_client = address
                relay_socket.sendto(payload, target_endpoint)
        finally:
            relay_socket.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward Android sensor bridge traffic from Windows into a NAT-mode WSL distro.",
    )
    parser.add_argument("--distro", default="Ubuntu-22.04", help="WSL distro name")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Windows bind address")
    parser.add_argument("--tcp-port", type=int, default=5000, help="TCP relay port")
    parser.add_argument("--udp-port", type=int, default=5001, help="UDP relay port")
    parser.add_argument(
        "--wsl-host",
        default="",
        help="Override detected WSL IPv4 address",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    wsl_host = args.wsl_host or detect_wsl_ip(args.distro)
    print(f"Using WSL target IP: {wsl_host}", flush=True)
    tcp_server = TcpRelayServer(args.listen_host, args.tcp_port, wsl_host, args.tcp_port)
    udp_server = UdpRelayServer(args.listen_host, args.udp_port, wsl_host, args.udp_port)

    threads = [
        threading.Thread(target=tcp_server.serve_forever, daemon=True),
        threading.Thread(target=udp_server.serve_forever, daemon=True),
    ]
    for thread in threads:
        thread.start()

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("\nRelay stopped.", flush=True)


if __name__ == "__main__":
    main()
