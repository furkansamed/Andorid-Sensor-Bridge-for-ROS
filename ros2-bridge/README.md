# VIO Stream Bridge for ROS 2

This package listens for the Android streamer protocol and republishes it as ROS 2 topics.

## Published topics

- `/camera/image_raw` as `sensor_msgs/msg/Image`
- `/camera/camera_info` as `sensor_msgs/msg/CameraInfo` when `publish_camera_info:=true`
- `/imu/data_raw` as `sensor_msgs/msg/Imu` from the raw UDP IMU stream
- `/imu/image_sync` as `sensor_msgs/msg/Imu` from the IMU sample embedded in each video packet header

## Timestamp handling

The phone now sends both camera frames and IMU in the same `elapsedRealtimeNanos` domain:

- Raw IMU over UDP includes accel, gyro, quaternion, and Euler angles.
- Each TCP video packet includes the encoded H.264 payload plus a frame-synced IMU snapshot with the same timestamp domain.

The bridge maps the phone clock onto the current ROS clock using a single offset estimated from the first packet it receives. Because the frame-synced IMU sample rides inside the video header, `/camera/image_raw` and `/imu/image_sync` stay aligned at the packet level.

## Ubuntu setup

1. Copy or clone this repository onto the Ubuntu machine.
2. Install ROS 2 plus the CycloneDDS runtime and Python decoding dependencies.

```bash
sudo apt install ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-rmw-cyclonedds-cpp python3-pip
python3 -m pip install av numpy
```

3. Build the workspace.

```bash
cd ~/sensorBridgeV1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --base-paths ros2-bridge --symlink-install
```

4. Source the project-scoped ROS environment.

```bash
source ros2-bridge/scripts/use_cyclonedds.sh
```

5. Start the bridge.

```bash
ros2 launch vio_stream_bridge vio_stream_bridge.launch.py
```

6. On the phone, enter the Ubuntu machine Wi-Fi IP and tap `Start`.

## WSL mirrored networking

If you are running Ubuntu in WSL with mirrored networking, keep these settings in place on Windows:

- `%USERPROFILE%\.wslconfig` with `networkingMode=mirrored`
- `%USERPROFILE%\.wslconfig` with `[experimental] hostAddressLoopback=true`
- Hyper-V firewall inbound rules for `5000/TCP`, `5001/UDP`, and `7400-7600/UDP`

The helper script keeps `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI=file://.../cyclonedds.xml` scoped to the current shell so the bridge and `ros2` CLI use the same discovery settings.
If the ROS CLI daemon ever becomes stale, short commands like `ros2 topic list`, `ros2 node list`, and `ros2 topic type` now reset it automatically once. You can still force a manual reset with `ros2_reset_daemon`.

## WSL NAT mode

If you keep WSL in `networkingMode=nat` for stability, the phone can no longer reach the WSL IP directly from the LAN. In that setup:

1. Start the ROS 2 bridge inside WSL as usual.
2. On Windows, run the relay script from `pc-receiver`:

```powershell
.\run_wsl_nat_relay.ps1
```

3. In the Android app, enter the Windows Wi-Fi IP instead of the WSL IP.

The relay forwards TCP `5000` and UDP `5001` from Windows into the current WSL distro IP.

If you want every interactive WSL terminal to come up with this environment already loaded, add:

```bash
vio_cyclone_helper="$HOME/ros2_ws/src/android_sensor_bridge/ros2-bridge/scripts/use_cyclonedds.sh"
if [ -f "$vio_cyclone_helper" ]; then
    VIO_CYCLONEDDS_QUIET=1 source "$vio_cyclone_helper"
fi
```

to `~/.bashrc`.

## Publish diagnostics

If you want to verify whether bursty behavior happens inside the bridge publisher itself or only in `ros2 topic hz`, launch the node with diagnostics enabled:

```bash
ros2 launch vio_stream_bridge vio_stream_bridge.launch.py diagnostics_enabled:=true diagnostics_period_sec:=1.0
```

The node will log one summary line per window, for example:

- `image_raw` publish count and effective Hz
- `imu_raw` publish count and effective Hz
- `imu_image_sync` publish count and effective Hz
- `coalesced(video=...)` showing how many queued video frames were intentionally skipped to keep the stream low-latency
- queue drop counts for video and IMU
- maximum number of queued messages drained in one publish timer cycle

The default pacing keeps the bridge low-latency by publishing from a single `0.002s` timer. Each cycle publishes at most one raw IMU sample and one queued video frame. If the decoder gets ahead, older queued video frames are coalesced into the newest one instead of being burst-published back-to-back.

## Camera calibration

For visual SLAM, you should calibrate the phone camera and then fill the calibration values in `config/vio_stream_bridge.yaml` or in a separate ROS parameter file. Until you do that, leave `publish_camera_info` as `false`.

Typical fields to replace after calibration:

- `distortion_coefficients`
- `camera_matrix`
- `rectification_matrix`
- `projection_matrix`

## Useful checks

```bash
source ros2-bridge/scripts/use_cyclonedds.sh
ros2 topic list --no-daemon --spin-time 10
ros2 topic hz /camera/image_raw --spin-time 5
ros2 topic hz /imu/data_raw --spin-time 5
ros2 topic hz /imu/image_sync --spin-time 5
ros2 topic echo /imu/data_raw sensor_msgs/msg/Imu --no-daemon --once
ros2 topic echo /imu/image_sync sensor_msgs/msg/Imu --no-daemon --once
```
