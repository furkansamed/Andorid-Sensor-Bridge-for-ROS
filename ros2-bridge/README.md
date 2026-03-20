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
2. Install ROS 2 plus Python decoding dependencies.

```bash
sudo apt install ros-$ROS_DISTRO-desktop python3-pip
python3 -m pip install av numpy
```

3. Build the workspace.

```bash
cd ~/sensorBridgeV1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --base-paths ros2-bridge --symlink-install
source install/setup.bash
```

4. Start the bridge.

```bash
ros2 launch vio_stream_bridge vio_stream_bridge.launch.py
```

5. On the phone, enter the Ubuntu machine Wi-Fi IP and tap `Start`.

## Camera calibration

For visual SLAM, you should calibrate the phone camera and then fill the calibration values in `config/vio_stream_bridge.yaml` or in a separate ROS parameter file. Until you do that, leave `publish_camera_info` as `false`.

Typical fields to replace after calibration:

- `distortion_coefficients`
- `camera_matrix`
- `rectification_matrix`
- `projection_matrix`

## Useful checks

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /imu/data_raw
ros2 topic hz /imu/image_sync
ros2 topic echo /imu/data_raw --once
ros2 topic echo /imu/image_sync --once
```
