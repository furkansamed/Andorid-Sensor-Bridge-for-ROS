# Android VIO Streamer

This repository keeps the three related parts of the system together:

- `android-app/` contains the Android Studio project that streams H.264 video over TCP and IMU over UDP.
- `pc-receiver/` contains the Windows validation receiver UI for checking that video and IMU packets arrive correctly.
- `ros2-bridge/` contains the ROS 2 package that republishes the same stream as ROS topics on Ubuntu.

## Why one repository

Keeping all three in one repository is the better default for this project because:

- the Android app, Windows receiver, and ROS 2 bridge all depend on the same network protocol
- versioning them together avoids drift between sender and receivers
- issues, documentation, and release notes stay in one place
- future protocol changes are easier to implement and review across all consumers

You should split them into separate repositories only if they start having different owners, different release cycles, or you want to open-source one part without the others.

## Suggested GitHub layout

Use a single GitHub repository with these top-level folders:

```text
android-app/
pc-receiver/
ros2-bridge/
README.md
```

## Local usage

### Android app

Open `android-app/` in Android Studio and build/run from there.

### Windows receiver

```powershell
cd pc-receiver
powershell -ExecutionPolicy Bypass -File .\run_receiver.ps1
```

### ROS 2 bridge on Ubuntu

From the repo root on Ubuntu:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 -m pip install av numpy
colcon build --base-paths ros2-bridge --symlink-install
source install/setup.bash
ros2 launch vio_stream_bridge vio_stream_bridge.launch.py
```
