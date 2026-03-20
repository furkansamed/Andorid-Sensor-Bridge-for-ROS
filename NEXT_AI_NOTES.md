# Next AI Notes

## Current networking decision

- WSL is intentionally running in `nat` mode, not `mirrored`.
- Windows file: [C:/Users/hp/.wslconfig](C:/Users/hp/.wslconfig)
- Current content:

```ini
[wsl2]
networkingMode=nat
dnsTunneling=true
```

## Why NAT is being used

- During Android sensor bridge testing on `2026-03-21`, Windows hit extreme RAM and disk pressure.
- The actual memory growth was in kernel `nonpaged pool`, not in user-space ROS/Python processes.
- Most likely trigger was `WSL mirrored networking + Cloudflare WARP + Hyper-V` interacting badly under sustained TCP/UDP traffic.
- Cloudflare WARP has been removed from the machine.
- Do not switch back to `mirrored` unless there is a very specific reason and WARP stays absent.

## Current runtime topology

The Android app does not connect directly to the WSL IP anymore.

Current path:

`Android phone -> Windows Wi-Fi IP -> Windows NAT relay -> WSL ROS bridge -> ROS 2 topics`

## Required startup order

1. Start the Windows relay:

```powershell
cd C:\Users\hp\Desktop\ws\sensorBridgeV1\pc-receiver
.\run_wsl_nat_relay.ps1
```

2. Start the ROS bridge inside WSL:

```bash
ros2 launch vio_stream_bridge vio_stream_bridge.launch.py
```

3. In the Android app, use the Windows Wi-Fi IP, not the WSL IP.

Current Windows Wi-Fi IP during setup was:

`192.168.1.36`

Do not assume this is static. Re-check with `ipconfig` if needed.

## Important files

- Windows relay:
  - `pc-receiver/wsl_nat_relay.py`
  - `pc-receiver/run_wsl_nat_relay.ps1`
- ROS bridge package:
  - `ros2-bridge/vio_stream_bridge/bridge_node.py`
- ROS launch:
  - `ros2-bridge/launch/vio_stream_bridge.launch.py`
- CycloneDDS helper:
  - `ros2-bridge/scripts/use_cyclonedds.sh`

## ROS environment notes

- The workspace is configured to use CycloneDDS.
- `ros2` short CLI commands may auto-recover from a stale daemon.
- Manual recovery command if needed:

```bash
ros2_reset_daemon
```

## Known operational caveats

- If the Android app cannot connect, first verify that the Windows relay process is actually running.
- If ROS topics exist but no data is flowing, verify that only one `vio_stream_bridge` instance is running.
- If video path dies but raw IMU still flows, check the relay and the TCP `5000` path before changing ROS code.
- Old Hyper-V firewall allow rules from the mirrored-mode setup may still exist. They are not currently needed for the NAT path, but removing them requires admin PowerShell and has not been fully cleaned up yet.

## Quick health checks

WSL:

```bash
ros2 topic list
ros2 topic hz /imu/data_raw --spin-time 5
ros2 topic hz /imu/image_sync --spin-time 5
ros2 topic hz /camera/image_raw --spin-time 5
```

Windows:

```powershell
Get-NetTCPConnection -State Listen | Where-Object { $_.LocalPort -eq 5000 }
Get-NetUDPEndpoint | Where-Object { $_.LocalPort -eq 5001 }
```

## Summary

If the next AI needs to continue work, assume:

- NAT mode is intentional.
- Direct phone -> WSL IP access is no longer the design.
- Windows relay is part of the required production path on this machine.
