# AGENTS.md — navbot_ros2 workspace

## Project

ROS 2 **Jazzy** workspace for an omniwheel navigation robot.

- Host: Orange Pi Zero 2W (aarch64), Ubuntu 24.04, Python 3.12
- Chassis: 4-wheel omni, **STM32** motor controller over USB CDC
- Sensors: **LD06** LiDAR (CP210x USB-serial), Yahboom 9-DOF **IMU** (CH340 USB-serial)
- Remote: `git@github.com:N-rwal/navbot_ros2.git` (branch `main`)
- Workspace: `/home/orangepi/ros2_ws`

## Layout

| Path | Build type | Role |
|---|---|---|
| `src/navbot_bringup` | ament_python | Launch + config entrypoint (no nodes) |
| `src/stm32_omni_driver` | ament_python | `/cmd_vel` → STM32 serial byte duties |
| `src/imu_ros2_device` | ament_python | Yahboom IMU → `/imu/data_raw`, `/imu/mag`, `/baro`, `/euler` |
| `src/ld06` | ament_cmake | **submodule** (`N-rwal/ld06`), package name `ldlidar` → `/scan` |
| `src/rf2o_laser_odometry` | ament_cmake | **submodule** (`N-rwal/rf2o_laser_odometry`) → `/odom_rf2o` |

- Directory `src/ld06` builds package **`ldlidar`** — use `ros2 launch ldlidar ...`, not `ld06`.
- Host IMU fusion: `imu_filter_madgwick` from **`ros-jazzy-imu-tools`** (apt, not in this workspace).

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/orangepi/ros2_ws
git submodule update --init --recursive   # if submodules are empty
colcon build --symlink-install
source install/setup.bash
```

- `build/`, `install/`, `log/` are gitignored.
- Single package: `colcon build --packages-select <pkg> --symlink-install`
- Always source `install/setup.bash` per shell; otherwise `Package '<pkg>' not found`.
- `--symlink-install` means launch/config edits under `src/` are picked up on restart (no rebuild needed for YAML/launch).
- IMU filter package: `sudo apt-get install ros-jazzy-imu-tools` if `ros2 pkg executables imu_filter_madgwick` is empty.

## Run

Full robot (LiDAR, TFs, IMU + Madgwick filter, rf2o, motor driver, SLAM, EKF):

```bash
source /opt/ros/jazzy/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
ros2 launch navbot_bringup robot.launch.py
```

What `robot.launch.py` starts:

- `ldlidar` (includes `ldlidar.launch.py`, defaults to `/dev/mylidar`)
- static TF `base_link → laser` at `(0.05, 0, 0.15)`, identity rotation
- static TF `base_link → imu_link` at origin, identity rotation
- `ybimu_driver` (`imu_ros2_device`) — 30 Hz, sensor's internal quaternion on `/imu/data_raw`
- `imu_filter_madgwick` (`imu_filter_madgwick_node`) — 6/9-axis host fusion → `/imu/data`
- `rf2o_laser_odometry` (params: `navbot_bringup/config/rf2o.yaml`)
- `stm32_omni_driver` (params path is hardcoded absolute, see Pitfalls)
- `slam_toolbox` `online_async_launch.py` (params: `navbot_bringup/config/slam.yaml`)
- `robot_localization` `ekf_node` (params: `navbot_bringup/config/ekf.yaml`, remaps `odometry/filtered` → `/odom`)

Teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Dataflow

```
LD06 /scan ──────────────► rf2o ──────────► /odom_rf2o (pose x/y only) ─┐
                                                                        ├─► EKF ─► /odom + TF odom→base_link
Yahboom IMU /imu/data_raw ─┐                                            │
Yahboom IMU /imu/mag ──────┴─► imu_filter_madgwick ─► /imu/data ───────┘
                                                                        
LD06 /scan ─► slam_toolbox ─► /map + TF map→odom
/cmd_vel ─► stm32_omni_driver ─► serial ─► STM32 ─► motors
```

- RF2O does **not** publish TF (`publish_tf: false`); the EKF owns `odom→base_link`.
- EKF fuses RF2O **x, y only** (do not fuse RF2O yaw — it is scan-matching based and environment-dependent).
- EKF fuses IMU **yaw + vyaw** from `/imu/data`, `imu0_relative: true`, `two_d_mode: true`.
- slam_toolbox owns global heading correction via `map→odom`.

## LiDAR scan convention (important)

`ld06` publishes a scan spanning **`[-π, π]`** with `angle_min = -π`, `angle_max = π`
and forward at index `N/2`. The index is computed as:

```cpp
float phi = -angle;                    // CW sensor -> CCW ROS
if (phi < -M_PI) phi += 2.f * M_PI;    // wrap into [-pi, pi)
int index = (int)((phi - output.angle_min) / output.angle_increment);
```

This is required by RF2O, which **ignores `scan.angle_min`** and assumes a scan
symmetric about +x (`tita = -0.5*fovh + u*...`). With the old `[0, 2π]` scan,
RF2O's pose was rotated by π: translation inverted, yaw unchanged. Do not revert
to `angle_min = 0`.

`src/ld06` history (fork `N-rwal/ld06`, `main`):
`a283850` merge of `bcbc20f` + `ee408e4`, then `d16a0b2` symmetric-scan fix.
`navbot_ros2` pins `d16a0b2`.

## Devices & udev

| Symlink | Device | USB ID |
|---|---|---|
| `/dev/mylidar` | LD06 LiDAR | CP210x `10c4:ea60` |
| `/dev/myimu` | Yahboom IMU | CH340 `1a86:7523` |
| `/dev/ttyACM0` | STM32 VCP | STM32 `0483:5740` |

Rules: `/etc/udev/rules.d/lidar.rules`, `/etc/udev/rules.d/myimu.rules`.
Reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`.

`ybimu_driver` tries `/dev/myimu`, then `/dev/ttyUSB0..2`.
`stm32_omni_driver` defaults to `/dev/ttyACM0` at 115200 baud.

## IMU pipeline

Driver (`ybimu_driver.py`):
- 30 Hz publish timer and `set_report_rate(30)` on the sensor.
- Publishes `/imu/data_raw` (sensor-internal quaternion + raw accel/gyro), `/imu/mag`, `/baro`, `/euler`.
- Mag is published **without** the original y-negation: `mag.magnetic_field.y = my`. The negation was flipped after the Madgwick heading came out mirrored; do not re-introduce unless re-validated.
- `set_algo_type(6|9)` is **not** called; the sensor's internal algorithm is whatever the firmware defaults to.

Host filter (`imu_filter_madgwick`, config `src/imu_ros2_device/config/imu_filter_param.yaml`):
```yaml
fixed_frame: "imu_link"
use_mag: true
publish_tf: false          # must stay false (no competing TF)
world_frame: "enu"
orientation_stddev: 0.15   # tune: larger = less EKF trust in filtered yaw
gain: 0.1                  # Madgwick correction gain; larger = faster, noisier
zeta: 0.0                  # gyro-bias estimation gain
```
- Consumes `/imu/data_raw` + `/imu/mag`, publishes `/imu/data` (ApproximateTime sync; driver stamps both identically).
- Post-rotation convergence (~seconds) is the Madgwick correction time constant; the EKF's `imu0` yaw trust (`orientation_stddev`) controls how much that drags the estimate.

## Parameters

- `src/stm32_omni_driver/config/params.yaml` — port, baud, `max_pwm`, `forward_gain`, `strafe_gain`, `rotation_gain`, `cmd_timeout`
- `src/navbot_bringup/config/rf2o.yaml` — scan/odom topics, frames, `freq` (10 Hz)
- `src/navbot_bringup/config/ekf.yaml` — EKF `frequency: 30`, frames, odom0/imu0 configs
- `src/navbot_bringup/config/slam.yaml` — slam_toolbox tuning (currently `minimum_time_interval: 0.2`, `map_update_interval: 0.5`, `max_laser_range: 3.0`)
- `src/imu_ros2_device/config/imu_filter_param.yaml` — Madgwick filter
- TF offsets and frames are inline in `robot.launch.py`

EKF config vector order (both odom0/imu0):

```
[ x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az ]
```

Current EKF: `odom0` x,y true (yaw false); `imu0` yaw + vyaw true.

## Pitfalls

- `robot.launch.py` passes the motor param file as an absolute path `/home/orangepi/ros2_ws/...`; prefer `FindPackageShare`.
- `static_transform_publisher` positional order in Jazzy is **`x y z yaw pitch roll frame child`**, *not* roll/pitch/yaw. The launch comments have been wrong in the past.
- Do **not** fuse RF2O yaw in the EKF. It is scan-matching derived and environment-dependent; it caused a ~9° idle oscillation and surroundings-dependent heading.
- RF2O reads the base→laser TF **only on the first scan** and falls back to identity if it is not up yet. Deferring the rf2o node (launch `TimerAction`) avoids a startup race; the current launch does not defer it.
- RF2O logs `Waiting for laser_scans....` occasionally. It is benign: its 10.0 Hz timer beats against the ~10.002 Hz scan + jitter. Not a failure; setting `freq` slightly below scan rate would silence it.
- Do not `git submodule update --remote` on `ld06` expecting the old behavior; the scan convention is now `[-π, π]`.
- `log.txt` in the workspace root is a stale error from an unsourced shell — ignore it.
- Package dir `ld06` ≠ package name `ldlidar`.
- Both serial symlinks use broad `ttyUSB*` vendor rules; verify with `ls -l /dev/mylidar /dev/myimu`.
- Don't run two `robot.launch.py` instances — serial contention; also leaves stale DDS graph entries (duplicate node names in `ros2 node list`).
- `navbot_bringup` and `imu_ros2_device` register only console scripts in `setup.py`; adding a node needs a `console_scripts` entry.
- Keep `/imu/mag` and `/imu/data_raw` frame `imu_link`; the filter and EKF rely on the static `base_link→imu_link` TF.
- No secrets/credentials in this file or repo.

## Verify

```bash
ros2 pkg list | grep -E 'navbot_bringup|stm32_omni_driver|imu_ros2_device|ldlidar|rf2o_laser_odometry'
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /imu/data_raw  # ~30 Hz
ros2 topic hz /imu/data      # ~30 Hz (filter output)
ros2 topic hz /odom_rf2o     # ~10 Hz
ros2 topic hz /odom          # ~30 Hz
ros2 topic echo /scan --once --field angle_min   # -3.14159...
ros2 topic echo /scan --once --field angle_max   #  3.14159...
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser       # (0.05, 0, 0.15), identity rotation
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

Odom sign check: push/drive the robot straight forward; `/odom_rf2o` and `/odom`
`position.x` must increase. If they decrease, the scan convention/RF2O offset regressed.

## Conventions

- `ament_python` packages: `package_name/` module, `setup.py`, `setup.cfg`, `resource/<pkg>`, `test/` lint stubs.
- `ament_cmake` packages (`ld06`, `rf2o_laser_odometry`) are vendored upstream code — keep their style.
- First-party nodes are simple `rclpy` nodes; snake_case, 4-space Python.
- Only ament lint tests exist (flake8/pep257/copyright); there are no functional tests.
- Never commit `build/`, `install/`, `log/`, `__pycache__/`, or `*.pyc`.
