# basalt_ros2

ROS2 Jazzy VIO node and ROS2 MCAP camera and IMU calibration built on [Basalt](https://gitlab.com/VladyslavUsenko/basalt) by Vladyslav Usenko et al. and a basalt_ros ROS1 library by [Matthew Tavatgis](https://github.com/Matt-Tav/), targeting Luxonis OAK-FFC-3P (IMX577 RGB camera, dual OAK-FFC-OV9282-M12 global shutter mono cameras + BNO086 IMU). Tested on Raspberry Pi 5 16GB with depthai-ros V3 Jazzy build.

![Foxglove Basalt ROS2](assets/Foxglove_Basalt_ROS2.png)

---

## 1. Installation

### 1.1 Prerequisites

Set `$DEV_HOME` in `~/.bashrc` to the parent directory of `orca_ws/`:

```bash
export DEV_HOME="/path/to/your/dev/root"  # e.g. ~ or /media/user/nvme
```

Install system dependencies:

```bash
sudo apt-get update && sudo apt-get -y install --no-install-recommends \
  software-properties-common git libusb-1.0-0-dev wget \
  python3-colcon-common-extensions zip unzip tar
```

Set USB rules for OAK cameras:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 1.2 Clone Repositories

```bash
cd $DEV_HOME/orca_ws/src

# depthai-core — required by depthai-ros
git clone https://github.com/luxonis/depthai-core.git
cd depthai-core
git submodule update --init --recursive
cd ..

# depthai-ros (v3-jazzy) — OAK camera driver
git clone https://github.com/luxonis/depthai-ros.git
cd depthai-ros && git checkout v3-jazzy && cd ..

# basalt_ros2 — this package
git clone https://github.com/roboticsmick/basalt_ros2.git
```

### 1.3 Install ROS Dependencies

```bash
cd $DEV_HOME/orca_ws
sudo rosdep init   # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 1.4 Build

> Always run `colcon build` from `orca_ws/`, not `src/`.

`-DBASALT_INSTANTIATIONS_FLOAT=ON` is required for basalt_ros2 — enables float optical flow templates not compiled upstream, and the float VIO estimator (faster on ARM NEON).

Calibration tools (`basalt_calibrate`, `basalt_calibrate_imu`) are built by default. They require **Pangolin** (vendored in `thirdparty/`) — an OpenGL GUI library that renders the interactive calibration windows.

**Step 1 — Build depthai-ros and dependencies:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential
```

> `-j1 -l1 --executor sequential` keeps RAM usage safe on Raspberry Pi 5. Pangolin's link step is heavy — parallel builds can OOM with 16 GB.

**Step 2 — Build basalt_ros2** (after depthai is installed):

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j1 -l1" colcon build \
  --symlink-install \
  --packages-select basalt_ros2 \
  --cmake-args -DBASALT_INSTANTIATIONS_FLOAT=ON
```

---

### 1.5 Source the Workspace

```bash
source $DEV_HOME/orca_ws/install/setup.bash
# Add to ~/.bashrc for persistence:
echo "source $DEV_HOME/orca_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2. Calibration

Calibration must be done before running the VIO node. Results are saved to `src/basalt_ros2/config/calibration/`.

### 2.1 Overview

```text
Recording: Stereo + IMU (30 fps, oak_ffc_3p_stereo_imu_calibration.yaml)
  ├── Step 1: Stereo camera calibration  → config/calibration/calibration.json + .yaml
  └── Step 2: IMU calibration            → config/calibration/calibration.json + .yaml (updated)
```

Both steps use the same recording and the same `--result-path`. The `--output-format yaml` flag writes both JSON (needed internally by Step 2) and YAML (recommended for editing — supports comments).

### 2.2 AprilGrid Config

Create the AprilGrid configuration file (measure your own grid):

```bash
mkdir -p $DEV_HOME/basalt_calibration
cat > $DEV_HOME/basalt_calibration/aprilgrid.json << 'EOF'
{
    "tagCols": 6,
    "tagRows": 6,
    "tagSize": 0.088,
    "tagSpacing": 0.3
}
EOF
```

`tagSpacing` is a ratio (gap / tagSize), not an absolute distance. 1% measurement error ≈ 1–2 px reprojection error.

### 2.3 Record Calibration Data

Mount the **AprilGrid on a wall** and **move the camera rig** dynamically in front of it (60–90 seconds). Use 30 fps stereo-only config.

**Terminal 1 — Camera driver:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/basalt_ros2/config/depthai-ros/oak_ffc_3p_stereo_imu_calibration.yaml \
  camera_model:=OAK-FFC-3P
```

**Terminal 2 — Record:**

```bash
mkdir -p $DEV_HOME/basalt_calibration
source /opt/ros/jazzy/setup.bash
ros2 bag record -o $DEV_HOME/basalt_calibration/stereo_imu_calibration_record_cb1_01 \
  /oak/left/image_raw \
  /oak/right/image_raw \
  /oak/imu/data
```

Stop the recording, then do a second record with the stereo and colour cameras.

**Terminal 1 — Camera driver:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/basalt_ros2/config/depthai-ros/oak_ffc_3p_rgb_stereo_calibration.yaml \
  camera_model:=OAK-FFC-3P
```

**Terminal 2 — Record:**

```bash
cd $DEV_HOME/basalt_calibration
source /opt/ros/jazzy/setup.bash
ros2 bag record -o $DEV_HOME/basalt_calibration/stereo_rgb_calibration_record_cb1_01 \
  /oak/left/image_raw \
  /oak/rgb/image_raw \
  /oak/right/image_raw
```

### 2.4 Step 1: Stereo Camera Calibration and RGB cameras

Calibration executables are built to `$DEV_HOME/orca_ws/build/basalt_ros2/`. Run from there:

```bash
cd $DEV_HOME/orca_ws/build/basalt_ros2
./basalt_calibrate \
  --dataset-path  $DEV_HOME/basalt_calibration/stereo_imu_calibration_record_cb1_01/stereo_imu_calibration_record_cb1_01_0.mcap \
  --dataset-type  mcap \
  --aprilgrid     $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path   $DEV_HOME/basalt_calibration/stereo_imu_calibration_results_cb1_01 \
  --cam-types     pinhole-radtan8 pinhole-radtan8 \
  --output-format yaml
```

**GUI sequence:**

1. Wait for corner detection
2. `init_cam_intr` → `init_cam_poses` → `init_cam_extr` → `init_opt`
3. Enable `opt_until_converge` → wait for convergence
4. `save_calib`

Good calibration: mean reprojection error < 1.0 px (ideal < 0.5 px).

For the RGB and stereo camera calibration:

```bash
cd $DEV_HOME/orca_ws/build/basalt_ros2
./basalt_calibrate \
  --dataset-path  $DEV_HOME/basalt_calibration/stereo_rgb_calibration_record_cb1_01/stereo_rgb_calibration_record_cb1_01_0.mcap \
  --dataset-type  mcap \
  --aprilgrid     $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path   $DEV_HOME/basalt_calibration/stereo_rgb_calibration_record_cb1_01_results \
  --cam-types     pinhole-radtan8 pinhole-radtan8 pinhole-radtan8\
  --output-format yaml
```

### 2.5 Step 2: IMU Stereo Calibration

Use the **same recording** and **same `--result-path`** as Step 1:

```bash
cd $DEV_HOME/orca_ws/build/basalt_ros2
./basalt_calibrate_imu \
  --dataset-path  $DEV_HOME/basalt_calibration/stereo_imu_calibration_record_cb1_01/stereo_imu_calibration_record_cb1_01_0.mcap \
  --dataset-type  mcap \
  --aprilgrid     $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path   $DEV_HOME/basalt_calibration/stereo_imu_calibration_results_cb1_01 \
  --gyro-noise-std  0.0005 \
  --accel-noise-std 0.02 \
  --gyro-bias-std   0.0001 \
  --accel-bias-std  0.001 \
  --output-format yaml
```

**IMU noise parameters** (Basalt continuous-time model — do not copy Kalibr values directly):

| IMU | `--gyro-noise-std` | `--accel-noise-std` | `--gyro-bias-std` | `--accel-bias-std` |
| --- | --- | --- | --- | --- |
| BNO086 (OAK-FFC-3P) | 0.0005 | 0.02 | 0.0001 | 0.001 |
| BMI160 (TUM-VI/EuRoC) | 0.000282 | 0.016 | 0.0001 | 0.001 |

> Do not set `gyro-noise-std` above 0.002 — it de-weights gyroscope measurements and prevents rotation constraints.

**GUI sequence:**

1. `load_dataset` → `detect_corners` → `init_cam_poses` → `init_cam_imu` → `init_opt`
2. Enable `opt_until_converge` → wait
3. Optionally enable `opt_cam_time_offset` and `opt_imu_scale`
4. `save_calib`

**Quality checks:**

- g-vector norm ≈ 9.81 m/s²
- Mean reprojection error < 2 px (ideal < 1 px)
- Spline plots (solid) should follow raw IMU data (dashed)

### 2.6 Step 3: 3-Camera Calibration (RGB Extrinsics)

TO BE TESTED

### 2.7 Copy Result to Config

```bash
# YAML (recommended — supports comments)
cp $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.yaml \
   $DEV_HOME/orca_ws/src/basalt_ros2/config/calibration/calibration.yaml

# JSON (alternative — for automated pipelines)
cp $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \
   $DEV_HOME/orca_ws/src/basalt_ros2/config/calibration/calibration.json
```

The VIO node detects format by file extension — pass either path to `calib_path`.

---

## 3. Running

**Terminal 1 — Camera driver:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/basalt_ros2/config/depthai-ros/oak_ffc_3p_vio.yaml \
  camera_model:=OAK-FFC-3P
```

**Terminal 2 — VIO node:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run basalt_ros2 visual_odometry_node \
  --ros-args \
  -p calib_path:=$(pwd)/src/basalt_ros2/config/calibration/calibration_cb1_stereo_imu.yaml \
  -p config_path:=$(pwd)/src/basalt_ros2/config/basalt_ros2/vio_config.yaml \
  -p imu_topic:=/oak/imu/data \
  -p left_image_topic:=/oak/left/image_raw \
  -p right_image_topic:=/oak/right/image_raw \
  -p publish_cloud:=true \
  -p publish_images:=true
```

**Terminal 3 — Foxglove visualisation:**

```bash
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

After ~0.3 s of stable tracking, `/basalt_vio/status` publishes `data: true` and `/odometry` begins publishing valid poses.

### 3.1 Using other cameras / IMU

> **Disclaimer:** This package was built for fun in spare time using Luxonis OAK-FFC-3P hardware. Other camera/IMU setups are **not supported**. The information below is provided as a starting point only — I have no spare time to debug other hardware configurations and very shy.

Three things to change:

**1. Your camera driver must publish these topics:**

| Topic | Message type | Notes |
|---|---|---|
| Left stereo image | `sensor_msgs/Image` | Grayscale (`mono8` or `mono16`); time-synchronized with right |
| Right stereo image | `sensor_msgs/Image` | Same resolution and encoding as left |
| IMU | `sensor_msgs/Imu` | `linear_acceleration` + `angular_velocity` populated |

**2. Pass your topic names to the VIO node** (Terminal 2):

```bash
ros2 run basalt_ros2 visual_odometry_node \
  --ros-args \
  -p calib_path:=/path/to/your/calibration.yaml \
  -p config_path:=$(pwd)/src/basalt_ros2/config/basalt_ros2/vio_config.yaml \
  -p imu_topic:=/your/imu/data \
  -p left_image_topic:=/your/left/image_raw \
  -p right_image_topic:=/your/right/image_raw
```

**3. Run calibration for your hardware** — follow section 2 with your own recording. The output `calibration.yaml` format is identical. The only IMU-specific values are the noise/bias priors passed to `basalt_calibrate_imu` (see table in section 2.5 for reference values).

---

## 4. ROS2 Topics

| Topic | Type | Always | Description |
|---|---|---|---|
| `/odometry` | `nav_msgs/Odometry` | Yes | Primary VIO output. Pose of `base_link` (left camera) in `odom` (world). Identity pose during startup/reset; valid pose+velocity once healthy. |
| `/basalt_vio/status` | `std_msgs/Bool` | Yes | `true` = VIO healthy and publishing valid odometry. Gate navigation on this. |
| `/basalt_vio/keypoint_ratio` | `std_msgs/Float32` | Yes | Ratio of VIO-confirmed 3D landmarks to optically-tracked 2D features in left camera [0–1]. Healthy ≥ 0.5; warns below 0.2. |
| `/basalt_vio/keypoint_stats` | `basalt_ros2/KeypointStats` | Yes | Per-camera tracking counts: `ratio`, `tracked_left`, `tracked_right`, `confirmed_left`, `confirmed_right`. Use for stereo balance and lighting diagnostics. |
| `/keypoints` | `sensor_msgs/PointCloud2` | `publish_cloud:=true` | Active 3D landmark positions in `base_link` (left camera) frame. Each point is `(x, y, z)` float32. |
| `/basalt_vio/left/image_annotated` | `sensor_msgs/Image` | `publish_images:=true` | Left frame with keypoints overlaid. Purple hollow ring = optically tracked 2D feature; pink filled dot = VIO-confirmed 3D landmark. |
| `/basalt_vio/right/image_annotated` | `sensor_msgs/Image` | `publish_images:=true` | Same annotation as left. Fewer pink dots expected — left camera is the primary host frame; right-camera confirmed dots only appear when a left-hosted landmark is also stereo-matched to the right camera. |
| TF: `odom` → `base_link` | TF2 | `publish_transform:=true` | VIO pose as TF transform. Not broadcast during startup. |

**Keypoint dot legend:**

| Symbol | Color | Meaning |
|---|---|---|
| Hollow ring | Purple | Optically tracked 2D feature — optical flow found it this frame; no 3D position yet |
| Filled circle | Pink | VIO-confirmed 3D landmark — triangulated by the sliding-window estimator and reprojected |

---

## 5. Parameters

### 5.1 Node Parameters (`--ros-args -p`)

| Parameter | Default | Description |
|---|---|---|
| `calib_path` | `""` | Path to calibration file (`.json` or `.yaml`). Required for IMU mode. |
| `config_path` | `""` | Path to VIO config file. Built-in defaults used if empty. |
| `left_image_topic` | `/oak/stereo/left/image` | Left camera topic |
| `right_image_topic` | `/oak/stereo/right/image` | Right camera topic |
| `imu_topic` | `""` | IMU topic. Empty = visual-odometry-only mode (no IMU). |
| `odom_frame` | `odom` | TF parent frame |
| `base_frame` | `base_link` | TF child frame |
| `publish_transform` | `true` | Broadcast TF. Set `false` if another node owns the TF tree. |
| `publish_cloud` | `true` | Publish 3D landmark point cloud |
| `publish_images` | `true` | Publish annotated stereo images |
| `thread_limit` | `0` | Max TBB threads. `0` = all cores. |
| `odom_watchdog_timeout_ms` | `5000` | Log FREEZE warning if no odometry for this many ms. Diagnostic only. |

### 5.2 VIO Config (`config/basalt_ros2/vio_config.yaml`)

Edit this file to tune algorithm behaviour. Pass via `-p config_path:=...`.

#### Optical Flow

| Parameter | Default | Description |
|---|---|---|
| `optical_flow_type` | `multiscale_frame_to_frame` | Tracker variant. `multiscale_frame_to_frame` adds sub-pixel refinement (~10% more CPU). |
| `optical_flow_detection_grid_size` | `10` | Grid cell size (px) for FAST corner detection. Smaller = denser features. 640×400, grid=10 → ~2560 cells. |
| `optical_flow_max_features` | `200` | Global feature cap per frame. `0` = unlimited. Recommended 200–400 on Raspberry Pi 5. |
| `optical_flow_fast_threshold` | `40` | FAST corner detector initial threshold. Halves adaptively to floor=5. Lower = detects weaker gradients. No extra CPU when `max_features` cap is active. |
| `optical_flow_max_recovered_dist2` | `0.04` | Forward-backward round-trip error threshold (px²). Lower = stricter. |
| `optical_flow_pattern` | `50` | LKT patch sampling pattern. Valid: 24, 50, 51, 52 only (invalid = abort). |
| `optical_flow_max_iterations` | `8` | Lucas-Kanade iterations per pyramid level. >8 gives diminishing returns. |
| `optical_flow_levels` | `4` | Pyramid depth (4 = 5 scales). More levels handle faster motion at higher CPU cost. |
| `optical_flow_epipolar_error` | `0.007` | Stereo epipolar constraint threshold (normalized coords). |
| `optical_flow_skip_frames` | `2` | Forward every Nth frame to VIO. 2 = VIO gets ~15 Hz from 30 Hz camera. |

#### Sliding Window / Keyframe

| Parameter | Default | Description |
|---|---|---|
| `vio_max_states` | `3` | Max IMU states in active window. |
| `vio_max_kfs` | `7` | Max keyframes retained. Reduce to 5 on embedded hardware. |
| `vio_min_frames_after_kf` | `1` | Min frames between new keyframes. `1` = aggressive. |
| `vio_new_kf_keypoints_thresh` | `0.7` | New keyframe when map-connected landmark ratio drops below this. |
| `vio_kf_marg_feature_ratio` | `0.1` | KF removable when <this fraction of its landmarks are still tracked. |
| `vio_marg_lost_landmarks` | `true` | Remove invisible landmarks before marginalization. |

#### Optimization

| Parameter | Default | Description |
|---|---|---|
| `vio_linearization_type` | `ABS_QR` | Solver method. `ABS_QR` most stable (recommended). `ABS_SC` / `REL_SC` faster but noisier. |
| `vio_sqrt_marg` | `true` | Square-root marginalization form. Required when using float precision. |
| `vio_max_iterations` | `7` | Max LM iterations per frame. |
| `vio_obs_std_dev` | `0.5` | Observation noise std-dev. Raise to 1.0 with poor calibration. |
| `vio_obs_huber_thresh` | `1.0` | Huber loss threshold. Lower = more aggressive outlier rejection. |
| `vio_min_triangulation_dist` | `0.03` | Minimum baseline (m) to triangulate a landmark. 3 cm suits slow AUV. |
| `vio_enforce_realtime` | `true` | Drop frames when VIO falls behind. Set `false` for bag replay. |

#### Levenberg-Marquardt Damping

| Parameter | Default | Description |
|---|---|---|
| `vio_lm_lambda_initial` | `1e-4` | Starting damping factor. Lower = more Newton-like initial step. |
| `vio_lm_lambda_min` | `1e-6` | Floor on λ. |
| `vio_lm_lambda_max` | `1e2` | Ceiling on λ. Optimization fails for the frame if exceeded. |

#### Initialization Priors

| Parameter | Default | Description |
|---|---|---|
| `vio_init_pose_weight` | `1e8` | Weight on initial position/yaw. Strong — locks first frame to origin. |
| `vio_init_ba_weight` | `1e1` | Weight on initial accel bias. Weak — allows fast calibration. |
| `vio_init_bg_weight` | `1e2` | Weight on initial gyro bias. |

#### VIO Health Tracking

These are read by the ROS2 node only — Basalt itself ignores them.

| Parameter | Default | Description |
|---|---|---|
| `vio_debug` | `false` | Enable Basalt solver verbosity (`[LINEARIZE]`/`[EVAL]` output every VIO frame). Expensive — use only for solver debugging. |
| `vio_node_debug` | `true` | Enable node-level structured health log throttled to 10 s: `kp=0.19 (46/239 L \| 22/239 R) \| speed=0.003m/s \| health: kp=OK vel=OK accel=FAIL`. Independent of `vio_debug`. |
| `vio_extended_logging` | `false` | Enable extended Basalt internal logging for offline analysis. |
| `vio_health_min_keypoints` | `1` | Minimum active 3D landmarks. Below this = keypoint health failure. |
| `vio_health_max_velocity` | `1.5` | Max expected speed (m/s). Exceeding this = velocity health failure. Set to rated top speed × 1.5. |
| `vio_health_max_acceleration` | `1.0` | Max **velocity delta per VIO frame** (m/s) before acceleration health fails. Triggers from noisy VIO estimates when keypoint quality is low, not real motion. |
| `vio_health_startup_duration` | `0.3` | Seconds all criteria must stay healthy before first pose is published. |
| `vio_health_keypoint_timeout` | `1.0` | Seconds below `vio_health_min_keypoints` before runtime reset fires. Increase for featureless scenes. |

### 5.3 Pipeline Buffer Constants (requires rebuild)

These are hardcoded in source. Edit and rebuild with `MAKEFLAGS="-j1 -l1" colcon build --packages-select basalt_ros2 --cmake-args -DBASALT_INSTANTIATIONS_FLOAT=ON`.

| Constant | File | Default | Description |
|---|---|---|---|
| `input_queue` capacity | `src/node/visual_odometer_node.cpp` | `10` | Camera→optical flow queue. Increase to 20 if `"Optical flow input queue full"` warnings appear. |
| `out_state_queue_` capacity | `src/node/visual_odometer_node.cpp` | `30` | VIO→odometry publisher queue. Increase to 60 if `/odometry` rate stalls. |
| `out_vis_queue_` capacity | `src/node/visual_odometer_node.cpp` | `30` | VIO→visualisation queue. |
| Camera frame buffer | `src/device/ros_camera.cpp` | `5` | Rolling buffer between image subscriber and camera thread. |
| IMU sample buffer | `src/device/ros_imu.cpp` | `100` | Rolling buffer between IMU subscriber and camera thread (~0.5 s at 200 Hz). |

---

## 6. Bag / MCAP Playback

Run the VIO node against a pre-recorded bag for benchmarking or offline evaluation.

**Step 1 — Inspect the bag:**

```bash
ros2 bag info /path/to/recording.mcap
```

Identify left image topic, right image topic, and IMU topic.

**Step 2 — Disable realtime enforcement** (prevents frame drops during playback):

Copy `src/basalt_ros2/config/basalt_ros2/vio_config.yaml` to `vio_config_replay.yaml` and set:

```yaml
vio_enforce_realtime: false
optical_flow_skip_frames: 0  # process every frame if bag FPS < 15
```

**Step 3 — Play bag and run VIO:**

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
ros2 bag play /path/to/recording.mcap --clock

# Terminal 2
cd $DEV_HOME/orca_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run basalt_ros2 visual_odometry_node \
  --ros-args \
  -p calib_path:=$(pwd)/src/basalt_ros2/config/calibration/calibration.yaml \
  -p config_path:=$(pwd)/src/basalt_ros2/config/basalt_ros2/vio_config_replay.yaml \
  -p imu_topic:=/imu/data \
  -p left_image_topic:=/camera/left/image_raw \
  -p right_image_topic:=/camera/right/image_raw \
  -p publish_cloud:=true \
  -p publish_images:=true
```

**Converting a ROS 1 bag to MCAP** (no ROS 1 required):

```bash
pip3 install rosbags

# Convert to ROS 2 SQLite bag
rosbags-convert my_recording.bag --dst my_ros2_bag

# Optional: convert to MCAP (for Foxglove direct open)
cat > convert_config.yaml << 'EOF'
output_bags:
  - uri: my_final_mcap
    storage_id: mcap
EOF
ros2 bag convert my_ros2_bag convert_config.yaml
```

---
## 7. Acknowledgements

None of this is mine. I've built it on the foundation of these big brains!

This package wraps the [Basalt](https://gitlab.com/VladyslavUsenko/basalt) library by Vladyslav Usenko et al. at TU Munich. Please cite their work if using this in research:

- **Visual-Inertial Mapping with Non-Linear Factor Recovery** — V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers. *IEEE RA-L 2020*. [DOI:10.1109/LRA.2019.2961227](https://doi.org/10.1109/LRA.2019.2961227)
- **Square Root Marginalization for Sliding-Window Bundle Adjustment** — N. Demmel et al. *ICCV 2021*. [arXiv:2109.02182](https://arxiv.org/abs/2109.02182)
- **The Double Sphere Camera Model** — V. Usenko, N. Demmel, D. Cremers. *3DV 2018*. [DOI:10.1109/3DV.2018.00069](https://doi.org/10.1109/3DV.2018.00069)
- **The TUM VI Benchmark** — D. Schubert et al. *IROS 2018*. [DOI:10.1109/IROS.2018.8593419](https://doi.org/10.1109/IROS.2018.8593419)

A huge thank you to [Matthew Tavatgis (Matt-Tav)](https://github.com/Matt-Tav) whose basalt_ros ROS1 library was the framework for this package. His original work provided:

- The ROS device abstraction pattern — the `RosCameraDevice` and `RosImuDevice` classes that bridge ROS subscribers into Basalt's internal data structures
- The visual odometer node architecture — the subscriber/publisher layout, threading model, and queue design that drives the VIO pipeline
- The VIO health monitoring concept — the startup gating and runtime health-check approach that prevents publishing garbage poses during initialisation and after tracking failures.

---

## 8. Want to Support Me

If you use this for your own work, and you're not a student, and you want to thank me, feel free to buy me a [ST-VIS-25 spectrometer](https://www.oceanoptics.com/spectrometer/st-vis/), or a [Headwall hyperspectral MV.C VNIR camera](https://www.automate.org/products/headwall-photonics/mv-c-vnir) or a [Specim FX10 Hyperspectral Camera](https://www.specim.com/products/specim-fx10/), or a [OAK 4 CS Global Shutter camera](https://shop.luxonis.com/products/oak-4-cs?Sensor+=OG05B10) so I can keep making cool ROS2 camera packages in my spare time ❤️

<a href="https://www.buymeacoffee.com/orcaengineer" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>

---

## 9. Disclaimer

**This is a hobby project built in my spare time after work. Use at your own risk.**

This software has been tested on a Raspberry Pi 5 16Gb with a Luxonis OAK-FFC-3P camera rig in a controlled indoor environment. **It has not been validated on large datasets, long-duration missions, or safety-critical applications.** There are likely bugs. Calibration-sensitive parameters may need tuning for your hardware.

Redistribution and use in source and binary forms, with or without modification, are permitted under the BSD 3-Clause License (see source file headers). This software is provided **"AS IS"**, without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and non-infringement. In no event shall the authors or copyright holders be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.
