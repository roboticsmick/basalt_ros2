# BASALT ROS

A ROS2 Jazzy build designed to work with Luxonis depthai-ros and the OAK-FFC-3P camera module.

![Foxglove Basalt ROS2](assets/Foxglove_Basalt_ROS2.png)

## TO DO

[] Add doxygen comments to libraries
[] All files, classes, and public functions must include Doxygen documentation for clear parameter and function definitions:

* Use `@brief` for one-line summaries
* Document all parameters with `@param[in/out]`
* Include `@return` for non-void functions
* Add `@pre` tags matching code assertions (NASA principle compliance)
  Example:

    ```cpp
    /**
     * @brief Brief description of the class/function.
     *
     * Detailed description providing more context, usage examples,
     * and any important considerations.
     *
     * @param parameter_name Description of the parameter.
     * @return Description of the return value.
     */
    class MyClass {
        // ...
    };
    ```

[x] Add reliability metric / odom freeze detection
[x] Methods for handling poor keyframes environment
[x] VIO health tracking with automatic reset (keypoints, velocity, acceleration)
[x] Startup criteria — suppress output until VIO is confirmed healthy
[x] Camera time offset correction enabled in VIO core
[x] `publish_transform` flag to decouple TF and odometry publication
[ ] Calibration only works with pinhole-radtan8 model. Implementation and testing needs to be done for other models.
[ ] Update calibration to export calibration file as yaml instead of json
[ ] Remove unused scripts and libraries from basalt and basalt-ros libraries that are not used in ros2 implementation
[ ] 3D point cloud is visualised in foxglove. Remove Basalt GUI testing environment as not used.

---

## Build

> **Always run `colcon build` from the workspace root (`ros2_ws/`), never from `src/`.** Running from `src/` puts `build/`, `install/`, and `log/` inside the source tree.

Source ROS2 before any build:

```bash
source /opt/ros/jazzy/setup.bash
```

### Full workspace build

Builds all packages in the workspace. The workspace root contains a `colcon.meta` file that automatically sets `CMAKE_PREFIX_PATH=/usr/local` for all depthai packages — no manual prefix is needed.

> **Parallelism warning:** `--parallel-workers N` and `MAKEFLAGS="-jN"` are **multiplicative**.
> `--parallel-workers 4` with `MAKEFLAGS="-j4"` = up to 16 simultaneous compiler processes.
> Keep the product ≤ 16 — Eigen/basalt template instantiation peaks at ~1.5 GB per process.

```bash
cd $DEV_HOME/ros2_ws
MAKEFLAGS="-j4" colcon build --parallel-workers 4
```

> **Jetson:** Use `MAKEFLAGS="-j2" colcon build --parallel-workers 2` to stay within RAM limits.

### VIO node only (faster iteration)

Builds only basalt_ros2. Calibration tools and Pangolin GUI are excluded by default.

```bash
cd $DEV_HOME/ros2_ws
MAKEFLAGS="-j8" colcon build --packages-select basalt_ros2 --parallel-workers 1
```

### With calibration tools

Includes `basalt_calibrate` and `basalt_calibrate_imu` (requires Pangolin — significantly longer compile):

```bash
cd $DEV_HOME/ros2_ws
MAKEFLAGS="-j8" colcon build --packages-select basalt_ros2 --parallel-workers 1 \
  --cmake-args -DBASALT_BUILD_CALIBRATION_TOOLS=ON
```

### depthai-ros driver only

Required for OAK-FFC-3P camera. Build separately if camera driver changes are needed. `CMAKE_PREFIX_PATH` is handled automatically by `colcon.meta`.

```bash
cd $DEV_HOME/ros2_ws
MAKEFLAGS="-j8" colcon build --packages-select depthai_ros_driver --parallel-workers 1
```

### Clean rebuild

If you see cmake install errors (e.g. missing `.a` files from stale build cache):

```bash
cd $DEV_HOME/ros2_ws
rm -rf build/basalt_ros2 install/basalt_ros2
MAKEFLAGS="-j8" colcon build --packages-select basalt_ros2 --parallel-workers 1
```

### colcon.meta — workspace build configuration

The file `ros2_ws/colcon.meta` encodes per-package cmake arguments so they are applied automatically on every `colcon build`, without needing to set environment variables manually. It currently sets `CMAKE_PREFIX_PATH=/usr/local` for the four depthai packages so they can find the depthai-core library installed there. Do not delete this file.

---

## Camera Calibration

Calibration must be done before running the VIO node. The calibration tools are built as part of the normal colcon build and are located in the build directory.

### Prerequisites

Set `$DEV_HOME` in your `~/.bashrc` to the parent directory of `ros2_ws/` (same as for the depthai-ros build):

```bash
export DEV_HOME="/path/to/your/dev/root"  # e.g. ~ or /media/user/nvme
```

`basalt_calibration/` (where recordings and results are stored) lives at `$DEV_HOME/basalt_calibration/`.

### Build Output Location

After building with colcon, the calibration executables are at:

```text
$DEV_HOME/ros2_ws/build/basalt_ros2/basalt_calibrate
$DEV_HOME/ros2_ws/build/basalt_ros2/basalt_calibrate_imu
```

Run them from that directory:

```bash
cd $DEV_HOME/ros2_ws/build/basalt_ros2
source /opt/ros/jazzy/setup.bash
```

### AprilGrid Configuration

Create the AprilGrid configuration file (measure your own grid):

```bash
cat > $DEV_HOME/basalt_calibration/aprilgrid.json << 'EOF'
{
    "tagCols": 6,
    "tagRows": 6,
    "tagSize": 0.0882,
    "tagSpacing": 0.303
}
EOF
```

`tagSpacing` is a **ratio** (gap / tagSize), not an absolute measurement. A 1% error in measurements causes ~1–2 px reprojection error.

### Calibration Overview

```text
Recording 1: Stereo + IMU (30 fps, stereo-only driver)
  ├── Step 1: Stereo camera calibration  → stereo_imu_calibration_results/calibration.json
  └── Step 2: IMU calibration            → stereo_imu_calibration_results/calibration.json (updated)

Recording 2: RGB + Stereo + IMU (~4.5 fps, 3-camera sync driver)  [optional]
  └── Step 3: 3-camera calibration       → stereo_rgb_imu_calibration_results/calibration.json

Step 4: Merge stereo+IMU with RGB extrinsics → merged_calibration_results/calibration.json  [optional]

Final: Copy result to config/calibration.json
```

**IMPORTANT:** The `--result-path` for Steps 1 and 2 **must be identical**. The number of cameras in `calibration.json` must match the recording — using a 3-camera file with a 2-camera recording causes silent failures.

### Step 1: Stereo Camera Calibration

Mount the **AprilGrid on a wall** and **move the camera rig** dynamically in front of it. Use Recording 1 (stereo + IMU, 30 fps).

```bash
./basalt_calibrate \
  --dataset-path  $DEV_HOME/basalt_calibration/stereo_imu_calibration_record/stereo_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid  $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_imu_calibration_results \
  --cam-types pinhole-radtan8 pinhole-radtan8
```

**In the GUI:**

1. Wait for corner detection to complete
2. Click **"init_cam_intr"** → **"init_cam_poses"** → **"init_cam_extr"** → **"init_opt"**
3. Check **"opt_until_converge"** — wait for convergence
4. Click **"save_calib"**

Good calibration: mean reprojection error < 1.0 px (ideal < 0.5 px).

### Step 2: IMU Calibration

Use the **same Recording 1** and the **same `--result-path`** as Step 1. The stereo `calibration.json` is loaded automatically.

**IMU noise parameters** (Kalibr continuous-time model):

| IMU | `--gyro-noise-std` | `--accel-noise-std` | `--gyro-bias-std` | `--accel-bias-std` |
| --- | --- | --- | --- | --- |
| BMI270 (OAK-FFC-3P) | 0.0005 | 0.02 | 0.0001 | 0.001 |
| BMI160 (TUM-VI/EuRoC) | 0.000282 | 0.016 | 0.0001 | 0.001 |

Warning: Do not set `gyro-noise-std` above 0.002 — it de-weights gyroscope measurements and prevents the optimizer from constraining rotation.

```bash
./basalt_calibrate_imu \
  --dataset-path $DEV_HOME/basalt_calibration/stereo_imu_calibration_record/stereo_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_imu_calibration_results \
  --gyro-noise-std 0.0005 \
  --accel-noise-std 0.02 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001
```

**In the GUI (buttons left to right):**

1. **"load_dataset"** → **"detect_corners"** → **"init_cam_poses"** → **"init_cam_imu"** → **"init_opt"**
2. Check **"opt_until_converge"** — wait for convergence
3. Optionally enable `opt_cam_time_offset` and `opt_imu_scale` for refinement
4. Click **"save_calib"** — updates `calibration.json` with IMU extrinsics

**Evaluate:**
- g vector norm should be close to 9.81 m/s²
- Mean reprojection error < 2 px (ideal < 1 px)
- Spline plots (solid) should follow raw IMU data (dashed)

### Step 3: 3-Camera Calibration (Optional — RGB Extrinsics)

Only needed if using the RGB camera. Use Recording 2 (~4.5 fps, all 3 cameras). Mount camera on tripod, **move the AprilGrid board** in front of the cameras.

```bash
./basalt_calibrate \
  --dataset-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_record/stereo_rgb_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results \
  --cam-types pinhole-radtan8 ds pinhole-radtan8
```

Use `ds` (double sphere) for the IMX577 RGB (113° HFOV). Same GUI workflow as Step 1.

### Step 4: Merge Calibrations (Optional — RGB Only)

Combines stereo+IMU (Step 2) with RGB extrinsics (Step 3):

```bash
python3 $DEV_HOME/ros2_ws/src/basalt_ros2/scripts/merge_calibrations.py \
  --stereo-imu $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \
  --three-cam $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results/calibration.json \
  --output $DEV_HOME/basalt_calibration/merged_calibration_results/calibration.json
```

For VIO (stereo + IMU only), skip Steps 3–4 and use the 2-camera result directly.

### Copy Result to Config

```bash
cp $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \
   $DEV_HOME/ros2_ws/src/basalt_ros2/config/calibration.json
```

---

## Testing instructions

```bash
# 1. Terminal 1 — Camera driver
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_stereo_vio.yaml \
  camera_model:=OAK-FFC-3P
```

```bash
# 2. Terminal 2 — VIO node
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run basalt_ros2 visual_odometry_node \
  --ros-args \
  -p calib_path:=$(pwd)/src/basalt_ros2/config/calibration.json \
  -p config_path:=$(pwd)/src/basalt_ros2/config/vio_config.json \
  -p imu_topic:=/oak/imu/data \
  -p left_image_topic:=/oak/left/image_raw \
  -p right_image_topic:=/oak/right/image_raw \
  -p publish_cloud:=true \
  -p publish_images:=true
```

```bash
# 3. Terminal 3 — Verify topics and health status
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic list | grep -E "(odometry|basalt_vio)"

# Monitor health status (true = VIO healthy and publishing valid odometry)
ros2 topic echo /basalt_vio/status

# Monitor keypoint quality ratio (VIO-confirmed / optically-tracked, healthy ~0.5+)
ros2 topic echo /basalt_vio/keypoint_ratio
```

After ~0.3 s of stable tracking you should see `/basalt_vio/status` publish `data: true` and `/odometry` begin publishing valid poses. If the VIO loses tracking it will reset automatically and re-enter the startup sequence.


## Configuration

Parameters are split across two files by concern:

| File | Purpose |
|---|---|
| `config/vio_config.json` / `vio_config.yaml` | Basalt algorithm tuning **and** VIO health thresholds |
| `config/vo_config.json` | Algorithm tuning for visual-odometry-only mode (no IMU) |
| ROS2 parameters (`--ros-args -p`) | Deployment settings: topic names, frame IDs, TF flags |

**Edit `vio_config.yaml`** when tuning how the VIO algorithm behaves or how aggressively it resets.  
**Edit ROS2 parameters** when changing topics, frame names, or infrastructure settings.

### Node Parameters (ROS2 `-p` args)

These control the ROS2 node itself and are independent of the Basalt algorithm.

| Parameter | Default | Description |
|---|---|---|
| `calib_path` | `""` | Path to calibration file (`.json` or `.yaml`). Required for IMU mode. If empty, falls back to ROS CameraInfo. |
| `config_path` | `""` | Path to VIO config file. If empty, Basalt built-in defaults are used. |
| `left_image_topic` | `/oak/stereo/left/image` | Left camera image topic |
| `right_image_topic` | `/oak/stereo/right/image` | Right camera image topic |
| `imu_topic` | `""` | IMU topic. Leave empty to run in visual-odometry-only mode (no IMU fusion). |
| `odom_frame` | `odom` | TF parent frame for published odometry |
| `base_frame` | `base_link` | TF child frame for published odometry |
| `publish_transform` | `true` | Broadcast odom→base_link TF transform. Set `false` if another node owns the TF tree and you only need `/odometry`. |
| `publish_cloud` | `true` | Publish 3D landmark point cloud on `/keypoints` |
| `publish_images` | `true` | Publish annotated stereo images on `/basalt_vio/left/image_annotated` etc. |
| `thread_limit` | `0` | Max TBB threads for VIO solver. `0` = use all available cores. |
| `odom_watchdog_timeout_ms` | `5000` | Milliseconds without odometry output before a FREEZE warning is logged. Does not trigger a reset — diagnostic only. |

### Applying a config file

Pass `config_path` and node parameters at launch:

```bash
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run basalt_ros2 visual_odometry_node \
  --ros-args \
  -p calib_path:=$(pwd)/src/basalt_ros2/config/calibration.json \
  -p config_path:=$(pwd)/src/basalt_ros2/config/vio_config.json \
  -p imu_topic:=/oak/imu/data \
  -p left_image_topic:=/oak/left/image_raw \
  -p right_image_topic:=/oak/right/image_raw
```

If `config_path` is omitted, built-in defaults are used silently.

---

## Config Parameter Reference

### Optical Flow Parameters

These control the Lucas-Kanade patch tracker that finds and tracks image features between frames.

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `optical_flow_type` | string | `multiscale_frame_to_frame` | Tracker variant. `multiscale_frame_to_frame` adds sub-pixel refinement across pyramid levels; improves tracking on fast motion and small details (~10% more CPU than `frame_to_frame`) |
| `optical_flow_detection_grid_size` | int | `10` | Size (px) of each detection cell. Smaller = denser features. At 640×400: grid 10 → ~2560 features, grid 40 → ~160 features |
| `optical_flow_max_recovered_dist2` | float | `0.04` | Forward-backward consistency check threshold. Smaller = stricter. 0.04 rejects if round-trip error > 0.2 px |
| `optical_flow_pattern` | int | `50` | Sampling pattern for tracked patch (valid: 24, 50, 51, 52 — invalid value aborts). 50/51 are equivalent 50/51-point patterns |
| `optical_flow_max_iterations` | int | `6` | Max Lucas-Kanade iterations per pyramid level. More = more precise, more CPU. >6 gives diminishing returns |
| `optical_flow_levels` | int | `4` | Image pyramid depth (4 → 5 scales). More levels handle faster motion; costs more CPU |
| `optical_flow_epipolar_error` | float | `0.01` | Epipolar constraint threshold for stereo matching (normalized coords). 0.01 is lenient enough for typical calibration noise |
| `optical_flow_skip_frames` | int | `2` | Send every N-th result to VIO. 2 = every other frame → VIO gets ~15 Hz from 30 Hz camera; halves optical flow CPU |

### Sliding Window and Keyframe Parameters

These control when new keyframes are created and how many states are kept in the optimizer.

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `vio_max_states` | int | `3` | Max IMU states in active window. Larger = more pre-integration history, more compute |
| `vio_max_kfs` | int | `7` | Max keyframes retained. Reduce to 5 on embedded, raise to 10 for more map coverage |
| `vio_min_frames_after_kf` | int | `1` | Minimum frames between new keyframes. `1` = aggressive (new KF every ~2 frames), more map updates |
| `vio_new_kf_keypoints_thresh` | float | `0.7` | Keyframe triggered when ratio of map-connected landmarks drops below this. `0.7` = new KF when >30% of tracked points are new |
| `vio_kf_marg_feature_ratio` | float | `0.1` | KF is removable when <this fraction of its features are still tracked. Lower = keep KFs longer |
| `vio_marg_lost_landmarks` | bool | `true` | Remove invisible landmarks before marginalization. Keeps map lean |

### Optimization Parameters

These control the non-linear bundle adjustment solver that refines poses and landmarks.

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `vio_linearization_type` | string | `ABS_QR` | Solver method. ABS_QR = QR (most stable, recommended). ABS_SC = Schur. REL_SC = relative Schur |
| `vio_sqrt_marg` | bool | true | Store marginalization prior in QR form. Improves numerical stability. Keep true with ABS_QR |
| `vio_max_iterations` | int | 7 | Max Levenberg-Marquardt iterations per frame. More = better convergence, higher latency |
| `vio_obs_std_dev` | float | 0.5 | Observation noise std-dev (pixels). Raise to 1.0 if vision residuals large (poor calibration) |
| `vio_obs_huber_thresh` | float | 1.0 | Huber loss threshold. Lower (0.5) = more aggressive outlier rejection |
| `vio_min_triangulation_dist` | float | `0.03` | Minimum baseline (meters) for landmark triangulation. 0.03 m (3 cm) allows triangulation with small motion; good for slow AUV |
| `vio_enforce_realtime` | bool | `true` | Drop incoming frames if VIO processing falls behind. Prevents queue buildup; set `false` only if VIO comfortably sustains camera rate |

### Levenberg-Marquardt Damping

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `vio_lm_lambda_initial` | float | 1e-4 | Starting damping factor. Lower = more aggressive (Newton-like) initial step |
| `vio_lm_lambda_min` | float | 1e-6 | Floor on λ. Prevents collapse to zero when LM succeeds every step |
| `vio_lm_lambda_max` | float | 1e2 | Ceiling on λ. If backtracking exceeds this, optimization fails for the frame |

### Initialization Priors

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `vio_init_pose_weight` | float | 1e8 | Weight on initial position/yaw (very strong — locks first frame to origin) |
| `vio_init_ba_weight` | float | 1e1 | Weight on initial accel bias (weak — allows fast calibration) |
| `vio_init_bg_weight` | float | 1e2 | Weight on initial gyro bias |

### VIO Health Tracking Parameters

These parameters live in `vio_config.yaml` / `vio_config.json` alongside the algorithm params. They are intentionally co-located so anyone tuning the VIO algorithm can also tune the health thresholds in one place. They are read by the ROS2 node after loading the Basalt config — Basalt itself ignores them.

| Parameter | Default | Description |
|---|---|---|
| `vio_health_min_keypoints` | `1` | Minimum number of active 3D landmarks. Below this, the keypoint tracker is considered unhealthy. Raise to `10`–`20` to require a richer map before considering the VIO valid. |
| `vio_health_max_velocity` | `1.5` | Maximum expected speed (m/s). If VIO estimates a speed above this, velocity health fails. **Set to the AUV's rated top speed × 1.5 as a safety margin.** |
| `vio_health_max_acceleration` | `0.2` | Maximum expected speed *change* between consecutive VIO outputs (m/s). Detects sudden tracking jumps that indicate a corrupted state, not real acceleration. Set higher for platforms with strong thrusters or frequent surge/stop. |
| `vio_health_startup_duration` | `0.3` | Seconds all three criteria must be continuously healthy before the node declares startup complete and begins publishing valid odometry. Increase to `1.0` for more confidence before first pose. |
| `vio_health_keypoint_timeout` | `1.0` | Seconds keypoints can remain below `vio_health_min_keypoints` before a **runtime reset** is triggered. A short timeout recovers quickly after brief occlusion; increase to `2.0` to tolerate longer feature loss (e.g. passing over featureless sand). |

### Tuning for Common Scenarios

#### Very slow motion / near-static (AUV hovering, benthic survey)

```json
{
  "config.optical_flow_detection_grid_size": 5,
  "config.vio_min_triangulation_dist": 0.01,
  "config.vio_enforce_realtime": false
}
```

Even denser features (grid 5 → ~8000 features at 640×400) and a smaller triangulation baseline compensate for minimal camera translation. Disable realtime enforcement when frame rate is low and VIO can process everything.

#### Fast motion / high rotation (surface vessel, aerial)

```json
{
  "config.optical_flow_detection_grid_size": 20,
  "config.optical_flow_max_iterations": 8,
  "config.vio_new_kf_keypoints_thresh": 0.8,
  "config.vio_obs_std_dev": 1.0
}
```

Sparser features reduce optical flow CPU on fast platforms. More LK iterations and relaxed observation noise improve robustness when per-frame motion is large.

#### CPU-constrained hardware (embedded ARM)

```json
{
  "config.optical_flow_type": "frame_to_frame",
  "config.optical_flow_detection_grid_size": 30,
  "config.vio_max_kfs": 5,
  "config.vio_max_iterations": 5
}
```

Single-scale tracker (drop multiscale), fewer features, smaller KF window, fewer optimizer iterations. `optical_flow_skip_frames: 2` and `vio_enforce_realtime: true` are already the defaults.

#### Poor stereo calibration / wide-angle lenses

```json
{
  "config.optical_flow_epipolar_error": 0.02,
  "config.vio_obs_std_dev": 1.0,
  "config.vio_obs_huber_thresh": 0.5
}
```

More lenient epipolar check (0.02 vs default 0.01) keeps more stereo matches when calibration is imperfect. Higher observation noise down-weights imprecise measurements; lower Huber threshold rejects large outliers.

### Verifying the config is loaded

Check the VIO node startup log:

```
[INFO] Loaded VIO config from /path/to/vio_config.json
```

If this line does not appear, `config_path` was not set and defaults are in use. Enable verbose logging by setting `vio_debug: true` in the JSON.

---

## VIO Health Tracking and Automatic Reset

The node monitors three independent health criteria and automatically resets the VIO estimator when any of them fail. Optical flow is **not** reset — it keeps running across resets so the new VIO cycle initialises faster.

### How it works

```
Startup phase
  ├── All three trackers must be healthy for vio_health_startup_duration seconds
  └── /basalt_vio/status: false — /odometry publishes identity pose (no garbage data)

Running phase (after startup_success)
  ├── /basalt_vio/status: true — /odometry publishes real VIO poses
  ├── Health checked at 10 Hz
  └── Any tracker fails → should_reset_ → VIO torn down → reset → back to startup

On reset
  ├── Optical flow continues (faster re-initialisation)
  ├── VIO estimator destroyed and recreated
  ├── All queues drained
  └── Health trackers cleared
```

### Three health criteria

| Criterion | Measured from | Triggers reset when |
|---|---|---|
| **Keypoints** | Count of active 3D landmarks (`/keypoints`) | Drops below `vio_health_min_keypoints` for longer than `vio_health_keypoint_timeout` seconds |
| **Velocity** | Magnitude of VIO-estimated velocity | Exceeds `vio_health_max_velocity` (m/s) for any single step |
| **Acceleration** | Change in velocity between consecutive VIO outputs | Exceeds `vio_health_max_acceleration` (m/s) for any single step |

Velocity and acceleration use zero-tolerance (immediate failure, no timeout) to catch sudden tracking divergence quickly.

### Published topics

| Topic | Type | Content |
| --- | --- | --- |
| `/basalt_vio/status` | `std_msgs/Bool` | `true` when startup is complete and VIO is actively publishing valid odometry. `false` during startup and after a reset. Published at 10 Hz. |
| `/basalt_vio/keypoint_ratio` | `std_msgs/Float32` | Ratio of VIO-confirmed 3D landmarks to optically-tracked 2D features (0–1). A healthy value is above 0.5. Below 0.2 triggers a warning. |
| `/odometry` | `nav_msgs/Odometry` | Publishes identity pose with zero twist during startup/reset. Publishes real VIO pose during running phase. Never stops publishing. |

### Tuning for the AUV

The defaults are set conservatively for the AUV's low-speed profile (≤ 1.5 m/s forward, low-texture seabed). Adjust for your operating conditions:

**Slow AUV, low-texture seabed:**

```yaml
# In vio_config.yaml — default settings, should work well
vio_health_min_keypoints:    1
vio_health_max_velocity:     1.5
vio_health_max_acceleration: 0.2
vio_health_startup_duration: 0.3
vio_health_keypoint_timeout: 1.0
```

**AUV with higher speed or regular surge/stop manoeuvres:**

```yaml
vio_health_max_velocity:     3.0   # Increase to rated speed × 1.5
vio_health_max_acceleration: 0.5   # Allow faster velocity changes
```

**Passing over featureless areas (sand, open water):**

```yaml
vio_health_min_keypoints:    1     # Keep low — any landmarks is better than none
vio_health_keypoint_timeout: 2.0   # Wait longer before deciding tracking is lost
```

**Require higher confidence before publishing to navigation:**

```yaml
vio_health_min_keypoints:    15    # Need a richer map
vio_health_startup_duration: 1.0   # Healthy for 1 full second before publishing
```

### Diagnosing resets

The node logs which criterion triggered each reset:

```
[WARN] Keypoint health failure — triggering VIO reset
[WARN] Velocity health failure — triggering VIO reset
[WARN] Acceleration health failure — triggering VIO reset
```

- **Frequent keypoint resets** — the scene has insufficient texture, or `vio_health_min_keypoints` is too high. Lower `vio_health_min_keypoints` or increase `vio_health_keypoint_timeout`.
- **Velocity resets** — VIO is producing unrealistic speed estimates. Either the real platform is exceeding `vio_health_max_velocity`, or VIO has diverged. If the AUV is genuinely stationary when this fires, it is a divergence — check calibration.
- **Acceleration resets** — the VIO state jumped suddenly. Likely a tracking failure rather than real motion. Lower `vio_health_max_acceleration` to be more sensitive, or raise it if the platform has genuine rapid velocity changes.

### Camera time offset correction

The Basalt VIO core corrects camera timestamps by an offset stored in `calibration.json`:

```json
"cam_time_offset_ns": 198
```

This corrects for the fixed hardware delay between camera exposure and IMU timestamp. If calibration was performed with `opt_cam_time_offset` enabled in the IMU calibration step, this value is non-zero and meaningful. If your calibration did not estimate the time offset, this will be 0 and has no effect.

---

## Pipeline Buffer Tuning

The pipeline uses five internal buffer/queue constants that are hardcoded in C++ source files. They are infrastructure parameters (not VIO algorithm parameters) and do not appear in `vio_config.json`. If you see dropped-frame or overflow warnings in the log, these are the values to adjust.

After changing any value, rebuild:

```bash
cd $DEV_HOME/ros2_ws
MAKEFLAGS="-j8" colcon build --packages-select basalt_ros2 --parallel-workers 1
```

### `optical_flow_->input_queue` capacity

**File:** `src/node/visual_odometer_node.cpp` — search for `input_queue.set_capacity`
**Default:** `10`

Queue between the camera polling thread and the optical flow processor. At 6 Hz camera, 10 = ~1.7 s of headroom.

- **Increase to 20** if you see `"Optical flow input queue full"` warnings during normal operation (optical flow is falling behind).
- **Decrease to 5** to reduce latency at the cost of more dropped frames under CPU load.

### `out_state_queue_` / `out_vis_queue_` capacity

**File:** `src/node/visual_odometer_node.cpp` — search for `out_state_queue_.set_capacity`
**Default:** `30` (each)

Output queues from the VIO estimator to the odometry publisher and visualisation publisher. At 10 Hz, 30 = 3 s of buffer.

- **Increase to 60** if you see stalls in `/odometry` publication rate (downstream consumer can't keep up).
- **Decrease to 10** on memory-constrained hardware. Risk: more dropped odometry frames if the publishing thread is briefly blocked.

### Camera frame buffer cap

**File:** `src/device/ros_camera.cpp` — search for `image_buffer_.size() > 5`
**Default:** `5`

Rolling frame buffer between the ROS image subscriber and the camera polling thread. At 6 Hz, 5 = ~0.8 s max latency before the oldest frame is dropped.

- **Increase to 10** if you see `"Camera buffer full"` warnings during normal operation (image pipeline is transiently overloaded).
- **Decrease to 3** to minimize processing latency at the cost of less tolerance for transient CPU spikes.

### IMU sample buffer cap

**File:** `src/device/ros_imu.cpp` — search for `imu_buffer_.size() > 100`
**Default:** `100`

Rolling buffer between the ROS IMU subscriber and the camera processing thread's IMU forwarding loop. At ~200 Hz IMU, 100 = ~0.5 s headroom.

- **Increase to 200** if you see `"IMU buffer full"` warnings (IMU data arriving faster than it is consumed, unlikely at 200 Hz).
- No reason to decrease — IMU samples are small (~40 bytes each).

---

## Keypoint Visualization

When `publish_images:=true`, two annotated image topics are published:

- `/basalt_vio/left/image_annotated`
- `/basalt_vio/right/image_annotated`

View them in Foxglove by subscribing to either topic.

### Dot legend

| Dot | Color | Meaning |
|-----|-------|---------|
| Hollow ring | Yellow | Optical flow is *currently tracking* this 2D feature. No 3D position yet. |
| Filled circle | Green | Feature has been triangulated into a 3D landmark and accepted into the VIO map. Dot is its reprojection from 3D back onto the image. |

Every green dot is also inside a yellow ring. Yellow-only rings are features the tracker sees but VIO has not yet triangulated — typically because:
- The feature is too new (appeared in the last 1–2 frames)
- Insufficient stereo baseline for triangulation (`vio_min_triangulation_dist`)
- The current frame is not a keyframe, so no new landmarks are being added

A high ratio of green-to-yellow dots indicates the VIO map is healthy and well-initialized.

### Why the right image has fewer dots

Feature detection runs on the **left camera only**. New left-camera features are then projected into the right image via KLT tracking. Two things reduce the right-camera count:

1. **Tracking failures** — if a patch can't be matched across the stereo baseline (occlusion, out of frame, bad texture), the point doesn't appear on the right.
2. **Epipolar filtering** — points that violate stereo geometry (epipolar error > `optical_flow_epipolar_error`) are removed from the right image. The left image is never filtered this way.

This asymmetry is normal and by design. A large discrepancy (>50% fewer right dots) may indicate imperfect stereo calibration — try raising `optical_flow_epipolar_error` to 0.01.

### Increasing keypoint density

The main density control is `optical_flow_detection_grid_size`. It defines the cell size (px) for corner detection — at most one FAST corner is detected per empty cell.

| Grid size | Features at 640×400 | Use case |
|-----------|---------------------|----------|
| 5 | ~8000 max | Maximum density, high CPU |
| 10 **(default)** | ~2560 max | Dense features, rich map |
| 20 | ~640 max | Good balance on faster platforms |
| 30 | ~280 max | Moderate density, lower CPU |
| 40 | ~160 max | Sparse, low CPU |

The default of 10 provides a rich feature map well-suited to slow AUV motion. Raise it (e.g. to 20–30) on CPU-constrained hardware or faster platforms where tracking more features doesn't add value.

If the right camera still shows heavy dropout after adjusting density, relax the epipolar filter beyond the default 0.01:

```json
{
  "config.optical_flow_epipolar_error": 0.02
}
```

---
