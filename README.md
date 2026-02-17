# Basalt ROS2

**ROS2 Jazzy Port of Basalt VIO** - Camera/IMU Calibration and Visual-Inertial Odometry

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green.svg)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange.svg)](https://ubuntu.com/)

---

## About This Fork

This is a **community-maintained ROS2 port** of the excellent [Basalt VIO project](https://gitlab.com/VladyslavUsenko/basalt) originally developed by **Vladyslav Usenko, Nikolaus Demmel, and collaborators** at the Technical University of Munich.

- **Original Project:** <https://vision.in.tum.de/research/vslam/basalt>
- **Original Repository:** <https://gitlab.com/VladyslavUsenko/basalt>

> **All credit for the core algorithms, camera models, and VIO implementation belongs to the original authors.**
> This fork only adds ROS2 Jazzy compatibility.

### Target Hardware

This fork is specifically developed for use with:
- **DepthAI OAK-FFC-3P** module with stereo and RGB cameras
- **OV9282** stereo cameras (global shutter, M12 lens mount)
- **IMX577** RGB camera (rolling shutter, M12 lens mount)
- Integration with **depthai-ros** and **depthai-core** libraries

---

## Current Status

| Component | Status | Notes |
|-----------|--------|-------|
| **Camera Calibration** | ✅ Working | Tested with OAK-FFC-3P, MCAP bags |
| **IMU Calibration** | ⚠️ Testing | Optimizer restored to match original Basalt |
| **VIO** | ⚠️ Untested | Needs calibration files first |
| **Mapping** | ⚠️ Untested | Needs VIO working first |

---

## Changelog

### Version 0.2.1 (December 2024) - Improved Calibration Messages

| Change | Description | Files Modified |
|--------|-------------|----------------|
| **Clear Initialization Messages** | Replaced confusing "Pinhole fallback" messages with clear two-phase explanation | `cam_calib.cpp` |
| **Target Model Display** | Now shows target camera model throughout initialization (not "pinhole") | `cam_calib.cpp` |
| **Next Steps Guide** | Added GUI workflow guide after initialization completes | `cam_calib.cpp` |
| **Phase Labels** | Added "Phase 1: Single-Frame" and "Phase 2: Multi-Frame" headers | `cam_calib.cpp` |

**What changed:** The calibration terminal output now clearly explains that:

1. Initialization estimates fx, fy, cx, cy as starting values
2. Distortion parameters start at 0 and are refined during optimization
3. The target model (e.g., `pinhole-radtan8`) is always shown, not confused with initialization method

### Version 0.2.0 (December 2024) - Camera Calibration Working

| Change | Description | Files Modified |
|--------|-------------|----------------|
| **ROS2 Bag Support** | Added `dataset_io_rosbag2.cpp/h` for reading ROS2 MCAP/SQLite3 bags | `src/io/`, `include/basalt/io/` |
| **ROS2 API Updates** | Updated from `time_stamp` to `recv_timestamp` (Jazzy API change) | `dataset_io_rosbag2.cpp` |
| **SerializedMessage Fix** | Added conversion helper for `rcutils_uint8_array_t` to `rclcpp::SerializedMessage` | `dataset_io_rosbag2.cpp` |
| **GCC 13 Compatibility** | Added missing `#include <cstdint>` headers to Pangolin | `thirdparty/Pangolin/` |
| **CMake Fixes** | Fixed duplicate "uninstall" target conflict between Eigen and ament_cmake | `CMakeLists.txt`, `thirdparty/basalt-headers/thirdparty/eigen/CMakeLists.txt` |
| **Color Image Support** | Added BGR8/RGB8/BGRA8/RGBA8 image encoding support (converted to grayscale) | `dataset_io_rosbag2.cpp` |
| **Timestamp Sync** | Added 5ms tolerance for multi-camera timestamp synchronization | `dataset_io_rosbag2.cpp` |
| **TBB Alignment Fix** | Changed `CalibInitPoseMap` from TBB to `std::unordered_map` to fix Eigen alignment crashes | `calibration_helper.h` |
| **Thread Limiting** | Added option to limit TBB threads to prevent system overload | `calibraiton_helper.cpp` |
| **Progress Output** | Added progress indicators for long-running operations | `calibraiton_helper.cpp`, `cam_calib.cpp` |

### To Do

- [ ] Test and fix IMU calibration (`basalt_calibrate_imu`)
- [ ] Test VIO with calibrated OAK-FFC-3P
- [ ] Create ROS2 node wrapper for real-time VIO
- [ ] Integration examples with depthai-ros
- [ ] Mapping functionality testing
- [ ] Documentation for depthai-core BasaltVIO integration

---

## Features

- **Camera Calibration** - Intrinsic and extrinsic calibration using AprilGrid
- **Camera-IMU Calibration** - Joint camera and IMU calibration
- **Visual-Inertial Odometry** - Real-time state estimation
- **Mapping** - Sparse map creation and localization
- **Multiple Camera Models** - See [Camera Model Selection Guide](#camera-model-selection-guide)

### Supported Dataset Formats
- ROS2 MCAP bags (`.mcap`) ✅ Tested
- ROS2 SQLite3 bags (`.db3`)
- EuRoC format
- TUM-VI format
- KITTI format

---

## Camera Model Selection Guide

Choosing the right camera model is **critical** for good calibration results. The wrong model can result in high reprojection errors (>2 pixels) even with good data.

### Available Camera Models

| Model | CLI Flag | Parameters | Best For | Initialization |
|-------|----------|------------|----------|----------------|
| **Pinhole** | `pinhole` | 4 (fx, fy, cx, cy) | Cameras with no distortion | Always works |
| **Pinhole-RadTan8** | `pinhole-radtan8` | 12 (fx, fy, cx, cy, k1-k6, p1, p2) | M12 lenses, OpenCV compatible | Pinhole fallback |
| **Double Sphere** | `ds` | 6 (fx, fy, cx, cy, xi, alpha) | Wide-angle, fisheye | Needs good corners |
| **Kannala-Brandt** | `kb4` | 8 (fx, fy, cx, cy, k1-k4) | Fisheye (OpenCV compatible) | Needs good corners |
| **Extended Unified** | `eucm` | 6 (fx, fy, cx, cy, alpha, beta) | Wide-angle | Needs good corners |
| **Unified** | `ucm` | 5 (fx, fy, cx, cy, alpha) | Wide-angle | Needs good corners |

### Model Parameters Explained

#### `pinhole` - Simple Pinhole (4 parameters)
```
fx, fy  - Focal length in pixels (x, y)
cx, cy  - Principal point (optical center)
```
**Use when:** Camera has negligible distortion (<0.5%)

#### `pinhole-radtan8` - Pinhole + Radial-Tangential Distortion (12 parameters)
```
fx, fy  - Focal length in pixels
cx, cy  - Principal point
k1, k2, k3 - Radial distortion (numerator)
k4, k5, k6 - Radial distortion (denominator)
p1, p2  - Tangential distortion
```
**Use when:**
- Standard M12 lenses with 1-5% distortion
- Need OpenCV-compatible output
- Other models fail to initialize

#### `ds` - Double Sphere (6 parameters)
```
fx, fy  - Focal length in pixels
cx, cy  - Principal point
xi      - First sphere parameter [-1, 1]
alpha   - Second sphere parameter [0, 1]
```
**Use when:** Wide-angle or fisheye lenses (>90° FOV)

#### `kb4` - Kannala-Brandt (8 parameters)
```
fx, fy  - Focal length in pixels
cx, cy  - Principal point
k1, k2, k3, k4 - Radial distortion coefficients
```
**Use when:** Fisheye lenses, need OpenCV fisheye compatibility

### Recommended Models by Lens Type

| Lens Type | FOV | Distortion | Recommended Model |
|-----------|-----|------------|-------------------|
| Standard M12 (2.8mm) | 75-100° | ~1.5% | `pinhole-radtan8` |
| Wide-angle M12 | 100-150° | 2-10% | `ds` or `kb4` |
| Fisheye | >150° | >10% | `kb4` or `ds` |
| Machine vision (low distortion) | <90° | <0.5% | `pinhole` |

### Example Commands

**OAK-FFC-3P with M12 lenses (recommended):**
```bash
./basalt_calibrate \
  --dataset-path /path/to/recording.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/calibration_result/ \
  --cam-types pinhole-radtan8 pinhole-radtan8 pinhole-radtan8
```

**Wide-angle fisheye cameras:**
```bash
./basalt_calibrate \
  --dataset-path /path/to/recording.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/calibration_result/ \
  --cam-types ds ds ds
```

**Mixed setup (fisheye RGB + standard stereo):**
```bash
./basalt_calibrate \
  --dataset-path /path/to/recording.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/calibration_result/ \
  --cam-types pinhole-radtan8 kb4 pinhole-radtan8
```

---

## AprilGrid Configuration

### Configuration File Format

Create a JSON file (e.g., `aprilgrid.json`):

```json
{
    "tagCols": 6,
    "tagRows": 6,
    "tagSize": 0.088,
    "tagSpacing": 0.3
}
```

### Parameter Definitions

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `tagCols` | Number of tags horizontally | Count the tags |
| `tagRows` | Number of tags vertically | Count the tags |
| `tagSize` | Size of one tag in **meters** | Measure edge of black square with calipers |
| `tagSpacing` | **Ratio** of gap to tag size | `gap_between_tags / tagSize` |

### ⚠️ Common Mistake: tagSpacing

`tagSpacing` is a **RATIO**, not an absolute measurement!

**Example calculation:**
- Tag size: 88.2mm (0.0882m)
- Gap between tags: 26.7mm (0.0267m)
- tagSpacing = 0.0267 / 0.0882 = **0.303**

**Wrong:** `"tagSpacing": 0.0267` (treating as absolute value)
**Correct:** `"tagSpacing": 0.303` (ratio)

### Verifying Your Measurements

Use calipers to measure precisely:
1. **tagSize**: Measure from outer edge to outer edge of the black square (not including white border)
2. **Gap**: Measure from one tag's edge to the next tag's edge
3. **Calculate ratio**: gap / tagSize

A 1% error in measurements can cause ~1-2 pixel reprojection error!

---

## Installation

### Prerequisites

- **Ubuntu 24.04** (tested)
- **ROS2 Jazzy**
- **GCC 13+** or **Clang 15+**

### Step 1: Install ROS2 Jazzy

```bash
# Follow official instructions: https://docs.ros.org/en/jazzy/Installation.html
sudo apt update && sudo apt install -y ros-jazzy-desktop
```

### Step 2: Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libeigen3-dev libtbb-dev libopencv-dev \
    libfmt-dev libglew-dev libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev
```

### Step 3: Clone and Build

```bash
# Clone with submodules
git clone --recursive https://github.com/roboticsmick/basalt_ros2.git
cd basalt_ros2

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Build Output

Executables are created in the `build/` directory:
- `basalt_calibrate` - Camera calibration tool ✅ Working
- `basalt_calibrate_imu` - Camera-IMU calibration tool ⚠️ Untested
- `basalt_vio` - Visual-Inertial Odometry ⚠️ Untested
- `basalt_mapper` - Mapping tool ⚠️ Untested

---

## Camera Calibration Workflow

### Step 1: Record Calibration Data

You need **two separate recordings** for the full calibration pipeline:

| Recording           | Config                        | Cameras                    | Purpose                                          |
|---------------------|-------------------------------|----------------------------|--------------------------------------------------|
| **A: Camera calib** | `oak_ffc_3p_sync.yaml`        | All 3 (left + rgb + right) | Calibrate all camera intrinsics and extrinsics   |
| **B: IMU calib**    | `oak_ffc_3p_stereo_imu.yaml`  | Stereo only (left + right) | Calibrate IMU extrinsics at higher FPS           |

**Why two recordings?** The 3-camera config (all BGR8) saturates USB bandwidth, limiting FPS to ~4.5. IMU calibration requires 15-30+ fps to properly constrain time alignment and motion dynamics. The stereo-only config drops the RGB camera, allowing 30 fps.

#### Recording A: Camera Calibration

Mount camera on tripod. Move the **AprilGrid board** slowly in front of the cameras, covering all image regions.

```bash
# Terminal 1: 3-camera sync driver
cd ~/dai_ws && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_sync.yaml \
  camera_model:=OAK-FFC-3P

# Terminal 2: Record (30-60 seconds)
ros2 bag record -o cam_calibration \
  /oak/left/image_raw /oak/rgb/image_raw /oak/right/image_raw /oak/imu/data
```

#### Recording B: IMU Calibration

Mount the **AprilGrid on a wall**. Hold the camera rig and **move it dynamically** — rotate around all 3 axes (pitch, yaw, roll).

```bash
# Terminal 1: Stereo-only driver (30 fps, no RGB)
cd ~/dai_ws && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_stereo_imu.yaml \
  camera_model:=OAK-FFC-3P

# Terminal 2: Record stereo + IMU (60-90 seconds)
ros2 bag record -o imu_calibration \
  /oak/left/image_raw /oak/right/image_raw /oak/imu/data
```

**IMU recording tips:**
- Hold still 2-3 seconds at start and end
- Keep AprilGrid visible in both cameras throughout
- Include smooth accelerations and decelerations
- Gyroscope should reach 1-5 rad/s during rotations

### Step 2: Create AprilGrid Config

```bash
cat > aprilgrid.json << 'EOF'
{
    "tagCols": 6,
    "tagRows": 6,
    "tagSize": 0.0882,
    "tagSpacing": 0.303
}
EOF
```

### Step 3: Run Camera Calibration

**3-camera calibration** (Recording A) — use `ds` (double sphere) for the wide-angle IMX577 RGB camera:

```bash
cd basalt_ros2/build
source /opt/ros/jazzy/setup.bash

./basalt_calibrate \
  --dataset-path /path/to/cam_calibration/cam_calibration_0.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/oak_results_3cam/ \
  --cam-types pinhole-radtan8 ds pinhole-radtan8
```

**Stereo-only calibration** (Recording B) — needed before IMU calibration:

```bash
./basalt_calibrate \
  --dataset-path /path/to/imu_calibration/imu_calibration_0.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/oak_results_stereo/ \
  --cam-types pinhole-radtan8 pinhole-radtan8
```

### Step 4: In the GUI

1. Wait for corner detection to complete
2. Click **"init_cam_intr"** - Initialize camera intrinsics
3. Click **"init_cam_poses"** - Compute initial poses
4. Click **"init_cam_extr"** - Initialize extrinsics
5. Click **"init_opt"** - Initialize optimizer
6. Check **"opt_until_converge"** - Run optimization until convergence
7. Wait for convergence (watch reprojection error)
8. Click **"save_calib"** - Save calibration.json

### Step 5: Evaluate Results

**Good calibration indicators:**
- Mean reprojection error: **< 1.0 pixel** (ideal: < 0.5)
- Converged without warnings
- Extrinsics match physical camera positions

**If reprojection error is high (>2 pixels):**
1. Verify AprilGrid measurements with calipers
2. Try a different camera model
3. Check for motion blur in images
4. Ensure AprilGrid is perfectly flat
5. Re-record with slower movements

---

## Camera + IMU Calibration Workflow

After completing camera calibration and saving `calibration.json`, you can calibrate the camera-IMU extrinsics. This determines the transform between the IMU and cameras (`T_i_c`), which is required for VIO.

### Step 1: Record IMU Calibration Data

**IMPORTANT: You MUST record a separate dataset for IMU calibration.** The camera calibration recording will NOT work because the camera/IMU was stationary (you moved the AprilGrid board). For IMU calibration, you must **move the camera+IMU rig** in front of a **stationary AprilGrid**.

```bash
ros2 bag record -o imu_calibration_recording \
    /oak/left/image_raw \
    /oak/rgb/image_raw \
    /oak/right/image_raw \
    /oak/imu/data
```

**Recording procedure:**

1. Mount the AprilGrid on a **wall or flat surface** (it must stay perfectly still)
2. Hold the camera rig and stand ~0.3-1.0m from the board
3. **Move the camera dynamically** - rotate around all 3 axes (pitch, yaw, roll)
4. Include smooth accelerations and decelerations
5. Keep the AprilGrid visible in at least one camera most of the time
6. Duration: 60-120 seconds

**Common mistake:** If you used a tripod-mounted camera and moved the board for camera calibration, you **cannot reuse** that recording for IMU calibration. The IMU must experience dynamic motion (rotations > 30 deg/s) for the calibration to work. You can verify your recording has sufficient motion by checking that gyroscope magnitudes reach 1-5 rad/s during the recording.

### Step 2: Find IMU Noise Parameters

The IMU noise parameters come from your IMU's datasheet. They use the [Kalibr continuous-time noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model):

| Parameter | Unit | Description |
|-----------|------|-------------|
| `--gyro-noise-std` | rad/s/√Hz | Gyroscope white noise density |
| `--accel-noise-std` | m/s²/√Hz | Accelerometer white noise density |
| `--gyro-bias-std` | rad/s²/√Hz | Gyroscope bias random walk |
| `--accel-bias-std` | m/s³/√Hz | Accelerometer bias random walk |

**Recommended values by IMU:**

| IMU | Gyro Noise | Accel Noise | Gyro Bias | Accel Bias |
|-----|-----------|-------------|-----------|------------|
| BNO086 (OAK-FFC-3P) | 0.0005 | 0.02 | 0.0001 | 0.001 |
| BMI160 (TUM-VI/EuRoC) | 0.000282 | 0.016 | 0.0001 | 0.001 |
| Snapdragon (UZH-FPV) | 0.05 | 0.1 | 4e-5 | 0.002 |

If you don't know your IMU's specs, start with the defaults (BMI160 row) and adjust if needed. The BNO086 outputs calibrated/fused data so its effective noise density is similar to or lower than raw MEMS sensors. **Warning:** Setting `gyro-noise-std` too high (e.g. >0.002) will de-weight gyroscope measurements and prevent the optimizer from constraining rotation dynamics.

### Step 3: Run IMU Calibration

The `--result-path` **must point to the same folder** where `calibration.json` was saved from camera calibration:

```bash
cd basalt_ros2/build
source /opt/ros/jazzy/setup.bash

./basalt_calibrate_imu \
  --dataset-path /path/to/imu_calibration_recording/imu_calibration_recording_0.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/oak_calibration_result/ \
  --gyro-noise-std 0.0005 \
  --accel-noise-std 0.02 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001
```

Example (OAK-FFC-3P with BNO086):

```bash
./basalt_calibrate_imu \
  --dataset-path /media/logic/USamsung/oak_calibration/calibration_recording_basalt2/calibration_recording_basalt2_0.mcap \
  --dataset-type mcap \
  --aprilgrid /media/logic/USamsung/oak_calibration/aprilgrid_large.json \
  --result-path /media/logic/USamsung/oak_calibration/oak_results/ \
  --gyro-noise-std 0.0005 \
  --accel-noise-std 0.02 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001
```

### Step 4: In the IMU Calibration GUI

Follow the buttons **in order from left to right**:

1. Click **"load_dataset"** - Load the IMU calibration recording
2. Click **"detect_corners"** - Detect AprilGrid corners (uses cached results if available)
3. Click **"init_cam_poses"** - Compute initial camera poses from corners
4. Click **"init_cam_imu"** - Initialize the rotation between camera and IMU by aligning camera rotation velocities with gyro data
5. Click **"init_opt"** - Initialize the spline optimization. You should see the IMU data plotted below with the spline overlay
6. Click **"optimize"** repeatedly or check **"opt_until_converge"** - Run optimization until convergence

**GUI checkbox settings for optimization:**

| Checkbox | Default | When to use |
|----------|---------|-------------|
| `opt_intr` | OFF | Leave OFF - camera intrinsics are already calibrated |
| `opt_poses` | OFF | Can enable for a few iterations to help initialize the spline before switching to corners |
| `opt_corners` | ON | **Should be ON by default** - this is the primary optimization mode |
| `opt_cam_time_offset` | OFF | Enable only **after** the main optimization has converged, for refinement |
| `opt_imu_scale` | OFF | Enable only **after** the main optimization has converged, for refinement |
| `opt_mocap` | OFF | Only if you have Mocap data (not applicable for most setups) |

**Recommended optimization sequence:**
1. With defaults (`opt_corners` ON, everything else OFF), click **"optimize"** or check **"opt_until_converge"** and wait for convergence
2. Optionally enable `opt_cam_time_offset` and run a few more iterations for time offset refinement
3. Optionally enable `opt_imu_scale` and run a few more iterations for IMU scale/misalignment refinement
4. Click **"save_calib"** to overwrite `calibration.json` with the updated calibration including IMU extrinsics

### Step 5: Evaluate IMU Calibration Results

**In the GUI plots:**
- `show_accel` / `show_gyro` - Toggle accelerometer/gyroscope data display
- `show_data` (dashed lines) - Raw IMU measurements
- `show_spline` (solid lines) - Optimized spline fit
- The solid spline lines should closely follow the dashed raw data lines when calibration is good

**In the console output:**
- `T_i_c0`, `T_i_c1`, `T_i_c2` - The IMU-to-camera transforms (should match your physical mounting)
- `g` vector - Should have norm close to **9.81 m/s²**
- `accel_bias` and `gyro_bias` - Should be small values
- Error should decrease and converge

**If optimization doesn't converge:**

1. **g norm ~1.0 instead of ~9.81, accel_bias absorbing gravity:** Your recording has no IMU motion. You likely moved the AprilGrid board instead of the camera. Record a new dataset where you **move the camera rig** in front of a stationary board.
2. **Reprojection error >100 pixels:** The `init_cam_imu` step failed (degenerate angular velocity matching). This happens when the recording has near-zero angular velocity. Record with dynamic rotational motion.
3. Try enabling `opt_poses` for a few iterations first, then switch to `opt_corners`
4. Verify the AprilGrid is visible and corners are detected in the recording
5. Check that the camera calibration (`calibration.json`) is good (low reprojection error)
6. Do NOT inflate `gyro-noise-std` above 0.002 - this de-weights gyro data and prevents rotation estimation

---

## Troubleshooting

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "Initialization failed for this frame" | Normal - DS model needs good corners | Falls back to pinhole, calibration continues |
| High reprojection error (>2px) | Wrong camera model or AprilGrid config | Try `pinhole-radtan8`, verify measurements |
| "Pinhole fallback" warning | DS initialization failed | Normal for low-distortion lenses |
| Segfault during pose computation | TBB alignment issue | Fixed in this fork |
| No corners detected | Bad lighting, blur, wrong AprilGrid config | Check image quality, verify tagSize |

### Verifying Camera Synchronization

Check if your cameras are hardware synchronized:

```bash
# This fork includes timestamp checking
# In the output, look for:
# "Synchronized frames (all 3 cameras): 625/625 (tolerance: 5ms)"
# Max diff should be 0.000 ms for hardware-synced cameras
```

---

## Integration with DepthAI

### depthai-ros Integration

This calibration output is designed to work with:
- `depthai-ros`: <https://github.com/luxonis/depthai-ros>
- `depthai-core` BasaltVIO: See `depthai-core/src/basalt/BasaltVIO.cpp`

### Example VIO Usage (planned)

```cpp
// In your depthai-ros node
// Load calibration from basalt output
auto calibration = loadBasaltCalibration("calibration.json");

// Initialize BasaltVIO
BasaltVIO vio(calibration);

// Process frames
vio.processFrame(left_image, right_image, imu_data);
```

---

## Documentation

- [Calibration Guide](doc/Calibration.md)
- [VIO and Mapping](doc/VioMapping.md)
- [Visual Odometry (no IMU)](doc/Vo.md)
- [Simulation Tools](doc/Simulation.md)
- [RealSense T265 Tutorial](doc/Realsense.md)

---

## Related Publications

Please cite the original authors if you use this software:

**Visual-Inertial Odometry:**
```bibtex
@article{usenko2019visual,
  title={Visual-Inertial Mapping with Non-Linear Factor Recovery},
  author={Usenko, Vladyslav and Demmel, Nikolaus and Schubert, David and St{\"u}ckler, J{\"o}rg and Cremers, Daniel},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  year={2019},
  doi={10.1109/LRA.2019.2961227}
}
```

**Double Sphere Camera Model:**
```bibtex
@inproceedings{usenko2018double,
  title={The Double Sphere Camera Model},
  author={Usenko, Vladyslav and Demmel, Nikolaus and Cremers, Daniel},
  booktitle={2018 International Conference on 3D Vision (3DV)},
  year={2018},
  doi={10.1109/3DV.2018.00069}
}
```

---

## License

This project is licensed under the **BSD 3-Clause License** - see the [LICENSE](LICENSE) file.

**Note:** The core algorithms and implementation are the work of the original Basalt authors. This fork only adds ROS2 compatibility modifications.

---

## Acknowledgments

- **Original Basalt Authors**: Vladyslav Usenko, Nikolaus Demmel, David Schubert, Christiane Sommer, and Daniel Cremers at TUM
- **Granite Fork**: Some improvements ported from [DLR-RM/granite](https://github.com/DLR-RM/granite) (MIT license)
- **pinhole-radtan8 Model**: Mateo de Mayo at Collabora Ltd.
- **ROS2 Port**: Community contribution for ROS2 Jazzy compatibility

---

## Contributing

Contributions are welcome! Please:
1. Fork this repository
2. Create a feature branch
3. Submit a pull request

For major changes, please open an issue first to discuss the proposed changes.
