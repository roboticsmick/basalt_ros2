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
  --dataset-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_record/stereo_rgb_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results \
  --cam-types pinhole-radtan8 pinhole-radtan8 pinhole-radtan8
```

**Wide-angle fisheye cameras:**
```bash
./basalt_calibrate \
  --dataset-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_record/stereo_rgb_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results \
  --cam-types ds ds ds
```

**Mixed setup (fisheye RGB + standard stereo):**
```bash
./basalt_calibrate \
  --dataset-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_record/stereo_rgb_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results \
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

### Common Mistake: tagSpacing

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

## Calibration Workflow

The full calibration pipeline has 4 steps using 2 separate recordings. The stereo + IMU calibration must be done first (requires high FPS), then the RGB camera is calibrated relative to the stereo pair (low FPS is fine).

For recording instructions, see the [depthai-ros README](https://github.com/roboticsmick/depthai-ros).

**Path convention:** This guide uses `$DEV_HOME` to refer to your development workspace root. Set it in your `~/.bashrc`:

```bash
export DEV_HOME="/path/to/your/workspace"  # e.g. ~/ or /media/user/nvme
```

**IMPORTANT:** The `--result-path` for camera calibration and IMU calibration **must match**, and the number of cameras in the calibration file must match the recording. Using a 3-camera `calibration.json` with a 2-camera recording will cause the IMU optimizer to apply wrong intrinsics to wrong cameras and fail silently.

### Overview

```text
Recording 1: Stereo + IMU (30 fps, stereo-only driver)
  ├── Step 1: Stereo camera calibration  → stereo_imu_calibration_results/calibration.json
  └── Step 2: IMU calibration            → stereo_imu_calibration_results/calibration.json (updated)

Recording 2: RGB + Stereo + IMU (~4.5 fps, 3-camera sync driver)
  └── Step 3: 3-camera calibration       → stereo_rgb_imu_calibration_results/calibration.json

Step 4: Merge stereo+IMU calibration with RGB extrinsics → merged_calibration_results/calibration.json
```

### Create AprilGrid Config

Create the AprilGrid configuration file (measure your own grid — see [AprilGrid Configuration](#aprilgrid-configuration)):

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

### Step 1: Stereo Camera Calibration

Using Recording 1 (stereo + IMU at 30 fps). Mount the **AprilGrid on a wall** and **move the camera rig** dynamically in front of it.

```bash
cd $DEV_HOME/basalt_ros2/build
source /opt/ros/jazzy/setup.bash

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

**Good calibration:** mean reprojection error < 1.0 pixel (ideal < 0.5).

### Step 2: IMU Calibration

Using the **same Recording 1** and the **same `--result-path`** as Step 1. The stereo `calibration.json` is loaded automatically.

#### IMU Noise Parameters

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
| BMI270 (OAK-FFC-3P) | 0.0005 | 0.02 | 0.0001 | 0.001 |
| BMI160 (TUM-VI/EuRoC) | 0.000282 | 0.016 | 0.0001 | 0.001 |
| Snapdragon (UZH-FPV) | 0.05 | 0.1 | 4e-5 | 0.002 |

**Warning:** Setting `gyro-noise-std` too high (e.g. >0.002) will de-weight gyroscope measurements and prevent the optimizer from constraining rotation dynamics.

#### Run

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

#### In the GUI

Follow the buttons **in order from left to right**:

1. Click **"load_dataset"** → **"detect_corners"** → **"init_cam_poses"** → **"init_cam_imu"** → **"init_opt"**
2. Check **"opt_until_converge"** — wait for convergence
3. Optionally enable `opt_cam_time_offset` and `opt_imu_scale` for refinement (see table below)
4. Click **"save_calib"** — updates `calibration.json` with IMU extrinsics

**Optimization checkboxes:**

| Checkbox               | Default | When to use                                          |
|------------------------|---------|------------------------------------------------------|
| `opt_intr`             | OFF     | Leave OFF — camera intrinsics are already calibrated |
| `opt_corners`          | ON      | Primary optimization mode                            |
| `opt_cam_time_offset`  | OFF     | Enable after main convergence for refinement         |
| `opt_imu_scale`        | OFF     | Enable after main convergence for refinement         |

#### Evaluate

- **g vector norm** should be close to **9.81 m/s²**
- **Mean reprojection error** should be < 2 pixels (ideal < 1)
- **Spline plots** (solid lines) should closely follow raw IMU data (dashed lines)

**If optimization doesn't converge:**

1. **g norm ~1.0, accel_bias absorbing gravity:** Recording has no IMU motion — you moved the board instead of the camera
2. **Reprojection error >100 px:** `init_cam_imu` failed — recording has insufficient rotational motion
3. **Optimizer stuck, wrong camera intrinsics:** Check that `--result-path` points to the stereo calibration, not a 3-camera calibration. The number of cameras in `calibration.json` must match the recording
4. Do NOT inflate `gyro-noise-std` above 0.002

### Step 3: 3-Camera Calibration (RGB Extrinsics)

Using Recording 2 (all 3 cameras + IMU at ~4.5 fps). This determines the RGB camera's position relative to the stereo pair. Mount camera on tripod, move the **AprilGrid board** slowly in front of the cameras.

Use `ds` (double sphere) for the wide-angle IMX577 RGB camera (113° HFOV), `pinhole-radtan8` for the OV9782 stereo cameras (75° HFOV):

```bash
./basalt_calibrate \
  --dataset-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_record/stereo_rgb_imu_calibration_record_0.mcap \
  --dataset-type mcap \
  --aprilgrid $DEV_HOME/basalt_calibration/aprilgrid.json \
  --result-path $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results \
  --cam-types pinhole-radtan8 ds pinhole-radtan8
```

**In the GUI:** Same workflow as Step 1 (init_cam_intr → init_cam_poses → init_cam_extr → init_opt → opt_until_converge → save_calib).

This produces `stereo_rgb_imu_calibration_results/calibration.json` with all 3 camera intrinsics and extrinsics in a common reference frame.

### Step 4: Merge Calibrations

The final step combines the stereo+IMU calibration (Step 2) with the RGB extrinsics (Step 3) into a single calibration file. This computes T_imu_rgb by chaining the transforms:

```text
T_left_rgb = T_imu_left_3cam⁻¹ × T_imu_rgb_3cam     (from 3-camera calibration)
T_imu_rgb  = T_imu_left_stereo × T_left_rgb           (into stereo+IMU frame)
```

```bash
python3 $DEV_HOME/basalt_ros2/scripts/merge_calibrations.py \
  --stereo-imu $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \
  --three-cam $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results/calibration.json \
  --output $DEV_HOME/basalt_calibration/merged_calibration_results/calibration.json
```

The script validates that the left camera intrinsics agree between both calibrations (sanity check), then produces a merged file with:

| Index | Camera       | Source                                             |
|-------|--------------|----------------------------------------------------|
| cam0  | Left stereo  | Intrinsics + T_imu_cam from stereo+IMU (Step 2)    |
| cam1  | Right stereo | Intrinsics + T_imu_cam from stereo+IMU (Step 2)    |
| cam2  | RGB          | Intrinsics from 3-cam (Step 3), T_imu_rgb computed |

IMU parameters (bias, noise, time offset) come from the stereo+IMU calibration.

**Important:** For Basalt VIO, use the 2-camera `stereo_imu_calibration_results/calibration.json` directly. The merged 3-camera file is for the photogrammetry pipeline.

### Use Case: Photogrammetry Pipeline

```text
Runtime:
  Stereo + IMU → Basalt VIO → continuous T_world_imu at stereo rate
                                    ↓
  T_world_rgb = T_world_imu × T_imu_rgb → interpolate to RGB timestamps
                                    ↓
  Stereo depth + T_left_rgb → project depth onto RGB frame
                                    ↓
  RGB images + accurate poses + depth → photogrammetry / SFM / point cloud
```

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

- `depthai-ros` (OAK-FFC-3P fork): <https://github.com/roboticsmick/depthai-ros>
- `depthai-core` BasaltVIO: See `depthai-core/src/basalt/BasaltVIO.cpp`

For recording instructions, driver configs, and calibration-to-DepthAI conversion, see the [depthai-ros README](https://github.com/roboticsmick/depthai-ros).

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
