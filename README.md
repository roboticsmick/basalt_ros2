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
> This fork only adds ROS2 Jazzy compatibility and bug fixes for modern toolchains.

![teaser](doc/img/teaser.png)

---

## What's New in This Fork

### ROS2 Jazzy Compatibility (Ubuntu 24.04)

This fork enables Basalt to work with:
- **ROS2 Jazzy** on Ubuntu 24.04
- **MCAP** and **SQLite3** bag formats (native ROS2 bag formats)
- **GCC 13+** (modern C++ compiler compatibility)
- **DepthAI cameras** and other modern camera systems

### Changelog

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

---

## Features

- **Camera Calibration** - Intrinsic and extrinsic calibration using AprilGrid
- **Camera-IMU Calibration** - Joint camera and IMU calibration
- **Visual-Inertial Odometry** - Real-time state estimation
- **Mapping** - Sparse map creation and localization
- **Multiple Camera Models** - Pinhole, Kannala-Brandt (kb4), Double Sphere (ds), Extended Unified (eucm)

### Supported Dataset Formats
- ROS2 MCAP bags (`.mcap`)
- ROS2 SQLite3 bags (`.db3`)
- EuRoC format
- TUM-VI format
- KITTI format

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
- `basalt_calibrate` - Camera calibration tool
- `basalt_calibrate_imu` - Camera-IMU calibration tool
- `basalt_vio` - Visual-Inertial Odometry
- `basalt_mapper` - Mapping tool

---

## Usage

### Camera Calibration with ROS2 Bags

**1. Create an AprilGrid configuration file (`aprilgrid.json`):**

```json
{
    "tagCols": 6,
    "tagRows": 6,
    "tagSize": 0.088,
    "tagSpacing": 0.3
}
```

**2. Record calibration data:**

```bash
# With your camera driver running
ros2 bag record /camera/left/image_raw /camera/right/image_raw /imu/data
```

**3. Run calibration:**

```bash
cd basalt_ros2/build
source /opt/ros/jazzy/setup.bash

./basalt_calibrate \
  --dataset-path /path/to/your/recording.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/calibration_result/ \
  --cam-types ds ds
```

### Camera Model Options

| Model | Flag | Description |
|-------|------|-------------|
| Double Sphere | `ds` | Good for fisheye lenses (recommended) |
| Kannala-Brandt | `kb4` | Alternative fisheye model |
| Pinhole | `pinhole` | Standard pinhole model |
| Extended Unified | `eucm` | Extended unified camera model |

### Example: DepthAI OAK-FFC-3P (3 cameras)

```bash
./basalt_calibrate \
  --dataset-path /path/to/oak_calibration.mcap \
  --dataset-type mcap \
  --aprilgrid /path/to/aprilgrid.json \
  --result-path ~/oak_calib_result/ \
  --cam-types ds ds ds
```

---

## Recording Tips for Best Results

### Camera Synchronization

For multi-camera systems, ensure hardware synchronization is enabled. Example for DepthAI ROS2 driver:

```yaml
# In your camera config yaml
left:
  i_synced: true
  i_fsync_continuous: true
  i_fsync_mode: "OUTPUT"  # Master camera
right:
  i_synced: true
  i_fsync_continuous: true
  i_fsync_mode: "INPUT"   # Slave camera
```

### Calibration Recording Guidelines

1. **Duration**: 30-60 seconds of movement
2. **Motion**: Slow, smooth movements covering all axes
3. **Coverage**: Move the AprilGrid to cover the entire field of view
4. **Distance**: Vary the distance from close to far
5. **Angles**: Include tilted views of the calibration target

---

## Known Issues & Workarounds

| Issue | Workaround |
|-------|------------|
| Low sync rate (< 50%) | Enable hardware FSYNC in camera driver |
| Segfault during pose computation | Fixed in this fork (TBB alignment issue) |
| "Null image" errors | Normal for unsynchronized frames - calibration still works |
| DS initialization fails | Falls back to pinhole - calibration continues |

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
- **ROS2 Port**: Community contribution for ROS2 Jazzy compatibility

---

## Contributing

Contributions are welcome! Please:
1. Fork this repository
2. Create a feature branch
3. Submit a pull request

For major changes, please open an issue first to discuss the proposed changes.
