## Basalt ROS2

**ROS2 Port of Basalt VIO** - Calibration and Visual-Inertial Odometry for ROS2

This is a ROS2-compatible fork of the original [Basalt project](https://gitlab.com/VladyslavUsenko/basalt) by Vladyslav Usenko.
For more information about the original project, see https://vision.in.tum.de/research/vslam/basalt

![teaser](doc/img/teaser.png)

### Features
This project contains tools for:
* **Camera, IMU and motion capture calibration** using ROS2 bag files (MCAP/SQLite3 format)
* Visual-inertial odometry and mapping
* Support for ROS2 Jazzy (Ubuntu 24.04)
* Compatible with modern camera systems (tested with DepthAI v3)

### Original Project
The original Basalt project provides a [header-only library](https://gitlab.com/VladyslavUsenko/basalt-headers) ([Documentation](https://vladyslavusenko.gitlab.io/basalt-headers/)) and supports multiple datasets including TUM-VI, EuRoC, and KITTI.

## Related Publications
Visual-Inertial Odometry and Mapping:
* **Visual-Inertial Mapping with Non-Linear Factor Recovery**, V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, In IEEE Robotics and Automation Letters (RA-L) [[DOI:10.1109/LRA.2019.2961227]](https://doi.org/10.1109/LRA.2019.2961227) [[arXiv:1904.06504]](https://arxiv.org/abs/1904.06504).

Calibration (explains implemented camera models):
* **The Double Sphere Camera Model**, V. Usenko and N. Demmel and D. Cremers, In 2018 International Conference on 3D Vision (3DV), [[DOI:10.1109/3DV.2018.00069]](https://doi.org/10.1109/3DV.2018.00069), [[arXiv:1807.08957]](https://arxiv.org/abs/1807.08957).

Calibration (demonstrates how these tools can be used for dataset calibration):
* **The TUM VI Benchmark for Evaluating Visual-Inertial Odometry**, D. Schubert, T. Goll,  N. Demmel, V. Usenko, J. Stückler, D. Cremers, In 2018 International Conference on Intelligent Robots and Systems (IROS), [[DOI:10.1109/IROS.2018.8593419]](https://doi.org/10.1109/IROS.2018.8593419), [[arXiv:1804.06120]](https://arxiv.org/abs/1804.06120).

Calibration (describes B-spline trajectory representation used in camera-IMU calibration):
* **Efficient Derivative Computation for Cumulative B-Splines on Lie Groups**, C. Sommer, V. Usenko, D. Schubert, N. Demmel, D. Cremers, In 2020 Conference on Computer Vision and Pattern Recognition (CVPR), [[DOI:10.1109/CVPR42600.2020.01116]](https://doi.org/10.1109/CVPR42600.2020.01116), [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860).

Optimization (describes square-root optimization and marginalization used in VIO/VO):
* **Square Root Marginalization for Sliding-Window Bundle Adjustment**, N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, In 2021 International Conference on Computer Vision (ICCV), [[arXiv:2109.02182]](https://arxiv.org/abs/2109.02182)


## Installation

### Prerequisites
- **Ubuntu 24.04 (recommended)** or Ubuntu 22.04
- **ROS2 Jazzy** (for Ubuntu 24.04) or ROS2 Humble (for Ubuntu 22.04)
- GCC 13+ / Clang 15+

### ROS2 Installation
First, install ROS2 Jazzy following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

For Ubuntu 24.04:
```bash
sudo apt update && sudo apt install -y ros-jazzy-desktop
```

### Build from Source

1. **Install system dependencies:**
```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libeigen3-dev libtbb-dev libopencv-dev \
    libfmt-dev libglew-dev libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev
```

2. **Clone this repository:**
```bash
git clone --recursive https://github.com/roboticsmick/basalt_ros2.git
cd basalt_ros2
```

3. **Build the project:**
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Create build directory and build
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS2=ON -DUSE_ROS1=OFF
make -j$(nproc)
```

4. **Built executables will be in the `build/` directory:**
   - `basalt_calibrate` - Camera calibration
   - `basalt_calibrate_imu` - Camera-IMU calibration
   - `basalt_vio` - Visual-Inertial Odometry
   - `basalt_mapper` - Mapping tool

### Installation Notes
- This ROS2 port **does not** use APT packages - build from source only
- The build has been tested with **GCC 13.3** on Ubuntu 24.04
- Submodule fixes for Pangolin and Eigen are included for GCC 13+ compatibility
- ROS2 Jazzy MCAP and SQLite3 bag formats are fully supported

## Usage
* [Camera, IMU and Mocap calibration. (TUM-VI, Euroc, UZH-FPV and Kalibr datasets)](doc/Calibration.md)
* [Visual-inertial odometry and mapping. (TUM-VI and Euroc datasets)](doc/VioMapping.md)
* [Visual odometry (no IMU). (KITTI dataset)](doc/Vo.md)
* [Simulation tools to test different components of the system.](doc/Simulation.md)
* [Batch evaluation tutorial (ICCV'21 experiments)](doc/BatchEvaluation.md)

## Device support
* [Tutorial on Camera-IMU and Motion capture calibration with Realsense T265.](doc/Realsense.md)

## Development
* [Development environment setup.](doc/DevSetup.md)

## Licence
The code is provided under a BSD 3-clause license. See the LICENSE file for details.
Note also the different licenses of thirdparty submodules.

Some improvements are ported back from the fork
[granite](https://github.com/DLR-RM/granite) (MIT license).
