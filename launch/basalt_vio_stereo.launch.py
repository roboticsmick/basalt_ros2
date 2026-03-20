#!/usr/bin/env python3
"""
basalt_vio_stereo.launch.py

Integrated VIO + Host-side Stereo Depth Pipeline using ROS2 Components.

This launch file runs CalibrationPublisher, VisualOdometerNode, and the stereo depth
pipeline (RectifyNode x2 + DisparityNode + PointCloudNode) in a single component_container
with intra-process communication enabled for zero-copy image passing.

The depthai-ros camera driver should be run separately in a different terminal:
  Terminal 1: ros2 launch depthai_ros_driver driver.launch.py \\
    params_file:=src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_stereo_vio.yaml \\
    camera_model:=OAK-FFC-3P

Then run this launch file in a second terminal:
  Terminal 2: ros2 launch basalt_ros2 basalt_vio_stereo.launch.py \\
    calib_path:=<path to calibration.yaml> \\
    config_path:=<path to vio_config.yaml>

Expected output topics:
  /odometry              — Basalt VIO odometry (30 Hz)
  /stereo/disparity     — SGBM disparity image (30 Hz)
  /stereo/points        — 3D point cloud (15-30 Hz, CPU-intensive)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    calib_path_arg = DeclareLaunchArgument(
        "calib_path",
        default_value="",
        description="Path to Basalt calibration file (YAML or JSON)",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Path to Basalt VIO config file",
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/oak/imu/data",
        description="IMU topic (from depthai driver)",
    )
    left_image_topic_arg = DeclareLaunchArgument(
        "left_image_topic",
        default_value="/oak/left/image_raw",
        description="Left camera image topic (from depthai driver)",
    )
    right_image_topic_arg = DeclareLaunchArgument(
        "right_image_topic",
        default_value="/oak/right/image_raw",
        description="Right camera image topic (from depthai driver)",
    )

    # Component container with VIO and calibration publisher
    container = ComposableNodeContainer(
        name="vio_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # 1. Basalt calibration publisher (computes stereo rectification)
            ComposableNode(
                package="basalt_ros2",
                plugin="basalt::CalibrationPublisherNode",
                name="calibration_publisher",
                parameters=[
                    {"calib_path": LaunchConfiguration("calib_path")},
                    {"left_camera_info_topic": "/basalt/left/camera_info"},
                    {"right_camera_info_topic": "/basalt/right/camera_info"},
                    {"publish_interval_hz": 10},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 2. Basalt VIO node
            ComposableNode(
                package="basalt_ros2",
                plugin="basalt::VisualOdometerNode",
                name="visual_odometry_node",
                parameters=[
                    {"calib_path": LaunchConfiguration("calib_path")},
                    {"config_path": LaunchConfiguration("config_path")},
                    {"imu_topic": LaunchConfiguration("imu_topic")},
                    {"left_image_topic": LaunchConfiguration("left_image_topic")},
                    {"right_image_topic": LaunchConfiguration("right_image_topic")},
                    {"publish_cloud": True},
                    {"publish_images": True},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 3. image_proc rectify (left camera)
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_left",
                remappings=[
                    ("image", LaunchConfiguration("left_image_topic")),
                    ("camera_info", "/basalt/left/camera_info"),
                    ("image_rect", "/oak/left/image_rect"),
                    ("camera_info_rect", "/basalt/left/camera_info_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 4. image_proc rectify (right camera)
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_right",
                remappings=[
                    ("image", LaunchConfiguration("right_image_topic")),
                    ("camera_info", "/basalt/right/camera_info"),
                    ("image_rect", "/oak/right/image_rect"),
                    ("camera_info_rect", "/basalt/right/camera_info_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 5. stereo_image_proc disparity (SGBM CPU-based)
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="stereo_disparity",
                parameters=[
                    {"stereo_algorithm": 1},  # 1 = SGBM, 0 = BM
                    {"min_disparity": 0},
                    {"disparity_range": 64},
                    {"block_size": 5},
                    {"speckle_size": 100},
                    {"speckle_range": 4},
                    {"p1": 24},
                    {"p2": 96},
                    {"uniqueness_ratio": 15},
                ],
                remappings=[
                    ("left/image_rect", "/oak/left/image_rect"),
                    ("left/camera_info", "/basalt/left/camera_info_rect"),
                    ("right/image_rect", "/oak/right/image_rect"),
                    ("right/camera_info", "/basalt/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 6. stereo_image_proc point cloud generator
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::PointCloudNode",
                name="stereo_point_cloud",
                parameters=[
                    {"use_color": True},
                    {"queue_size": 5},
                ],
                remappings=[
                    ("left/image_rect_color", "/oak/left/image_rect"),
                    ("right/camera_info", "/basalt/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                    ("points2", "/stereo/points"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        calib_path_arg,
        config_path_arg,
        imu_topic_arg,
        left_image_topic_arg,
        right_image_topic_arg,
        LogInfo(msg="Starting Basalt VIO Pipeline (depthai driver should be running separately)"),
        container,
    ])
