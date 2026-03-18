#!/usr/bin/env python3
"""
basalt_vio_stereo.launch.py

Launch file for integrated VIO + host-side stereo depth pipeline.

Loads all nodes into a single component_container with intra-process communication
enabled for zero-copy image passing between the camera driver, VIO, rectification,
disparity, and point cloud nodes.

Usage:
    ros2 launch basalt_ros2 basalt_vio_stereo.launch.py \\
      params_file:=<path to oak_ffc_3p_stereo_vio.yaml> \\
      calib_path:=<path to calibration.yaml> \\
      config_path:=<path to vio_config.yaml>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Path to depthai-ros driver YAML config (e.g., oak_ffc_3p_stereo_vio.yaml)",
    )
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
        description="IMU topic",
    )
    left_image_topic_arg = DeclareLaunchArgument(
        "left_image_topic",
        default_value="/oak/left/image_raw",
        description="Left camera image topic",
    )
    right_image_topic_arg = DeclareLaunchArgument(
        "right_image_topic",
        default_value="/oak/right/image_raw",
        description="Right camera image topic",
    )

    # SGBM disparity parameters
    min_disparity_arg = DeclareLaunchArgument(
        "min_disparity",
        default_value="0",
        description="Minimum disparity for SGBM",
    )
    disparity_range_arg = DeclareLaunchArgument(
        "disparity_range",
        default_value="64",
        description="Disparity range for SGBM (DISPARITY_64 or DISPARITY_96)",
    )
    block_size_arg = DeclareLaunchArgument(
        "block_size",
        default_value="5",
        description="SAD window size for SGBM",
    )

    # Component container
    container = ComposableNodeContainer(
        name="vio_stereo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # 1. depthai-ros camera driver
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Driver",
                name="depthai_driver",
                parameters=[LaunchConfiguration("params_file")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 2. Basalt calibration publisher (computes stereo rectification)
            ComposableNode(
                package="basalt_ros2",
                plugin="basalt::CalibrationPublisherNode",
                name="calibration_publisher",
                parameters=[
                    {"calib_path": LaunchConfiguration("calib_path")},
                    {"left_camera_info_topic": "/oak/left/camera_info"},
                    {"right_camera_info_topic": "/oak/right/camera_info"},
                    {"publish_interval_hz": 10},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 3. Basalt VIO node
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
            # 4. image_proc rectify (left camera)
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_left",
                remappings=[
                    ("image", LaunchConfiguration("left_image_topic")),
                    ("camera_info", "/oak/left/camera_info"),
                    ("image_rect", "/oak/left/image_rect"),
                    ("camera_info_rect", "/oak/left/camera_info_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 5. image_proc rectify (right camera)
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_right",
                remappings=[
                    ("image", LaunchConfiguration("right_image_topic")),
                    ("camera_info", "/oak/right/camera_info"),
                    ("image_rect", "/oak/right/image_rect"),
                    ("camera_info_rect", "/oak/right/camera_info_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 6. stereo_image_proc disparity (SGBM CPU)
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="stereo_disparity",
                parameters=[
                    {"stereo_algorithm": 1},  # 1 = SGBM, 0 = BM
                    {"min_disparity": LaunchConfiguration("min_disparity")},
                    {"disparity_range": LaunchConfiguration("disparity_range")},
                    {"block_size": LaunchConfiguration("block_size")},
                    {"speckle_size": 50},
                    {"speckle_range": 32},
                    {"p1": 24},
                    {"p2": 96},
                    {"correlation_window_size": 5},
                    {"uniqueness_ratio": 15},
                    {"texture_threshold": 10},
                    {"prefilter_size": 1},
                    {"prefilter_cap": 32},
                ],
                remappings=[
                    ("left/image_rect", "/oak/left/image_rect"),
                    ("left/camera_info", "/oak/left/camera_info_rect"),
                    ("right/image_rect", "/oak/right/image_rect"),
                    ("right/camera_info", "/oak/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 7. depth_image_proc disparity to depth converter
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::DisparityToDepthNode",
                name="disparity_to_depth",
                remappings=[
                    ("disparity", "/stereo/disparity"),
                    ("depth", "/stereo/depth"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 8. stereo_image_proc point cloud generator
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
                    ("right/camera_info", "/oak/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                    ("points2", "/stereo/points"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        calib_path_arg,
        config_path_arg,
        imu_topic_arg,
        left_image_topic_arg,
        right_image_topic_arg,
        min_disparity_arg,
        disparity_range_arg,
        block_size_arg,
        LogInfo(msg="Starting Basalt VIO + Host-Side Stereo Depth Pipeline"),
        container,
    ])
