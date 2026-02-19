#!/usr/bin/env python3
#
# Merges a stereo+IMU calibration with a 3-camera calibration to produce
# a single calibration file containing all 3 cameras + IMU parameters.
#
# The stereo+IMU calibration provides:
#   - Accurate stereo intrinsics (from high-FPS recording)
#   - IMU parameters (bias, noise, time offset)
#   - T_imu_left, T_imu_right
#
# The 3-camera calibration provides:
#   - RGB intrinsics
#   - Relative transform between left stereo and RGB cameras
#
# The merge computes T_imu_rgb by chaining:
#   T_left_rgb = T_imu_left_3cam^{-1} * T_imu_rgb_3cam
#   T_imu_rgb  = T_imu_left_stereo * T_left_rgb
#
# Output camera ordering:
#   cam0: left stereo  (intrinsics + T_imu_cam from stereo+IMU)
#   cam1: right stereo (intrinsics + T_imu_cam from stereo+IMU)
#   cam2: RGB          (intrinsics from 3-cam, T_imu_cam computed)
#

import argparse
import copy
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def pose_to_matrix(pose):
    """Convert {px,py,pz,qx,qy,qz,qw} dict to 4x4 homogeneous matrix."""
    t = np.array([pose['px'], pose['py'], pose['pz']])
    q = [pose['qx'], pose['qy'], pose['qz'], pose['qw']]
    R = Rotation.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix to {px,py,pz,qx,qy,qz,qw} dict."""
    t = T[:3, 3]
    q = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    return {
        'px': float(t[0]),
        'py': float(t[1]),
        'pz': float(t[2]),
        'qx': float(q[0]),
        'qy': float(q[1]),
        'qz': float(q[2]),
        'qw': float(q[3]),
    }


def validate_matching_camera(stereo_intr, three_cam_intr,
                             stereo_res, three_cam_res, label):
    """Warn if the left camera doesn't match between calibrations."""
    if stereo_res != three_cam_res:
        print(f'WARNING: {label} resolution mismatch: '
              f'stereo={stereo_res}, 3-cam={three_cam_res}')
    if stereo_intr['camera_type'] != three_cam_intr['camera_type']:
        print(f'WARNING: {label} camera type mismatch: '
              f'stereo={stereo_intr["camera_type"]}, '
              f'3-cam={three_cam_intr["camera_type"]}')

    # Compare focal lengths as a sanity check
    s_fx = stereo_intr['intrinsics']['fx']
    t_fx = three_cam_intr['intrinsics']['fx']
    rel_diff = abs(s_fx - t_fx) / s_fx * 100.0
    if rel_diff > 5.0:
        print(f'WARNING: {label} fx differs by {rel_diff:.1f}%: '
              f'stereo={s_fx:.2f}, 3-cam={t_fx:.2f}')
    else:
        print(f'  {label} fx agreement: {rel_diff:.2f}% '
              f'(stereo={s_fx:.2f}, 3-cam={t_fx:.2f})')


def main():
    parser = argparse.ArgumentParser(
        description='Merge stereo+IMU and 3-camera calibrations.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Camera index defaults (OAK-FFC-3P):
  Stereo+IMU:  cam0=left, cam1=right
  3-camera:    cam0=left, cam1=RGB, cam2=right

Example:
  python3 merge_calibrations.py \\
    --stereo-imu $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \\
    --three-cam $DEV_HOME/basalt_calibration/stereo_rgb_imu_calibration_results/calibration.json \\
    --output $DEV_HOME/basalt_calibration/merged_calibration_results/calibration.json
        ''')

    parser.add_argument('--stereo-imu', required=True,
                        help='Path to stereo+IMU calibration.json (from Steps 1-2)')
    parser.add_argument('--three-cam', required=True,
                        help='Path to 3-camera calibration.json (from Step 3)')
    parser.add_argument('--output', required=True,
                        help='Path to write merged calibration.json')

    # Camera index overrides (defaults match OAK-FFC-3P ordering)
    parser.add_argument('--stereo-left-idx', type=int, default=0,
                        help='Left camera index in stereo+IMU calib (default: 0)')
    parser.add_argument('--stereo-right-idx', type=int, default=1,
                        help='Right camera index in stereo+IMU calib (default: 1)')
    parser.add_argument('--three-cam-left-idx', type=int, default=0,
                        help='Left camera index in 3-cam calib (default: 0)')
    parser.add_argument('--three-cam-rgb-idx', type=int, default=1,
                        help='RGB camera index in 3-cam calib (default: 1)')

    args = parser.parse_args()

    # Load calibrations
    with open(args.stereo_imu, 'r') as f:
        stereo = json.load(f)['value0']
    with open(args.three_cam, 'r') as f:
        three_cam = json.load(f)['value0']

    si_left = args.stereo_left_idx
    si_right = args.stereo_right_idx
    tc_left = args.three_cam_left_idx
    tc_rgb = args.three_cam_rgb_idx

    print('=== Merge Calibrations ===\n')
    print(f'Stereo+IMU: {len(stereo["T_imu_cam"])} cameras')
    print(f'3-camera:   {len(three_cam["T_imu_cam"])} cameras\n')

    # Validate: left camera should roughly agree between both calibrations
    print('Sanity checks:')
    validate_matching_camera(
        stereo['intrinsics'][si_left], three_cam['intrinsics'][tc_left],
        stereo['resolution'][si_left], three_cam['resolution'][tc_left],
        'Left camera')

    # Compute T_left_rgb from 3-camera calibration
    # T_left_rgb = T_imu_left_3cam^{-1} * T_imu_rgb_3cam
    T_imu_left_3cam = pose_to_matrix(three_cam['T_imu_cam'][tc_left])
    T_imu_rgb_3cam = pose_to_matrix(three_cam['T_imu_cam'][tc_rgb])
    T_left_rgb = np.linalg.inv(T_imu_left_3cam) @ T_imu_rgb_3cam

    # Compute T_imu_rgb in stereo+IMU frame
    # T_imu_rgb = T_imu_left_stereo * T_left_rgb
    T_imu_left_stereo = pose_to_matrix(stereo['T_imu_cam'][si_left])
    T_imu_rgb_merged = T_imu_left_stereo @ T_left_rgb

    # Report the transforms
    t_left_rgb = T_left_rgb[:3, 3]
    baseline = np.linalg.norm(t_left_rgb)
    print(f'\nT_left_rgb translation: [{t_left_rgb[0]:.4f}, '
          f'{t_left_rgb[1]:.4f}, {t_left_rgb[2]:.4f}] m')
    print(f'Left-to-RGB baseline: {baseline * 1000:.1f} mm')

    T_imu_left = pose_to_matrix(stereo['T_imu_cam'][si_left])
    T_imu_right = pose_to_matrix(stereo['T_imu_cam'][si_right])
    T_left_right = np.linalg.inv(T_imu_left) @ T_imu_right
    stereo_baseline = np.linalg.norm(T_left_right[:3, 3])
    print(f'Stereo baseline: {stereo_baseline * 1000:.1f} mm')

    # Build merged calibration
    merged = {
        'value0': {
            # cam0=left, cam1=right (from stereo+IMU), cam2=RGB (computed)
            'T_imu_cam': [
                stereo['T_imu_cam'][si_left],
                stereo['T_imu_cam'][si_right],
                matrix_to_pose(T_imu_rgb_merged),
            ],
            'intrinsics': [
                stereo['intrinsics'][si_left],
                stereo['intrinsics'][si_right],
                three_cam['intrinsics'][tc_rgb],
            ],
            'resolution': [
                stereo['resolution'][si_left],
                stereo['resolution'][si_right],
                three_cam['resolution'][tc_rgb],
            ],
            # All IMU params from stereo+IMU calibration
            'calib_accel_bias': stereo['calib_accel_bias'],
            'calib_gyro_bias': stereo['calib_gyro_bias'],
            'imu_update_rate': stereo['imu_update_rate'],
            'accel_noise_std': stereo['accel_noise_std'],
            'gyro_noise_std': stereo['gyro_noise_std'],
            'accel_bias_std': stereo['accel_bias_std'],
            'gyro_bias_std': stereo['gyro_bias_std'],
            'cam_time_offset_ns': stereo['cam_time_offset_ns'],
            'vignette': stereo.get('vignette', []),
        }
    }

    # Write output
    import os
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(merged, f, indent=4)

    print(f'\nMerged calibration written to: {args.output}')
    print(f'  cam0: left stereo  ({stereo["intrinsics"][si_left]["camera_type"]}, '
          f'{stereo["resolution"][si_left][0]}x{stereo["resolution"][si_left][1]})')
    print(f'  cam1: right stereo ({stereo["intrinsics"][si_right]["camera_type"]}, '
          f'{stereo["resolution"][si_right][0]}x{stereo["resolution"][si_right][1]})')
    print(f'  cam2: RGB          ({three_cam["intrinsics"][tc_rgb]["camera_type"]}, '
          f'{three_cam["resolution"][tc_rgb][0]}x{three_cam["resolution"][tc_rgb][1]})')
    print(f'  IMU:  {stereo["imu_update_rate"]:.1f} Hz')


if __name__ == '__main__':
    main()
