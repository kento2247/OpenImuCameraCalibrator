#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np


def load_json(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def quat_to_rotmat(q: dict) -> np.ndarray:
    w = float(q['w'])
    x = float(q['x'])
    y = float(q['y'])
    z = float(q['z'])
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        raise ValueError('Quaternion norm is zero.')
    w /= n
    x /= n
    y /= n
    z /= n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def build_transform(imu_cam_result: dict, apply_gopro_permutation: bool) -> np.ndarray:
    rot = quat_to_rotmat(imu_cam_result['q_i_c'])
    trans = np.array([
        float(imu_cam_result['t_i_c']['x']),
        float(imu_cam_result['t_i_c']['y']),
        float(imu_cam_result['t_i_c']['z']),
    ], dtype=float)

    t_i_c = np.eye(4, dtype=float)
    t_i_c[:3, :3] = rot
    t_i_c[:3, 3] = trans

    if not apply_gopro_permutation:
        return t_i_c

    perm = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return perm @ t_i_c


def infer_camera_type(camera_model: str) -> str:
    mapping = {
        'FISHEYE': 'KannalaBrandt8',
        'PINHOLE': 'PinHole',
        'PINHOLE_RADIAL_TANGENTIAL': 'PinHole',
        'DIVISION_UNDISTORTION': 'PinHole',
        'DOUBLE_SPHERE': 'KannalaBrandt8',
        'EXTENDED_UNIFIED': 'KannalaBrandt8',
    }
    if camera_model not in mapping:
        raise ValueError(f'Unsupported intrinsic_type: {camera_model}')
    return mapping[camera_model]


def format_matrix_data(mat: np.ndarray) -> str:
    flat = mat.reshape(-1)
    return ', '.join(f'{v:.12g}' for v in flat)


def generate_yaml(camera_json: dict, imu_cam_json: dict, args: argparse.Namespace) -> str:
    intr = camera_json['intrinsics']
    fx = float(intr['focal_length'])
    fy = fx * float(intr.get('aspect_ratio', 1.0))
    cx = float(intr['principal_pt_x'])
    cy = float(intr['principal_pt_y'])
    width = int(camera_json['image_width'])
    height = int(camera_json['image_height'])
    fps = float(camera_json['fps'])
    camera_type = infer_camera_type(camera_json['intrinsic_type'])
    t_b_c1 = build_transform(imu_cam_json, apply_gopro_permutation=not args.no_gopro_axis_permutation)

    lines = [
        '%YAML:1.0',
        '',
        '#--------------------------------------------------------------------------------------------',
        '# Camera Parameters',
        '#--------------------------------------------------------------------------------------------',
        'File.version: "1.0"',
        f'Camera.type: "{camera_type}"',
        f'Camera1.fx: {fx:.12g}',
        f'Camera1.fy: {fy:.12g}',
        f'Camera1.cx: {cx:.12g}',
        f'Camera1.cy: {cy:.12g}',
        '',
    ]

    if camera_json['intrinsic_type'] == 'FISHEYE':
        lines.extend([
            f"Camera1.k1: {float(intr['radial_distortion_1']):.12g}",
            f"Camera1.k2: {float(intr['radial_distortion_2']):.12g}",
            f"Camera1.k3: {float(intr['radial_distortion_3']):.12g}",
            f"Camera1.k4: {float(intr['radial_distortion_4']):.12g}",
        ])
    elif camera_json['intrinsic_type'] == 'PINHOLE_RADIAL_TANGENTIAL':
        lines.extend([
            f"Camera1.k1: {float(intr['radial_distortion_1']):.12g}",
            f"Camera1.k2: {float(intr['radial_distortion_2']):.12g}",
            f"Camera1.p1: {float(intr['tangential_distortion_1']):.12g}",
            f"Camera1.p2: {float(intr['tangential_distortion_2']):.12g}",
            f"Camera1.k3: {float(intr['radial_distortion_3']):.12g}",
        ])
    else:
        lines.append('# Distortion values were not exported for this intrinsic model.')

    lines.extend([
        '',
        f'Camera.width: {width}',
        f'Camera.height: {height}',
        '',
        f'Camera.fps: {fps:.12g}',
        'Camera.RGB: 1',
        '',
        '# Transformation from body/IMU to camera.',
        'IMU.T_b_c1: !!opencv-matrix',
        '    rows: 4',
        '    cols: 4',
        '    dt: f',
        f'    data: [ {format_matrix_data(t_b_c1)} ]',
        '',
        f'IMU.NoiseGyro: {args.noise_gyro:.12g}',
        f'IMU.NoiseAcc: {args.noise_acc:.12g}',
        f'IMU.GyroWalk: {args.gyro_walk:.12g}',
        f'IMU.AccWalk: {args.acc_walk:.12g}',
        f'IMU.Frequency: {args.imu_frequency:.12g}',
        '',
        '#--------------------------------------------------------------------------------------------',
        '# ORB Parameters',
        '#--------------------------------------------------------------------------------------------',
        f'ORBextractor.nFeatures: {args.orb_nfeatures}',
        f'ORBextractor.scaleFactor: {args.orb_scale_factor:.12g}',
        f'ORBextractor.nLevels: {args.orb_nlevels}',
        f'ORBextractor.iniThFAST: {args.orb_ini_thfast}',
        f'ORBextractor.minThFAST: {args.orb_min_thfast}',
        '',
        f'System.thFarPoints: {args.system_th_far_points:.12g}',
        '',
        '#--------------------------------------------------------------------------------------------',
        '# Viewer Parameters',
        '#--------------------------------------------------------------------------------------------',
        f'Viewer.KeyFrameSize: {args.viewer_keyframe_size:.12g}',
        f'Viewer.KeyFrameLineWidth: {args.viewer_keyframe_line_width:.12g}',
        f'Viewer.GraphLineWidth: {args.viewer_graph_line_width:.12g}',
        f'Viewer.PointSize: {args.viewer_point_size:.12g}',
        f'Viewer.CameraSize: {args.viewer_camera_size:.12g}',
        f'Viewer.CameraLineWidth: {args.viewer_camera_line_width:.12g}',
        f'Viewer.ViewpointX: {args.viewer_viewpoint_x:.12g}',
        f'Viewer.ViewpointY: {args.viewer_viewpoint_y:.12g}',
        f'Viewer.ViewpointZ: {args.viewer_viewpoint_z:.12g}',
        f'Viewer.ViewpointF: {args.viewer_viewpoint_f:.12g}',
        f'Viewer.imageViewScale: {args.viewer_image_view_scale:.12g}',
        '',
    ])
    return '\n'.join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Convert OpenImuCameraCalibrator JSON outputs to an ORB-SLAM3-style YAML file.')
    parser.add_argument('--camera_json', required=True, help='Path to cam_calib_*.json')
    parser.add_argument('--imu_cam_json', required=True, help='Path to cam_imu_calib_result_*.json')
    parser.add_argument('--output', required=True, help='Path to output yaml')
    parser.add_argument('--noise_gyro', type=float, default=0.0015)
    parser.add_argument('--noise_acc', type=float, default=0.017)
    parser.add_argument('--gyro_walk', type=float, default=5.0e-5)
    parser.add_argument('--acc_walk', type=float, default=0.0055)
    parser.add_argument('--imu_frequency', type=float, default=200.0)
    parser.add_argument('--no_gopro_axis_permutation', action='store_true', help='Disable the GoPro axis permutation used in hero9_sample.yaml.')
    parser.add_argument('--orb_nfeatures', type=int, default=1250)
    parser.add_argument('--orb_scale_factor', type=float, default=1.2)
    parser.add_argument('--orb_nlevels', type=int, default=8)
    parser.add_argument('--orb_ini_thfast', type=int, default=20)
    parser.add_argument('--orb_min_thfast', type=int, default=7)
    parser.add_argument('--system_th_far_points', type=float, default=20.0)
    parser.add_argument('--viewer_keyframe_size', type=float, default=0.05)
    parser.add_argument('--viewer_keyframe_line_width', type=float, default=1.0)
    parser.add_argument('--viewer_graph_line_width', type=float, default=0.9)
    parser.add_argument('--viewer_point_size', type=float, default=2.0)
    parser.add_argument('--viewer_camera_size', type=float, default=0.08)
    parser.add_argument('--viewer_camera_line_width', type=float, default=3.0)
    parser.add_argument('--viewer_viewpoint_x', type=float, default=0.0)
    parser.add_argument('--viewer_viewpoint_y', type=float, default=-0.7)
    parser.add_argument('--viewer_viewpoint_z', type=float, default=-3.5)
    parser.add_argument('--viewer_viewpoint_f', type=float, default=500.0)
    parser.add_argument('--viewer_image_view_scale', type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera_json = load_json(args.camera_json)
    imu_cam_json = load_json(args.imu_cam_json)
    yaml_text = generate_yaml(camera_json, imu_cam_json, args)
    output_path = Path(args.output)
    output_path.write_text(yaml_text, encoding='utf-8')
    print(f'Wrote {output_path}')


if __name__ == '__main__':
    main()
