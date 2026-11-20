"""Convert measured Kalibr files to ORB-SLAM3 stereo-inertial settings."""

import argparse
import math
from pathlib import Path
import sys

import yaml


NOISE_KEYS = {
    "IMU.NoiseGyro": "gyroscope_noise_density",
    "IMU.NoiseAcc": "accelerometer_noise_density",
    "IMU.GyroWalk": "gyroscope_random_walk",
    "IMU.AccWalk": "accelerometer_random_walk",
    "IMU.Frequency": "update_rate",
}


def scalar(value: object) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"expected a finite number, got {value!r}")
    return float(value)


def vector(values: list, size: int) -> list[float]:
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f"expected a list of {size} numbers")
    return [scalar(value) for value in values]


def inverse_pose(rows: list) -> list[list[float]]:
    if not isinstance(rows, list) or len(rows) != 4:
        raise ValueError("expected a 4x4 homogeneous transform")
    matrix = [vector(row, 4) for row in rows]
    if matrix[3] != [0.0, 0.0, 0.0, 1.0]:
        raise ValueError("invalid homogeneous bottom row")
    rotation = [row[:3] for row in matrix[:3]]
    for i, row in enumerate(rotation):
        for j, other in enumerate(rotation):
            if abs(sum(a * b for a, b in zip(row, other)) - (i == j)) > 1e-5:
                raise ValueError("rotation is not orthonormal")
    a, b, c = rotation
    cross = (a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0])
    if sum(x * y for x, y in zip(cross, c)) < 0:
        raise ValueError("rotation has negative determinant")
    translation = [row[3] for row in matrix[:3]]
    return [
        [*column, -sum(r * t for r, t in zip(column, translation))]
        for column in zip(*rotation)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def matrix_yaml(key: str, matrix: list[list[float]]) -> str:
    values = ", ".join(repr(number) for row in matrix for number in row)
    return f"{key}: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [ {values} ]"


def camera_fields(key: str, camera: dict) -> dict[str, float]:
    if (camera["camera_model"], camera["distortion_model"]) != ("pinhole", "radtan"):
        raise ValueError(f"{key} must use Kalibr pinhole-radtan")
    intrinsics = vector(camera["intrinsics"], 4)
    if min(intrinsics[:2]) <= 0:
        raise ValueError(f"{key} focal lengths must be positive")
    distortion = vector(camera["distortion_coeffs"], 4)
    keys = ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2")
    return {f"{key}.{name}": value for name, value in zip(keys, intrinsics + distortion)}


def build_settings(chain: dict, imu: dict, fps: int) -> str:
    left, right = chain["cam0"], chain["cam1"]
    width, height = left["resolution"]
    if (left["resolution"] != right["resolution"]
            or any(type(n) is not int or n <= 0 for n in (width, height, fps))):
        raise ValueError("stereo resolution and camera rate must be valid")
    right_to_left = inverse_pose(right["T_cn_cnm1"])
    if right_to_left[0][3] <= max(abs(right_to_left[1][3]), abs(right_to_left[2][3])):
        raise ValueError("expected the right camera on the positive X side of the left camera")
    left_to_imu = inverse_pose(left["T_cam_imu"])
    time_shift = scalar(left["timeshift_cam_imu"])
    noise = {orb: scalar(imu[kalibr]) for orb, kalibr in NOISE_KEYS.items()}
    if min(noise.values()) <= 0:
        raise ValueError("IMU noise, random walk and rate must all be positive")
    fields = {
        "Camera.width": width, "Camera.height": height, "Camera.fps": fps, "Camera.RGB": 0,
        **camera_fields("Camera1", left), **camera_fields("Camera2", right),
        "Stereo.ThDepth": 40.0, **noise, "IMU.InsertKFsWhenLost": 0,
        "ORBextractor.nFeatures": 1250, "ORBextractor.scaleFactor": 1.2,
        "ORBextractor.nLevels": 8, "ORBextractor.iniThFAST": 20, "ORBextractor.minThFAST": 7,
        "Viewer.KeyFrameSize": 0.05, "Viewer.KeyFrameLineWidth": 1.0,
        "Viewer.GraphLineWidth": 0.9, "Viewer.PointSize": 2.0, "Viewer.CameraSize": 0.08,
        "Viewer.CameraLineWidth": 3.0, "Viewer.ViewpointX": 0.0, "Viewer.ViewpointY": -0.7,
        "Viewer.ViewpointZ": -3.5, "Viewer.ViewpointF": 500.0,
    }
    lines = ['%YAML:1.0', 'File.version: "1.0"', 'Camera.type: "PinHole"']
    lines.extend(f"{key}: {value!r}" for key, value in fields.items())
    lines.append(matrix_yaml("Stereo.T_c1_c2", right_to_left))
    lines.append(matrix_yaml("IMU.T_b_c1", left_to_imu))
    lines.append("# ROS adapter uses t_imu = t_left_camera + shift for IMU lookup.")
    lines.append(f"Calibration.CameraToImuTimeShiftSec: {time_shift!r}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("camchain", type=Path)
    parser.add_argument("imu", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--camera-fps", required=True, type=int)
    args = parser.parse_args()
    try:
        chain = yaml.safe_load(args.camchain.read_text(encoding="utf-8"))
        imu = yaml.safe_load(args.imu.read_text(encoding="utf-8"))
        settings = build_settings(chain, imu, args.camera_fps)
        with args.output.open("x", encoding="utf-8") as output:
            output.write(settings)
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"Could not generate ORB-SLAM3 settings: {error}", file=sys.stderr)
        return 1
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
