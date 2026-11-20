import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import unittest

import yaml


STAGE = Path(__file__).resolve().parents[1] / "stage-02"
GENERATOR = STAGE / "make-orb-settings.py"
CALIBRATION = runpy.run_path(str(GENERATOR))


def fixture():
    left = {
        "camera_model": "pinhole",
        "distortion_model": "radtan",
        "intrinsics": [380.0, 380.5, 320.0, 240.0],
        "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
        "resolution": [640, 480],
        "T_cam_imu": [[0, -1, 0, 0.01], [1, 0, 0, 0.02], [0, 0, 1, 0], [0, 0, 0, 1]],
        "timeshift_cam_imu": -0.003,
    }
    right = dict(left, T_cn_cnm1=[
        [1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1],
    ])
    imu = {name: 0.0002 for name in CALIBRATION["NOISE_KEYS"].values()}
    imu["update_rate"] = 200.0
    return {"cam0": left, "cam1": right}, imu


class Stage2Tests(unittest.TestCase):
    def test_settings_use_inverted_transforms_and_preserve_shift(self):
        chain, imu = fixture()
        settings = CALIBRATION["build_settings"](chain, imu, 30)
        self.assertIn('Camera.type: "PinHole"', settings)
        self.assertIn("Calibration.CameraToImuTimeShiftSec: -0.003", settings)
        stereo = settings.split("Stereo.T_c1_c2:", 1)[1].split("IMU.T_b_c1:", 1)[0]
        self.assertIn("1.0, 0.0, 0.0, 0.05", stereo)
        self.assertIn("0.0, 1.0, 0.0, -0.02, -1.0, 0.0, 0.0, 0.01", settings)

    def test_reject_reversed_stereo_and_missing_imu_values(self):
        chain, imu = fixture()
        chain["cam1"]["T_cn_cnm1"][0][3] = 0.05
        with self.assertRaisesRegex(ValueError, "positive X"):
            CALIBRATION["build_settings"](chain, imu, 30)
        chain, imu = fixture()
        del imu["gyroscope_random_walk"]
        with self.assertRaises(KeyError):
            CALIBRATION["build_settings"](chain, imu, 30)

    def test_generator_leaves_existing_output_intact(self):
        chain, imu = fixture()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source, noise, result = (directory / name for name in ("chain.yml", "imu.yml", "output.yml"))
            source.write_text(yaml.safe_dump(chain), encoding="utf-8")
            noise.write_text(yaml.safe_dump(imu), encoding="utf-8")
            command = [sys.executable, str(GENERATOR), str(source), str(noise), str(result),
                       "--camera-fps", "30"]
            created = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(created.returncode, 0, created.stderr)
            original = result.read_bytes()
            rejected = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(rejected.returncode, 1)
            self.assertEqual(result.read_bytes(), original)

    def test_capture_records_only_the_requested_streams(self):
        left = "/camera/camera/infra1/image_rect_raw"
        right = "/camera/camera/infra2/image_rect_raw"
        imu = "/camera/camera/imu"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            cli = directory / "ros2"
            cli.write_text(
                '#!/bin/bash\n'
                'if [[ $1 == topic ]]; then printf "%s\\n" "$MOCK_TOPICS"; '
                'else printf "%s\\n" "$@"; fi\n',
                encoding="utf-8",
            )
            cli.chmod(0o700)
            env = dict(os.environ, ROS_DISTRO="humble",
                       PATH=f"{directory}:{os.environ['PATH']}", MOCK_TOPICS="\n".join([left, right, imu]))
            output = directory / "new bag"
            for mode, topics in (("stereo", [left, right]), ("camera-imu", [left, right, imu]),
                                 ("allan", [imu])):
                with self.subTest(mode=mode):
                    result = subprocess.run(["bash", str(STAGE / "record-calibration.sh"), mode, str(output)],
                                            env=env, text=True, capture_output=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(result.stdout.splitlines(), [
                        "bag", "record", "-s", "sqlite3", f"--output={output}", *topics,
                    ])
