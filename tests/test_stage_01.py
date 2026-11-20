import json
import os
from pathlib import Path
import runpy
import sqlite3
import subprocess
import tempfile
import unittest


STAGE = Path(__file__).resolve().parents[1] / "stage-01"
BAG_CHECK = STAGE / "check-smoke-bag.py"
expected_rates = runpy.run_path(str(BAG_CHECK))["expected_rates"]


class Stage1Tests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        cli = self.directory / "ros2"
        cli.write_text("""#!/usr/bin/env python3
import json, os, sys
if sys.argv[1:] == ["topic", "list"]:
    print(os.environ["TEST_ROS_TOPICS"])
else:
    old_mask = os.umask(0)
    print(json.dumps([sys.argv[1:], os.environ["ROS_LOCALHOST_ONLY"], old_mask]))
""", encoding="utf-8")
        cli.chmod(0o700)

    def shell(self, script, *args, **settings):
        environment = dict(os.environ, PATH=f"{self.directory}:{os.environ['PATH']}",
                           ROS_DISTRO="humble", ROS_LOCALHOST_ONLY="", ACCEL_FPS="")
        return subprocess.run(["bash", str(STAGE / script), *args],
                              env=environment | settings, text=True, capture_output=True)

    def test_launch_modes_and_localhost_default(self):
        for mode, method in (("fused", 2), ("raw", 0)):
            with self.subTest(mode=mode):
                result = self.shell("launch-d435i.sh", mode, ACCEL_FPS="100")
                self.assertEqual(result.returncode, 0, result.stderr)
                args, localhost, _ = json.loads(result.stdout)
                self.assertEqual(args[:3], ["launch", "realsense2_camera", "rs_launch.py"])
                self.assertIn(f"unite_imu_method:={method}", args)
                self.assertIn("accel_fps:=100", args)
                self.assertIn("gyro_fps:=200", args)
                self.assertEqual(localhost, "1")

    def test_launch_argument_and_environment_validation(self):
        for args, settings, status in (
            (("bad",), {}, 2),
            (("raw", "extra"), {}, 2),
            (("raw",), {"ACCEL_FPS": "200 --unexpected"}, 2),
            ((), {"ROS_DISTRO": "jazzy"}, 1),
        ):
            with self.subTest(args=args, settings=settings):
                self.assertEqual(self.shell("launch-d435i.sh", *args, **settings).returncode, status)

    def test_record_modes_and_private_permissions(self):
        for mode in ("fused", "raw"):
            with self.subTest(mode=mode):
                topics = sorted(expected_rates(mode))
                output = self.directory / "new bag"
                result = self.shell("record-smoke-test.sh", mode, str(output),
                                    TEST_ROS_TOPICS="\n".join(topics))
                self.assertEqual(result.returncode, 0, result.stderr)
                args, localhost, umask = json.loads(result.stdout)
                self.assertEqual(args[:4], ["bag", "record", "-s", "sqlite3"])
                self.assertEqual(args[4], f"--output={output}")
                self.assertEqual(sorted(args[5:]), topics)
                self.assertEqual((localhost, umask), ("1", 0o077))

    def test_recorder_refuses_missing_topics_and_existing_path(self):
        output = self.directory / "bag"
        missing = self.shell("record-smoke-test.sh", "fused", str(output), TEST_ROS_TOPICS="")
        self.assertEqual(missing.returncode, 1)
        self.assertIn("Missing topic:", missing.stderr)
        output.symlink_to(self.directory / "absent")
        existing = self.shell("record-smoke-test.sh", "fused", str(output))
        self.assertEqual(existing.returncode, 2)
        self.assertIn("Provide a new bag path", existing.stderr)

    def bag_fixture(self, mode, *, omitted=None, slow=None):
        folder = self.directory / f"{mode} bag"
        folder.mkdir()
        (folder / "metadata.yaml").touch()
        with sqlite3.connect(folder / "data.db3") as database:
            database.executescript("""
                CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);
                CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);
            """)
            for index, (name, target) in enumerate(expected_rates(mode).items()):
                if name == omitted:
                    continue
                database.execute("INSERT INTO topics VALUES (?, ?)", (index, name))
                rate = target // 2 if name == slow else target
                timestamps = (round(i * 1e9 / rate) for i in range(12 * rate + 1)) if rate else (0,)
                database.executemany("INSERT INTO messages(topic_id, timestamp) VALUES (?, ?)",
                                     ((index, timestamp) for timestamp in timestamps))
        return folder

    def check(self, mode, directory):
        return subprocess.run(["python3", str(BAG_CHECK), mode, str(directory)],
                              text=True, capture_output=True)

    def test_bag_checker_passes_both_modes(self):
        for mode in ("fused", "raw"):
            with self.subTest(mode=mode):
                self.assertEqual(self.check(mode, self.bag_fixture(mode)).returncode, 0)

    def test_bag_checker_reports_missing_and_slow_topics(self):
        slow = "/camera/camera/imu"
        result = self.check("fused", self.bag_fixture("fused", omitted="/tf_static", slow=slow))
        self.assertEqual(result.returncode, 1)
        self.assertIn(f"FAIL {slow}", result.stdout)
        self.assertIn("FAIL /tf_static", result.stdout)

    def test_bag_checker_rejects_unfinalized_bag(self):
        result = self.check("raw", self.directory)
        self.assertEqual(result.returncode, 2)
        self.assertIn("finalized bag directory", result.stderr)
