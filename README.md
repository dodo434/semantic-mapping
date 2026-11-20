# Semantic mapping for a differential-drive robot

[Stage 1 brings up JetPack, ROS 2 Humble, and the RealSense D435i](stage-01/README.md).
It provides a camera launcher, a smoke-bag recorder, and an offline bag
checker. Calibration follows the on-hardware Stage 1 checkpoint.

## Local checks

From the project root, run these checks with Bash and Python 3.10 or newer.
The shell tests use a fake ROS CLI and the bag tests use SQLite fixtures.

```bash
for script in stage-01/*.sh; do bash -n "$script" || break; done
python3 -m unittest discover -s tests -v
shellcheck stage-01/*.sh
```

ShellCheck is also run by GitHub Actions. Verify live camera and IMU timing,
wheel odometry, installed package security updates, and cooling on the robot
using the Stage 1 runbook.
