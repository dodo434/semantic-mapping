# Stage 1 — Jetson and camera bring-up

This procedure assumes an NVIDIA Jetson Orin Nano 8GB Developer Kit and a single
RealSense D435i connected directly over USB 3. The boot medium and existing
firmware are unconfirmed. The differential-drive controller is unspecified, so
the bench procedure leaves the motors disabled and checks wheel odometry only
if a base driver is already available.

The two IR streams and interpolated IMU are the inputs for later
stereo-inertial tracking. RGB and RGB-aligned depth will be used for semantic
mapping. This stage collects a baseline for both paths.

## 1. Flash or update JetPack

Keep the board on a stable supply with active cooling while flashing and
testing. Back up anything on the target boot medium before imaging it.

1. **Developer kit with microSD and JetPack 6-capable firmware.** On a host
   computer, download the JetPack 6.2.1 microSD image from the JetPack 6.2.3
   release page. Flash it with Etcher, insert it in the Jetson and complete
   first boot. NVIDIA publishes 6.2.1 as the starting microSD image for the
   6.2.3 upgrade.
2. **Factory firmware or JetPack 5.x.** Follow NVIDIA's firmware update
   procedure before booting the JetPack 6 image, or use SDK Manager to flash
   JetPack 6.2.3 in Force Recovery Mode from a supported host.
3. **Developer kit booting from NVMe.** In NVIDIA SDK Manager, select
   JetPack 6.2.3, the Orin Nano Developer Kit and NVMe as the target storage.
   SDK Manager prepares both the boot firmware and the target storage.
4. **Existing JetPack 6 installation on the NVIDIA carrier.** Inspect the
   current BSP and follow the upgrade commands below if needed. For a
   third-party carrier, follow its vendor's BSP and flashing instructions.

[NVIDIA JetPack 6.2.3 release and install paths](https://developer.nvidia.com/embedded/jetpack-sdk-623) · [NVIDIA flashing and bootloader update guide](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/FlashingSupport.html)

On the Jetson:

```bash
cat /etc/nv_tegra_release
. /etc/os-release
printf '%s\n' "$PRETTY_NAME"
dpkg --print-architecture
lsblk -o NAME,SIZE,TYPE,MOUNTPOINTS
```

The target for this runbook is Jetson Linux 36.5.2, Ubuntu 22.04 and Debian
architecture `arm64`. If upgrading from the official 6.2.1 image, use
`sudoedit /etc/apt/sources.list.d/nvidia-l4t-apt-source.list` and set the
release in both NVIDIA entries (`common` and `t234`) to `r36.5`. Then run:

```bash
sudo apt update
sudo apt dist-upgrade
sudo reboot
```

Once the board is back, check the BSP again with `cat /etc/nv_tegra_release`.
Install the JetPack compute stack if it was omitted during flash. Check the
quoted package versions and the current power mode for future benchmarks.

```bash
sudo apt update
sudo apt install nvidia-jetpack
dpkg-query -W nvidia-l4t-core nvidia-jetpack libnvinfer10
sudo nvpmodel -q
```

[NVIDIA Jetson Linux upgrade procedure](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/SoftwarePackagesAndTheUpdateMechanism.html)

## 2. Install ROS 2 Humble

Run this on the Jetson. Review the upgrade transaction before accepting it,
especially on a freshly flashed image where ROS dependencies need current
Ubuntu system packages.

```bash
sudo apt update
sudo apt install locales software-properties-common curl jq usbutils
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe

(
  set -euo pipefail
  ros_apt_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    jq -er '.tag_name | select(test("^[0-9]+(\\.[0-9]+)+$"))') || exit 1
  ros_apt_download=$(mktemp -d) || exit 1
  trap 'rm -f -- "$ros_apt_download/ros2-apt-source.deb"; rmdir -- "$ros_apt_download"' EXIT
  curl -fL -o "$ros_apt_download/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.jammy_all.deb" || exit 1
  sudo dpkg -i "$ros_apt_download/ros2-apt-source.deb"
) || exit 1
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins

source /opt/ros/humble/setup.bash
ros2 doctor --report
```

[ROS 2 Humble installation source](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)

## 3. Install the D435i driver and start the streams

Try the ROS binary package first. Record the installed versions and compare
the launch options with the reference. The launch arguments in this stage
were checked against wrapper tag `4.59.1`.

```bash
apt-cache policy ros-humble-realsense2-camera ros-humble-librealsense2
sudo apt install ros-humble-realsense2-camera
dpkg-query -W 'ros-humble-*realsense*'
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py --show-args
lsusb -t
```

Use a USB 3 data cable and a direct port. Look for `5000M` or higher in
`lsusb -t` and confirm the USB speed in the camera startup log. If the
device opens but the motion streams fail, see the Jetson driver troubleshooting
below before changing firmware or kernel drivers.

Copy this project to the Jetson and run these commands from the project root.
In terminal 1, launch in fused mode:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 daemon stop
umask 077
mkdir -p artifacts
set -o pipefail
bash stage-01/launch-d435i.sh fused 2>&1 | tee artifacts/camera-fused.log
```

The four primary image streams request 640×480 at 30 Hz. The wrapper matches
video frames in software and publishes depth in the RGB pixel grid, alongside
native depth. The gyro requests 200 Hz. The accelerometer starts at the
device's default supported rate because D435i revisions differ. Set
`ACCEL_FPS` before launching if you need a specific supported accelerometer
profile. In fused mode, the wrapper interpolates acceleration to the gyro
timestamps. Record the negotiated rates from the startup log and keep the
same camera settings for both runs.

Use the same `ROS_LOCALHOST_ONLY` value in all bench terminals. A local-only
graph keeps ROS traffic on the Jetson for this test. Run the bench as a normal
user. Bags and logs can reveal people, interior layouts, and camera IDs.
Keep the `artifacts/` directory local and share reviewed text summaries.

[RealSense wrapper launch arguments](https://github.com/RealSenseAI/realsense-ros/blob/4.59.1/realsense2_camera/launch/rs_launch.py) · [RealSense wrapper topics and IMU modes](https://github.com/RealSenseAI/realsense-ros)

## 4. Verify rates, timestamps and recording

In terminal 2, from the project root:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 topic list -t
camera=/camera/camera
for topic in infra1/image_rect_raw infra2/image_rect_raw color/image_raw \
  depth/image_rect_raw aligned_depth_to_color/image_raw imu; do
  printf '\n%s\n' "$camera/$topic"
  timeout 12s ros2 topic hz "$camera/$topic"
done
ros2 topic echo "$camera/imu" --once --qos-reliability best_effort
ros2 topic echo /tf_static --once --qos-durability transient_local
```

`timeout` ends each rate measurement after 12 seconds. Expect reported rates
near 30 Hz for each image stream and 200 Hz for the interpolated IMU. At rest,
angular velocity should be near zero and acceleration magnitude near
9.8 m/s². In two terminals, use
`ros2 topic echo TOPIC --field header --qos-reliability best_effort` on
the two IR topics and compare corresponding header timestamps. Check that
stamps increase, the two IR views have the same or closely matching stamps,
and the IMU is in the same time domain. Repeat for RGB and aligned depth.
Record any persistent skew. Camera–IMU time offset is measured in calibration.

With the camera publishing steadily and several GB available for raw images,
record about 20 seconds. Stop with **Ctrl+C** so rosbag writes its metadata.
Use a fresh output path for each retry.

```bash
bash stage-01/record-smoke-test.sh fused artifacts/fused-smoke
ros2 bag info artifacts/fused-smoke
python3 stage-01/check-smoke-bag.py fused artifacts/fused-smoke
```

Stop the camera in terminal 1. Start it again in raw IMU mode:

```bash
bash stage-01/launch-d435i.sh raw 2>&1 | tee artifacts/camera-raw.log
```

In terminal 2, check the separate IMU streams and record a second bag:

```bash
timeout 12s ros2 topic hz /camera/camera/gyro/sample
timeout 12s ros2 topic hz /camera/camera/accel/sample
bash stage-01/record-smoke-test.sh raw artifacts/raw-smoke
ros2 bag info artifacts/raw-smoke
python3 stage-01/check-smoke-bag.py raw artifacts/raw-smoke
```

The checker reads the finalized SQLite bag in read-only mode. It requires
messages on all expected topics and at least 10 seconds of receive timestamps
within 15% of nominal rates for the image and gyro-timed streams. Compare the
raw accelerometer rate it prints with the negotiated rate in the launch log.
The checker uses recorder timestamps. Use the header check above for
sensor timestamp monotonicity and skew. Check the
camera logs for dropped frames, stream resets or profile fallback. The raw
smoke bag verifies access to native-rate readings. A dedicated stationary
capture will be needed for Allan variance.

If a base driver is already running, use `ros2 topic list -t` to find the
`nav_msgs/msg/Odometry` topic. Check its rate and one message with
`ros2 topic hz` and `ros2 topic echo --once`. Record the topic, rate,
`header.frame_id`, and `child_frame_id`. With the base stationary, check
that the reported twist is near zero.

[Humble rosbag2 recorder command](https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/record.py)

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Jetson fails to boot the microSD image | Check the bootloader generation against NVIDIA's JetPack 6 firmware instructions. If the kit is on JetPack 5 firmware, complete the QSPI update or flash through SDK Manager. |
| Camera absent, disconnecting or detected at 480M | Use a different direct USB 3 port and known USB 3 cable. Check supply stability, `lsusb -t`, and recent USB logs with `sudo journalctl -k -b -n 100`. |
| USB sees camera but the process gets permission errors | Check that the current RealSense udev rules are installed. Reload the rules and reconnect the camera. Use the normal user for ROS. |
| Video works but motion does not | Check which IMU mode is running, the negotiated rates, and the wrapper's built-versus-running SDK versions. If the installed backend cannot surface motion on the Jetson BSP, follow the RealSense Jetson guide for an RSUSB source build and build a compatible ROS wrapper against that SDK. Keep one SDK on the wrapper's library path. |
| Image rate low or frame timeouts | Inspect logs for profile fallback and verify the USB speed. Test IR + IMU alone by temporarily disabling color, depth and alignment in a manual launch, then restore the full set. Check rate without the recorder and use fast storage for the bag. |
| Hardware timestamps reset or streams drift apart | Check shutdown logs for UVC/USB reconnects, confirm the timestamp domains and repeat the bag with a single launch. Diagnose recurring resets before attempting calibration. |

[RealSense installation on Jetson](https://github.com/RealSenseAI/librealsense/blob/master/doc/installation_jetson.md) · [RealSense Linux setup and udev rules](https://github.com/RealSenseAI/librealsense/blob/master/doc/installation.md)

## Stage 1 checkpoint

Confirm the exact board and carrier, boot medium, camera model and base
controller. Provide the JetPack/ROS/RealSense versions, negotiated rates,
any timestamp skew or stream warnings, and the two `ros2 bag info`
summaries. Include the odometry details if available. Use this evidence to
close Stage 1 before collecting calibration data in Stage 2.
