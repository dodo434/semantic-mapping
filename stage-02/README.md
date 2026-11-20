# Stage 2 — D435i stereo and IMU calibration

Run the [Stage 1 checks](../stage-01/README.md) on the Jetson before collecting
calibration data. This procedure uses the same 640×480 stereo IR pair at 30 Hz
and the same gyro-timed IMU stream at 200 Hz that the SLAM adapter will use.
Keep the D435i rigidly mounted for deployment. Record its firmware, negotiated
stream rates, emitter setting, and mount orientation with the run.

## 1. Prepare the target and collect three recordings

Print a [Kalibr Aprilgrid](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)
at full scale and attach it to a rigid, flat board. Measure the actual tag
side and the clear gap between adjacent tags. Create
`artifacts/calibration/target.yaml` on the processing computer. The numbers
below describe a 6×6 grid with 88 mm tags and 26.4 mm gaps. Replace them with
the measurements from your own print.

```yaml
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.30
```

On the Jetson, start the camera in one terminal from the project root:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
bash stage-01/launch-d435i.sh fused
```

In a second terminal, source ROS and use the same discovery setting:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
umask 077
mkdir -p artifacts/calibration
bash stage-02/record-calibration.sh stereo artifacts/calibration/stereo-ros2
# Ctrl+C after the stereo sweep.
bash stage-02/record-calibration.sh camera-imu artifacts/calibration/camera-imu-ros2
# Ctrl+C after the motion sweep.
bash stage-02/record-calibration.sh allan artifacts/calibration/allan-ros2
# Ctrl+C after at least 3 hours of stationary data.
```

- **Stereo sweep.** Collect 30–60 seconds. Move the board slowly through both
  IR views at several tilts and distances. Cover each image corner and edge.
  Use bright, even lighting. Check that the printed target has strong contrast
  in the IR images.
- **Camera–IMU sweep.** Fix the target, then move the camera and IMU together
  for 45–90 seconds. With the base drive disabled, safely move the assembly
  by hand so you excite roll, pitch, yaw and translation along all axes. Use
  smooth motion and avoid impacts or prolonged occlusion. Repeat and save a
  second independent sweep under another name for the acceptance check.
- **Allan recording.** Let the IMU warm up and keep it stationary on a stable
  surface for at least three hours. The recorder saves only the IMU topic for
  this run. The fused stream interpolates accelerometer samples, so these
  noise estimates characterize the stream used by SLAM. Note the native
  accelerometer rate from Stage 1 when interpreting short-period Allan slopes.

Stop each bag with Ctrl+C and inspect it with `ros2 bag info`. Confirm that
both IR topics in each sweep have similar counts, that `/camera/camera/imu`
is present in the two IMU recordings, and that there are no camera resets.
Keep bags, calibration results, and reports under `artifacts/`.

## 2. Convert the ROS 2 bags on the processing computer

Copy the complete bag directories, including `metadata.yaml` and the
`.db3` file, to the processing computer. The conversion runs in its own
Python virtual environment. Use Python 3.10 or newer.

```bash
python3 -m venv .venv-calibration
source .venv-calibration/bin/activate
python -m pip install rosbags==0.11.9 PyYAML==6.0.3
cd artifacts/calibration
rosbags-convert --src stereo-ros2 --dst stereo.bag --src-typestore ros2_humble
rosbags-convert --src camera-imu-ros2 --dst camera-imu.bag --src-typestore ros2_humble
rosbags-convert --src allan-ros2 --dst allan.bag --src-typestore ros2_humble
```

Use `rosbag info` in a ROS Noetic environment to compare timestamps, topic
types and counts with the ROS 2 summaries. Kalibr uses the sensor header
timestamps for estimation. [Rosbags conversion documentation](https://ternaris.gitlab.io/rosbags/topics/convert.html)

## 3. Estimate IMU noise with Allan deviation

On an Ubuntu 20.04 ROS Noetic processing VM with `catkin_tools` and
`rosdep` installed, build the analysis tool. Use an isolated VM or
container for these ROS 1 tools and leave the Jetson on ROS 2.

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/allan_ws/src
git clone https://github.com/ori-drs/allan_variance_ros.git ~/allan_ws/src/allan_variance_ros
git -C ~/allan_ws/src/allan_variance_ros checkout --detach 1d54b602ee7f2ba0427865d63afe4945d913ed24
cd ~/allan_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin build allan_variance_ros
source devel/setup.bash
```

Copy [allan-d435i.yaml](allan-d435i.yaml) beside `allan.bag`. In the VM,
start `roscore` in a separate terminal, then run the following from the
directory containing both files. Use a graphical VM session to inspect the
analysis plots. If the VM has no current rosdep cache, initialize rosdep
as needed and run `rosdep update --include-eol-distros`.

```bash
mkdir -p allan-cooked
rosrun allan_variance_ros cookbag.py --input allan.bag --output allan-cooked/allan.bag
rosrun allan_variance_ros allan_variance allan-cooked "$PWD/allan-d435i.yaml"
rosrun allan_variance_ros analysis.py --data allan-cooked/allan_variance.csv \
  --config allan-d435i.yaml --output imu.yaml
```

Inspect the plots for the white-noise and random-walk regions. Treat a fit
with strong vibration, thermal drift, or no discernible random-walk region as
provisional. Check that `imu.yaml` names `/camera/camera/imu`, reports the
stream rate, and contains four positive noise and random-walk values in SI
units. Copy that file back beside the two motion bags.
[Allan Variance ROS](https://github.com/ori-drs/allan_variance_ros/tree/1d54b602ee7f2ba0427865d63afe4945d913ed24)

## 4. Run stereo and camera–IMU calibration

On an x86-64 processing host with Docker, build the pinned Kalibr revision.
Run these commands from the project root. The working directory inside the
container contains the ROS 1 bags, `target.yaml`, and `imu.yaml`.

```bash
git clone https://github.com/ethz-asl/kalibr.git /tmp/semantic-mapping-kalibr
git -C /tmp/semantic-mapping-kalibr checkout --detach 1f60227442d25e36365ef5f72cd80b9666d73467
docker build -t semantic-kalibr -f /tmp/semantic-mapping-kalibr/Dockerfile_ros1_20_04 \
  /tmp/semantic-mapping-kalibr
docker run --rm -it --network none --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e MPLBACKEND=Agg \
  --mount "type=bind,source=$PWD/artifacts/calibration,target=/data" \
  -w /data --entrypoint /bin/bash semantic-kalibr
```

Inside the container, estimate stereo intrinsics and the left-to-right pose,
then estimate the IMU extrinsics and time shift:

```bash
source /catkin_ws/devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras --bag stereo.bag --target target.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  --dont-show-report

rosrun kalibr kalibr_calibrate_imu_camera --bag camera-imu.bag \
  --cam stereo-camchain.yaml --imu imu.yaml --target target.yaml --dont-show-report
```

Save the PDFs and result summaries. Repeat the camera–IMU command with the
second motion bag and compare both transforms and time shifts.
[Kalibr installation](https://github.com/ethz-asl/kalibr/wiki/installation) ·
[Stereo calibration](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) ·
[Camera–IMU calibration](https://github.com/ethz-asl/kalibr/wiki/Camera-IMU-calibration)

## 5. Generate and review the ORB-SLAM3 YAML

Run this on the processing host, using the Python virtual environment from
step 2 and the measured files from step 4:

```bash
.venv-calibration/bin/python stage-02/make-orb-settings.py \
  artifacts/calibration/camera-imu-camchain-imucam.yaml \
  artifacts/calibration/imu.yaml \
  artifacts/calibration/d435i-stereo-inertial.yaml --camera-fps 30
```

`Stereo.T_c1_c2` maps right-camera coordinates into the left camera.
`IMU.T_b_c1` maps left-camera coordinates into the IMU. The generator inverts
Kalibr’s transforms to get those directions. It uses ORB-SLAM3’s `PinHole`
mode so ORB-SLAM3 applies the calibrated stereo geometry to the exact images
used in the bags. Pass the original driver images to the SLAM API. Its
internal rectification adjusts the left camera frame, which later matters for
projecting RGB masks into the tracking images.

`Calibration.CameraToImuTimeShiftSec` records the measured left-camera time
shift in the settings file. The Stage 3 ROS adapter must apply
`t_imu = t_left_camera + shift` for both the IMU interval boundary and
the camera timestamp passed to `TrackStereo`. Pass the raw IMU stamps with
those samples so both inputs use the same time basis. Preserve the original
camera acquisition timestamp for output messages and later pose lookup.
ORB-SLAM3 core reads its built-in settings and leaves this extra key for the
adapter.
[Kalibr YAML conventions](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) ·
[ORB-SLAM3 settings and rectification](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Settings.cc)

## Stage 2 checkpoint

Review the two calibration PDFs for well-distributed target detections,
reprojection residuals without a spatial pattern, and IMU residuals without
bursts or sustained bias. A starting acceptance target is RMS reprojection
error below one pixel per camera. Compare the measured stereo baseline with
the device’s reported IR-to-IR baseline, nominally around 50 mm on a D435i.
Use the second independent sweep to confirm that the IMU pose and time shift
are repeatable. Investigate outliers or unstable estimates before going on.
Record the baseline, per-camera RMS, the time shift in milliseconds, and
the four IMU noise values. Keep the measured YAML for Stage 3.
