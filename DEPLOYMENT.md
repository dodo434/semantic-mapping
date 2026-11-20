# Stages 3–10 — From calibrated sensors to navigation

The application assumes ROS 2 Humble on JetPack 6.x and the D435i topics
from [Stage 1](stage-01/README.md). Complete the hardware and
[Stage 2 calibration](stage-02/README.md) checkpoints before bringing up this
pipeline. The CPU path uses the same semantic map and can add OctoMap geometry
on a Raspberry Pi 5 running ROS 2 Humble on a supported ARM64 OS or container.
OAK-D, ZED, and other bases need their own topic, TF, and camera calibration
adapters. Use a differential-drive base with a hardware emergency stop,
motor-controller timeout, and independent near-field obstacle protection.
The ROS velocity gate is a bench interlock.

## 3. Build and baseline stereo-inertial SLAM

On the target, install the Humble packages and the native dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake git libboost-serialization-dev \
  libeigen3-dev libepoxy-dev libgl1-mesa-dev libglew-dev libssl-dev \
  libx11-dev libopencv-dev python3-colcon-common-extensions python3-numpy \
  python3-opencv python3-venv ros-humble-cv-bridge ros-humble-navigation2 \
  ros-humble-sensor-msgs-py ros-humble-rviz2 ros-humble-tf2-tools

source /opt/ros/humble/setup.bash
project="$PWD"
JOBS=2 bash tools/build-orb.sh
cd ros_ws
colcon build --symlink-install --cmake-args \
  "-DORB_SLAM3_ROOT=$project/artifacts/deps/ORB_SLAM3" \
  "-DCMAKE_PREFIX_PATH=$project/artifacts/deps/install"
cd "$project"
export LD_LIBRARY_PATH="$project/artifacts/deps/install/lib:$project/artifacts/deps/install/lib/aarch64-linux-gnu:$project/artifacts/deps/ORB_SLAM3/lib:$project/artifacts/deps/ORB_SLAM3/Thirdparty/DBoW2/lib:$project/artifacts/deps/ORB_SLAM3/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
source ros_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

Use these environment settings in each robot terminal. The script pins
Pangolin and ORB-SLAM3 and applies the [tracking patch](patches/orb-slam3-semantic.patch).
Reduce `JOBS` to 1 if the compiler is killed on the Jetson. The installed
ROS package also builds without `ORB_SLAM3_ROOT` for the Nav2 plugin smoke
test. That build contains the mapping and Nav2 tools.

Start the Stage 1 camera in fused mode. Launch SLAM with a new trajectory
filename from Stage 2's measured settings. Run a textured loop with masks
disabled to establish the baseline.

```bash
mkdir -p artifacts/runs
ros2 launch semantic_mapping slam.launch.py \
  vocabulary:="$PWD/artifacts/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  settings:="$PWD/artifacts/calibration/d435i-stereo-inertial.yaml" \
  trajectory:="$PWD/artifacts/runs/slam-baseline.tum"

ros2 topic hz /semantic/body_pose
ros2 topic echo /semantic/tracking_state
ros2 topic echo /semantic/map_revision
```

The wrapper pairs left and right images within a 3 ms tolerance and uses the
measured time shift on the SLAM and IMU time basis. It publishes gravity-aligned
IMU/body poses only after IMU initialization and successful tracking.
`/semantic/keyframe` emits new keyframe poses. `/semantic/map_revision`
marks the first initialized map, a map switch, or an ORB-SLAM3 big-change
event such as loop closure. `/semantic/body_pose` and the TUM file use the
original acquisition time. Use stationary initialization followed by
well-lit motion with texture and parallax. Record ground truth in the same
IMU reference frame for the evaluation below.

## 4. Export and benchmark segmentation

Use a separate environment so the ROS Python extensions retain their
system NumPy ABI. On JetPack, install the NVIDIA-compatible CUDA PyTorch
and TorchVision wheels for the installed JetPack release using the Jetson
guide. Verify Torch, TensorRT, and NumPy imports before export.

```bash
python3 -m venv --system-site-packages .venv-vision
source .venv-vision/bin/activate
python -m pip install 'numpy<2' ultralytics==8.3.253
python -c 'import torch, torchvision, tensorrt, numpy, cv_bridge; print(torch.__version__, torch.cuda.is_available(), tensorrt.__version__, numpy.__version__)'
mkdir -p artifacts/models
curl -fL -o artifacts/models/yolov8n-seg.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt
python tools/export-model.py artifacts/models/yolov8n-seg.pt --image-size 416

ros2 launch semantic_mapping perception.launch.py \
  model:="$PWD/artifacts/models/yolov8n-seg.engine" enable_fusion:=false
```

Install or reinstall the vendor GPU wheels last if the generic pip
dependency resolver selected a CPU build. Check that CUDA availability is
true and TensorRT matches the BSP. Run the engine export on the target GPU.
Use only trusted weights and engines. TensorRT INT8 is optional. Use a copy
of the checkpoint with a distinct basename so the previous engine is
preserved, then pass `--int8-data /path/to/representative-seg.yaml`. Benchmark
INT8 against FP16 on the same labeled scenes and retain FP16 if movable
recall or edge agreement regresses. The export omits embedded NMS.
On the Pi, export with `--format onnx --image-size 320` and install ONNX
Runtime, then use `model:=...onnx device:=cpu image_size:=320 max_hz:=3.0`.
The Pi still needs the CPU PyTorch and TorchVision runtime required by
Ultralytics for prediction and postprocessing.

`/semantic/movable_mask` carries a mono8 mask in original RGB pixels.
The segmentation thread keeps only the latest pending RGB frame and
preserves its acquisition stamp. COCO predictions gate potentially movable
things. COCO provides no floor or wall label. It also misses generic boxes
and robots, which therefore default to static occupied when observed by
depth. Fine-tune a segmentation model with your actual obstacle classes
for reliable movable classification. FastSAM supplies shapes and would
need an additional labeling model.

[Jetson PyTorch and TensorRT setup](https://docs.ultralytics.com/guides/nvidia-jetson/) · [Pinned Ultralytics TensorRT export docs](https://github.com/ultralytics/ultralytics/blob/v8.3.253/docs/en/integrations/tensorrt.md)

## 5. Integrate masks and wheel odometry

Verify the camera mount transform `base_link -> camera_link` with your
robot description or a measured static transform. Confirm that the wheel
driver publishes `/odom` and `odom -> base_link`. Avoid duplicate
publishers for either transform. If you temporarily lack wheel odometry,
`use_wheel_odom:=false publish_tf:=true` allows a motor-disabled map test
using a direct `map -> base_link` transform. Enable wheel odometry and
check its axes and scale for navigation.

Restart SLAM with `use_masks:=true publish_tf:=true` and a new trajectory
file. Restart perception with `enable_fusion:=true`. The fusion process
matches aligned depth and RGB masks by header stamps. It projects movable
pixels into each IR view using the camera rig TF. The ORB-SLAM3 patch applies
the same internal rectification to the masks and filters candidates before
octree distribution and descriptor computation. The extractor expands each
exclusion region by its descriptor radius at that pyramid level.
The tracker waits up to 120 ms for a matching mask and can use the available
image features if the mask misses the deadline. Inspect the fallback count
in logs and compare tracking against the unmasked baseline. Mask projection
depends on valid depth and class recall. Do a static target edge alignment
check in both IR views before enabling masking on the robot.

Wheel odometry provides the continuous `odom -> base_link` transform.
SLAM uses camera and IMU to correct `map -> odom`. The semantic node looks
up TF at the frame stamp and skips inputs it cannot transform. Validate
that `map -> base_link` works throughout movement before attempting Nav2.

## 6. Start 3D mapping and visualize

On Jetson, start perception with `publish_filtered_depth:=true`. Install
`nvblox_ros` through the supported Isaac ROS 3.2 Humble environment. Source
that environment and this workspace overlay in the shell where you will run:

```bash
ros2 launch semantic_mapping nvblox.launch.py
ros2 topic hz /semantic/static_depth
ros2 topic hz /semantic/semantic_grid
ros2 topic list -t
rviz2
```

Set the RViz fixed frame to `map`. Add a PointCloud2 for
`/semantic/semantic_voxels` and a Map display for
`/semantic/semantic_grid`. Use the nvblox visualization plugin and inspect
`/nvblox_node/static_map_slice` for the GPU ESDF. The GPU backend integrates
the original depth with movable pixels removed. The parallel 5 cm occupancy
map stores the three semantic classes and drives navigation. This split keeps
class evidence out of nvblox's TSDF voxel type. A loop closure invalidates
the semantic cache and rebuilds it from new observations. Restart the
nvblox process after a map revision to discard geometry integrated in the
old frame. The GPU map is a reconstruction and diagnostic output.

For the Pi, install `ros-humble-octomap-server`, enable
`publish_static_cloud:=true` in perception, and start
`ros2 launch semantic_mapping octomap.launch.py`. The cloud preserves
its optical sensor frame for the ray origin. Restart OctoMap after a map
revision. Start with `map_hz:=3.0 pixel_stride:=12` on the Pi.
[NVIDIA Isaac ROS nvblox 3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html)

## 7. Confirm class behavior and clearing

Measure how high `base_link` sits above the floor, and pass that measurement
in metres as `base_height:=...` to perception. RANSAC fits an upward plane
near this height in the gravity-aligned frame. Points near the fitted plane
are floor. Other valid depth defaults to static structure, including
vertical surfaces. Movable instance evidence takes precedence. The 3D map
tracks bounded per-voxel class evidence and clears transient obstacles by
free-space rays and a 1.2 s observation timeout. The 2D projection labels
floor as 0, movable as 99, static as 100, and unseen cells as −1.

Move a chair across a previously observed floor and check that it clears
without deleting the wall behind it. Walk in and out of view, pass a camera
occlusion, and verify that old messages cannot refresh the costmap. Recording
`/semantic/class_mask` enables the dense evaluation mask with pixel IDs
0 unknown, 1 floor, 2 static, and 3 movable. Leave that subscription off
during rate measurements because generating the dense mask adds work.

## 8. Configure and test Nav2

Set `robot_radius` or a measured polygon `footprint` in both costmaps in
[nav2.yaml](ros_ws/src/semantic_mapping/config/nav2.yaml). The settings use
rolling maps, NavFn A*, and Regulated Pure Pursuit. Both obstacle classes
are lethal where occupied and get normal inflation. Movable occupancy is
temporary. Unobserved and stale cells are blocked. In particular, a low
forward camera can leave the approach to the footprint unobserved. Cover
the initial floor with safe, manual data collection or add proper near-field
sensing, then verify the complete initial footprint has observed floor.

```bash
ros2 launch semantic_mapping nav.launch.py autostart:=true
ros2 topic echo /semantic/bench_velocity
ros2 action list
```

The launch sends commands to the isolated bench velocity topic with the
gate disabled. Start by exercising `/navigate_to_pose` in RViz with motors
disabled. Inspect both costmaps and the planned path. Before enabling the
motor output, verify the E-stop, the base driver's command timeout, wheel
directions, footprint, and a bounded speed and stopping distance. Wire the
operator or base supervisor to publish a positive `std_msgs/Bool` arm heartbeat
on `/semantic/armed` at 10 Hz only while armed. Then relaunch with
`enable_velocity_gate:=true base_command:=/YOUR_DRIVER_COMMAND`, replacing
the topic with the input verified for this controller. The gate targets
`geometry_msgs/Twist` and a differential-drive controller. The gate zeros the
command if the heartbeat, SLAM pose, grid or planner command times out.
Retest loss of each input with the wheels off the ground before a slow drive.

## 9. Optimize within the power budget

Start with the current power mode, fan, 416-pixel FP16 segmentation and
5 cm voxels. First lower the inference resolution or pixel sampling load.
Try 8–10 Hz fusion and 10–15 Hz segmentation independently, then measure
end-to-end at 20–30 Hz pose. On the Pi target 2–5 Hz segmentation and use
a tighter window if memory grows. Check timestamps after changing the
camera profile and repeat the affected calibration.

```bash
sudo nvpmodel -q --verbose
sudo jetson_clocks --show
tegrastats --interval 1000 --logfile artifacts/runs/tegrastats.log
python tools/measure-topics.py --seconds 60 > artifacts/runs/rates.json
tegrastats --stop
```

Use an inline meter at the board and camera supply to evaluate the
under-15 W goal. Log average and peak power and state whether motors
are included. Timestamp age uses the live ROS clock and can be distorted
by unsynchronized ROS graph hosts. Collect instrumentation on the Jetson.
Use `nvpmodel` to test a board-supported higher-power profile and
`jetson_clocks` to isolate clock effects only during benchmarking. MAXN
can exceed the target. Compare to the measured standard profile and
restore it. If CPU scheduling is the bottleneck, try CPU affinity and
process nice settings before elevated real-time priority. Keep cooling
airflow clear and log thermals, CPU/GPU use, RAM, and throttle events.

## 10. Evaluate and troubleshoot

Use a synchronized ground-truth source such as motion capture or a surveyed
reference trajectory. Convert it to TUM format in the camera IMU body
frame. For stereo-inertial, use rigid SE(3) alignment and report metric
scale error. Run these commands in an evaluation virtual environment with
evo installed. Repeat with masks on and off.

```bash
evo_ape tum truth.tum artifacts/runs/slam-baseline.tum --align \
  --save_results artifacts/runs/ape.zip
evo_rpe tum truth.tum artifacts/runs/slam-baseline.tum --align \
  --delta 1 --delta_unit m --save_results artifacts/runs/rpe-1m.zip
python tools/score-labels.py path/to/annotated-png path/to/predicted-png
```

| Measure | Protocol |
| --- | --- |
| ATE and RPE | Report ATE RMSE in metres, translation RPE at 1 m and 10 m, rotation RPE, percent of matched frames, and resets. Compute drift as position error at the end of a no-loop section divided by its measured travel distance. Compare to the under-1% target. |
| Segmentation and map | Export paired RGB-aligned class masks and score per-class IoU, mIoU, movable recall and worst-case occlusion errors. Report labeled coverage. Annotate 50–100 frames covering corridors, people, boxes, lighting changes and floor transitions. Score temporary obstacle clearance time separately. |
| Runtime | Report pose, segmentation and map Hz, capture-to-receive p50 and p95 latency, and dropped frames over at least 60 s. Test a cold start and a 30–60 minute run. |
| Power and navigation | Record board-plus-camera watts, thermals and throttling. Log goal success, interventions, minimum clearance, unintended stops, and unsafe motion on loss of localization, sensing, or arm heartbeat. Start motor-disabled and progress to supervised low-speed runs. |

| Symptom | Check |
| --- | --- |
| Drift near blank walls | Confirm IMU timing and noise first. Increase scene texture or use a separately validated depth-odometry input when required. The base wheels can support local continuity but can slip. |
| Tracking drops at speed | Review blur and IMU gaps. Tune exposure and lighting. The D435i IR pair uses global shutter for tracking. Reduce robot speed and retest. |
| Ghosts after motion | Check the 3D moving evidence timeout, class misses, whether current rays see behind the obstacle, and map revision handling. A COCO miss can enter static occupancy and requires later free-space evidence to clear. |
| Memory or heat climbs | Disable dense evaluation and nvblox visual streams while benchmarking, lower sampling load, monitor the rolling bounds, and verify cooling. |
| Floor missing or robot refuses to plan | Confirm camera pitch, `base_height`, valid depth, floor coverage under the footprint, and that both grids use `map`. Inspect TF and the controller's topic. |
| Engine fails at startup | Compare the installed TensorRT runtime and exact export device. Re-export locally using the same TensorRT, CUDA, and model version. |

The hardware acceptance requires measured results. CI builds the software
and uses synthetic inputs. TensorRT, nvblox, thermal behavior, drift,
clearance, and actual motor stop times are measured on the robot.
