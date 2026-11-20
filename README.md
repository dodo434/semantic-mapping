# A robot that maps where it can drive

This project helps a two-wheel robot build a 3D map of an indoor space as it
moves. The robot estimates where it is, builds a map from what the camera sees,
and uses that map to plan a route. **Semantic mapping** means the map records
what an area represents as well as where it is.

## What the robot sees

| On the map | Examples | How the robot uses it |
| --- | --- | --- |
| **Floor** | An observed, flat patch of ground | The route planner can use it once the robot has enough room to pass. |
| **Fixed obstacle** | Walls, pillars, and fixed furniture | The planner goes around it and leaves clearance. |
| **Movable obstacle** | A person, chair, or bag that the model recognizes | The planner keeps clear. The mark fades as fresh observations show the area has cleared, or when the temporary mark expires. |
| **Unseen** | An area the camera has yet to observe | The planner treats it as blocked. |

The supplied model recognizes common objects such as people and chairs.
Ordinary boxes and other robots can require additional training examples.
When the depth camera sees one of these objects without a label, the map
normally treats it as a fixed obstacle until later observations clear it.

## How it works

1. **Find the robot’s position.** The two infrared cameras see the same scene
   from slightly different angles. A motion sensor, called an IMU, measures
   turns and acceleration. ORB-SLAM3 uses both to estimate the robot’s
   position and orientation. Wheel readings track how far the base rolls.
2. **Find objects in the color picture.** YOLOv8 outlines objects it recognizes.
   The position estimator can skip image features on those moving objects and
   use more of the background to track the robot.
3. **Build the map.** The depth camera measures how far away surfaces are. The
   system matches each camera frame to the robot’s position when the frame
   was taken. It divides the space into 5 cm cubes and records evidence
   for floor and obstacles. It finds the floor by looking for a flat surface
   near the expected ground height.
4. **Plan a route.** The software turns the 3D evidence into a top-down map.
   Nav2 uses that map to plan a path and work out how to steer the two wheels.
   The velocity gate starts disabled. A stale map blocks navigation.

## What you need

- **Computer and camera.** The main setup targets an NVIDIA Jetson Orin Nano
  8GB and a RealSense D435i connected over USB 3. The Jetson uses JetPack 6
  and ROS 2 Humble. Use a stable power supply and a fan.
- **Robot base.** Use a two-wheel base that reports wheel motion. Before
  connecting navigation to the motors, verify the motor controller’s
  command input, its automatic stop on lost commands, and the physical
  emergency stop.
- **Other hardware.** The Raspberry Pi 5 path uses CPU inference with ONNX
  Runtime and can use OctoMap for 3D reconstruction. Other cameras and
  bases need their own driver settings, sensor alignment and on-robot tests.

## Where to start

The first two guides take you from a new board to measured camera settings.
The deployment guide then takes you through tracking, mapping and a supervised
drive.

| Stage | What you’ll do | Guide |
| --- | --- | --- |
| 1. Set up the board | Install the software, plug in the camera, check the video and motion streams, and record a short test. | [Board and camera setup](stage-01/README.md) |
| 2. Calibrate the mounted camera | Measure how the cameras and motion sensor line up, how their clocks differ, and how noisy the motion sensor is. | [Calibration](stage-02/README.md) |
| 3–5. Track and identify objects | Check position tracking by itself, run object recognition, then connect the two. | [Tracking and recognition](DEPLOYMENT.md#3-build-and-baseline-stereo-inertial-slam) |
| 6–7. Build and inspect the map | View the 3D and top-down maps. Move a chair and check that it clears from the map. | [Mapping](DEPLOYMENT.md#6-start-3d-mapping-and-visualize) |
| 8–10. Test navigation and measure performance | Try route planning with the motors disabled. Verify stopping behavior before a supervised drive. Measure speed, drift, latency and power. | [Navigation and testing](DEPLOYMENT.md#8-configure-and-test-nav2) |

## What’s been tested

The automated checks build and start the tracking software, verify that the
mapping code handles example depth and object data, and check that navigation
blocks a stale map. The object model also ran on a sample image using CPU
inference. The tests and their results are available in GitHub Actions.
[GitHub Actions](https://github.com/dodo434/semantic-mapping/actions)

Your robot still needs its own camera calibration, motor setup, and driving
tests. On the Jetson, the project aims for 20–30 position estimates per second,
10–15 camera frames analyzed for objects per second, and 5–10 map updates per second. The
power target is below 15 W for the board and camera, and the drift target is
below 1% of the distance traveled. These are targets to measure on the robot.

## Run the checks without a robot

On Linux with Bash and Python 3.10–3.12, run these commands from the project
folder. The tests use example sensor data and can run without ROS or a camera.

```bash
python3 -m venv .venv-calibration
source .venv-calibration/bin/activate
python -m pip install PyYAML==6.0.3 numpy==1.26.4
python -m unittest discover -s tests -v
```

If you have ShellCheck installed, you can also run a static check of the shell
scripts.

```bash
shellcheck stage-0[12]/*.sh tools/*.sh
```
