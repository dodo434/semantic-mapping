# A robot that maps where it can drive

This project helps a two-wheel robot build a 3D map of a room and plan a route.
The map shows **floor it can drive on**, **fixed obstacles** like walls, and
**movable obstacles** like people or chairs. Unseen areas are treated as blocked.

## Get the project

Open a terminal with Git installed and run:

```bash
git clone https://github.com/dodo434/semantic-mapping.git
cd semantic-mapping
```

Have a robot? Start with the [board and camera setup](stage-01/README.md).
You can also [run the software checks](#run-the-checks-without-a-robot) on
a Linux computer without a robot.

## How it works

1. **Find the robot.** Two infrared cameras, a motion sensor, and wheel
   readings help the robot estimate where it is and which way it’s facing.
2. **Read the room.** The depth camera measures how far away things are.
   The picture model recognizes objects such as people and chairs. The
   software uses the ground’s shape to find the floor.
3. **Make a map and plan.** The robot records what it sees in 5 cm cubes.
   It uses a top-down view of that map to plan a route around obstacles.
   Movable marks can clear as the room changes. A stale map blocks navigation.

Objects the model misses, such as ordinary boxes, normally count as fixed
obstacles when seen by the depth camera.

## Set up a robot

The main setup uses an **NVIDIA Jetson Orin Nano 8GB**, a **RealSense D435i**
USB camera, and a two-wheel base that reports wheel motion. Use a stable
power supply and a fan. A Raspberry Pi 5 can use the slower CPU setup
described in the deployment guide.

1. Install the software and check that the camera and motion sensor stream
   data. [Board and camera setup](stage-01/README.md)
2. Measure how the sensors line up and how their clocks differ.
   [Camera calibration](stage-02/README.md)
3. Start tracking, view the map, and test route planning. The drive output
   starts disabled. Verify the emergency stop and motor timeout before a
   supervised drive. [Deployment and troubleshooting](DEPLOYMENT.md)

## Run the checks without a robot

On Linux with Bash and Python 3.10–3.12, run these commands inside the
cloned folder:

```bash
python3 -m venv .venv-calibration
source .venv-calibration/bin/activate
python -m pip install PyYAML==6.0.3 numpy==1.26.4
python -m unittest discover -s tests -v
```

GitHub checks build the tracking software, exercise mapping with example
sensor data, and verify that navigation blocks a stale map. Camera
calibration, speed, drift, power, and stopping behavior still need to be
measured on your robot. [GitHub Actions](https://github.com/dodo434/semantic-mapping/actions)

## License

See the [project license](LICENSE). Third-party tools and model weights
come with their own license terms.
