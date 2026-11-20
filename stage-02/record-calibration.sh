#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 2 ]]; then
  printf 'Usage: bash %s {stereo|camera-imu|allan} NEW_BAG_PATH\n' "$0" >&2
  exit 2
fi

camera=/camera/camera
stereo=("$camera/infra1/image_rect_raw" "$camera/infra2/image_rect_raw")
case "$1" in
  stereo) topics=("${stereo[@]}") ;;
  camera-imu) topics=("${stereo[@]}" "$camera/imu") ;;
  allan) topics=("$camera/imu") ;;
  *) printf 'Unknown capture: %s\n' "$1" >&2; exit 2 ;;
esac

if [[ ${ROS_DISTRO:-} != humble ]]; then
  printf 'Source /opt/ros/humble/setup.bash first.\n' >&2
  exit 1
fi

output=$2
if [[ -z $output || -e $output || -L $output ]]; then
  printf 'Provide a new bag path: %s\n' "$output" >&2
  exit 2
fi

umask 077
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
published_topics=$(ros2 topic list)
for topic in "${topics[@]}"; do
  if ! grep -Fxq -- "$topic" <<< "$published_topics"; then
    printf 'Missing topic: %s. Start the D435i in fused mode.\n' "$topic" >&2
    exit 1
  fi
done

exec ros2 bag record -s sqlite3 "--output=$output" "${topics[@]}"
