#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 2 ]]; then
  printf 'Usage: bash %s {fused|raw} BAG_PATH\n' "$0" >&2
  exit 2
fi

camera=/camera/camera
case "$1" in
  fused) imu_topics=("$camera/imu") ;;
  raw) imu_topics=("$camera/gyro/sample" "$camera/accel/sample") ;;
  *)
    printf 'Unknown mode: %s\n' "$1" >&2
    exit 2
    ;;
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
topics=(
  "$camera/infra1/image_rect_raw" "$camera/infra1/camera_info"
  "$camera/infra2/image_rect_raw" "$camera/infra2/camera_info"
  "$camera/color/image_raw" "$camera/color/camera_info"
  "$camera/depth/image_rect_raw" "$camera/depth/camera_info"
  "$camera/aligned_depth_to_color/image_raw"
  "$camera/aligned_depth_to_color/camera_info"
  "${imu_topics[@]}"
  /tf_static
)

published_topics=$(ros2 topic list)
for topic in "${topics[@]}"; do
  if ! grep -Fxq -- "$topic" <<< "$published_topics"; then
    printf 'Missing topic: %s. Check the camera launch mode and log.\n' "$topic" >&2
    exit 1
  fi
done

exec ros2 bag record -s sqlite3 "--output=$output" "${topics[@]}"
