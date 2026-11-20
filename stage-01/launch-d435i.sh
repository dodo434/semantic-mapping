#!/usr/bin/env bash
set -euo pipefail

if (( $# > 1 )); then
  printf 'Usage: bash %s [fused|raw]\n' "$0" >&2
  exit 2
fi

mode=${1-fused}
case "$mode" in
  fused) imu_method=2 ;;
  raw) imu_method=0 ;;
  *)
    printf 'Usage: bash %s [fused|raw]\n' "$0" >&2
    exit 2
    ;;
esac

if [[ ${ROS_DISTRO:-} != humble ]]; then
  printf 'Source /opt/ros/humble/setup.bash first.\n' >&2
  exit 1
fi

accel_fps=${ACCEL_FPS:-0}
if [[ ! $accel_fps =~ ^(0|[1-9][0-9]*)$ ]]; then
  printf 'ACCEL_FPS must be a nonnegative integer.\n' >&2
  exit 2
fi

export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera device_type:=d435i \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640,480,30 \
  depth_module.infra1_format:=Y8 depth_module.infra2_format:=Y8 \
  enable_color:=true rgb_camera.color_profile:=640,480,30 \
  enable_depth:=true depth_module.depth_profile:=640,480,30 \
  align_depth.enable:=true enable_sync:=true \
  enable_gyro:=true enable_accel:=true gyro_fps:=200 \
  "accel_fps:=$accel_fps" "unite_imu_method:=$imu_method" \
  publish_tf:=true pointcloud.enable:=false wait_for_device_timeout:=15.0
