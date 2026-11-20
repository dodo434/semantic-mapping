#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "$0")/.." && pwd)
install_dir=${1:-"$project/artifacts/deps"}
jobs=${JOBS:-2}
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { printf 'JOBS must be positive\n' >&2; exit 2; }
mkdir -p -- "$install_dir"
install_dir=$(cd -- "$install_dir" && pwd)

checkout() {
  local url=$1 destination=$2 version=$3
  if [[ ! -d $destination/.git ]]; then
    git clone -- "$url" "$destination"
  fi
  if [[ $(git -C "$destination" rev-parse HEAD) != "$version" ]]; then
    git -C "$destination" checkout --detach "$version"
  fi
}

pangolin="$install_dir/Pangolin"
orb="$install_dir/ORB_SLAM3"
prefix="$install_dir/install"
checkout https://github.com/stevenlovegrove/Pangolin.git "$pangolin" aff6883c83f3fd7e8268a9715e84266c42e2efe3
checkout https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$orb" 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
patch="$project/patches/orb-slam3-semantic.patch"
if ! git -C "$orb" apply --reverse --check "$patch" 2>/dev/null; then
  git -C "$orb" apply --check "$patch"
  git -C "$orb" apply "$patch"
fi

cmake -S "$pangolin" -B "$pangolin/build" -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_INSTALL_PREFIX=$prefix" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build "$pangolin/build" --parallel "$jobs"
cmake --install "$pangolin/build"

for dependency in DBoW2 g2o; do
  cmake -S "$orb/Thirdparty/$dependency" -B "$orb/Thirdparty/$dependency/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$orb/Thirdparty/$dependency/build" --parallel "$jobs"
done
cmake -S "$orb" -B "$orb/build" -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=$prefix"
cmake --build "$orb/build" --target ORB_SLAM3 --parallel "$jobs"
tar -xf "$orb/Vocabulary/ORBvoc.txt.tar.gz" -C "$orb/Vocabulary"
printf 'ORB_SLAM3_ROOT=%s\nPangolin install prefix=%s\n' "$orb" "$prefix"
