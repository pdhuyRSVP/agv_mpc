#!/usr/bin/env bash
set -euo pipefail
ORB_ROOT="${1:-$HOME/Dev/ORB_SLAM3}"

echo "[INFO] ORB_ROOT=$ORB_ROOT"

restore_latest() {
  local rel="$1"
  local file="$ORB_ROOT/$rel"
  local latest
  latest=$(ls -1t "$file".bak_planar_imu_* 2>/dev/null | head -n 1 || true)
  if [[ -n "$latest" ]]; then
    echo "[INFO] restoring $rel from $(basename "$latest")"
    cp "$latest" "$file"
  else
    echo "[WARN] no planar backup found for $rel"
  fi
}

restore_latest "Thirdparty/Sophus/sophus/so3.hpp"
restore_latest "src/Tracking.cc"
restore_latest "src/LocalMapping.cc"
restore_latest "src/Optimizer.cc"

echo "[INFO] done. Now build ORB-SLAM3 lightly:"
echo "  cd $ORB_ROOT/build && make -j1"
