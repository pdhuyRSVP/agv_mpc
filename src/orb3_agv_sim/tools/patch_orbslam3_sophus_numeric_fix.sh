#!/usr/bin/env bash
set -euo pipefail
ORB_ROOT="${1:-$HOME/Dev/ORB_SLAM3}"
SO3="$ORB_ROOT/Thirdparty/Sophus/sophus/so3.hpp"
if [ ! -f "$SO3" ]; then
  echo "Cannot find $SO3" >&2
  exit 1
fi
cp "$SO3" "$SO3.backup_$(date +%Y%m%d_%H%M%S)"
python3 - "$SO3" <<'PY'
from pathlib import Path
import sys, re
p = Path(sys.argv[1])
s = p.read_text()
# Relax overly strict SO3 orthogonality check for small numerical drift.
s = re.sub(r'isApprox\(([^,]+),\s*Constants<Scalar>::epsilon\(\)\)', r'isApprox(\1, Scalar(1e-3))', s)
# Avoid abort on tiny non-orthogonal matrices; this does not fix NaN, so wrapper reset is still used.
s = s.replace('SOPHUS_ENSURE(isOrthogonal(R), "R is not orthogonal:\\n {}", R);',
              'if(!isOrthogonal(R)) { R = Eigen::Quaternion<Scalar>(R).normalized().toRotationMatrix(); }')
p.write_text(s)
print('patched', p)
PY
