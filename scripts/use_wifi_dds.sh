#!/usr/bin/env bash
# Source this file before starting ROS 2 nodes to force CycloneDDS onto WiFi.
# Usage:
#   source ~/agv_mpc/scripts/use_wifi_dds.sh
#   source ~/agv_mpc/scripts/use_wifi_dds.sh wlP1p1s0

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Run with: source ${0} [wifi_interface]" >&2
  exit 1
fi

_wifi_iface="${1:-}"

if [[ -z "${_wifi_iface}" ]]; then
  _wifi_iface="$(ip -o link show | awk -F': ' '$2 ~ /^(wl|wlan|wlp|wlx)/ {print $2; exit}')"
fi

if [[ -z "${_wifi_iface}" ]]; then
  echo "No WiFi interface found. Pass it explicitly, for example:" >&2
  echo "  source ~/agv_mpc/scripts/use_wifi_dds.sh wlP1p1s0" >&2
  return 1
fi

if ! ip link show "${_wifi_iface}" >/dev/null 2>&1; then
  echo "Interface not found: ${_wifi_iface}" >&2
  return 1
fi

_dds_file="/tmp/cyclonedds_${USER}_${_wifi_iface}.xml"

cat > "${_dds_file}" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="${_wifi_iface}" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_dds_file}"
unset FASTRTPS_DEFAULT_PROFILES_FILE

echo "CycloneDDS WiFi interface: ${_wifi_iface}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
