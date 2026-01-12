# !/bin/bash
# SPDX-License-Identifier: MIT
set -e

source /opt/ros/*/setup.bash || true

ros2 run ros2_safety_monitor safety_node &
NODE_PID=$!

sleep 2

# テストケース: safe
RESULT=$(ros2 topic pub -1 /robot_position geometry_msgs/msg/Point "{x: 2.0, y: 3.0, z: 0.0}" \
  | ros2 topic echo -n 1 /safety_status | grep data | awk '{print $2}' | tr -d '"')

if [ "$RESULT" != "safe" ]; then
  echo "FAILED: (2,3) -> got '$RESULT', expected 'safe'"
  kill $NODE_PID || true
  exit 1
fi

echo "PASS: safe"

kill $NODE_PID || true
