# !/bin/bash
# SPDX-License-Identifier: MIT

source ~/ros2_ws/install/setup.bash

ros2 run ros2_safety_monitor safety_node &
NODE_PID=$!
sleep 2  

test_case() {
  local x=$1
  local y=$2
  local expected=$3

  ros2 topic pub /robot_position geometry_msgs/msg/Point "{x: $x, y: $y, z: 0.0}" -1 --qos-reliability reliable >/dev/null 2>&1

  result=$(ros2 topic echo /safety_status -n1 | awk '{print $2}' | tr -d '"')

  if [ "$result" != "$expected" ]; then
    echo "FAILED: ($x, $y) -> got '$result', expected '$expected'"
    kill $NODE_PID
    exit 1
  else
    echo "PASSED: ($x, $y) -> '$result'"
  fi
}

test_case 2 3 safe
test_case 7 1 warning
test_case 10.5 0 danger

kill $NODE_PID
echo "All tests passed!"
exit 0
