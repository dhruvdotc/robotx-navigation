#!/bin/bash
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=/ws/robotx-navigation/simulation/gazebo/models

# Force ign-transport to use loopback (unicast) — fixes multicast discovery
# failure inside Docker where multicast is often disabled or unreliable.
export IGN_IP=127.0.0.1
export GZ_IP=127.0.0.1

# Start bridge FIRST so its Gz subscriber is ready when Gazebo announces
ros2 run ros_gz_bridge parameter_bridge \
    '/model/iris_uav/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > /tmp/bridgex.log 2>&1 &
BRIDGE_PID=$!
sleep 2

# Now start Gazebo — bridge subscriber already listening
gz sim -r -s /ws/robotx-navigation/simulation/gazebo/worlds/robotx_docker_test.sdf > /tmp/gzx.log 2>&1 &
GZ_PID=$!
sleep 8

echo "=== gz odometry check ==="
timeout 3 gz topic -e -n 2 -t /model/iris_uav/odometry 2>&1 | grep "sec:" | head -3 || echo 'no gz msgs'

echo "=== bridge log ==="
cat /tmp/bridgex.log

echo "=== ros2 topic hz /clock (7s) ==="
timeout 9 ros2 topic hz /clock 2>&1 || true

echo "=== ros2 topic hz /odometry (7s) ==="
timeout 9 ros2 topic hz /model/iris_uav/odometry 2>&1 || true

kill $BRIDGE_PID $GZ_PID 2>/dev/null || true
echo "DONE"
