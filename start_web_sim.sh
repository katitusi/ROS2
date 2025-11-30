#!/bin/bash
set -e

# Function to handle cleanup on exit
cleanup() {
    echo "Stopping processes..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT

echo "🚀 Starting Igus ReBeL Simulation with Web Visualization..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# 1. Start Rosbridge Server in the background
echo "🌐 Starting Rosbridge Server (ws://0.0.0.0:9090)..."
# Launch with explicit parameters to fix connection drops
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0 -p unregister_timeout:=9999999.0 &
PID_BRIDGE=$!

# Wait a moment for bridge to start
sleep 2

# 2. Start the Robot Simulation
echo "🤖 Starting Igus ReBeL Simulation (Mock Hardware)..."
# We use 'ros2 launch' directly. 
# Note: We don't need 'rviz' here if we are using Foxglove, but the launch file might start it.
# We can try to disable rviz if the launch file supports it, but rebel.launch.py seems to have 'use_rviz' arg.
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware use_rviz:=false &
PID_SIM=$!

echo "✅ System is running!"
echo "👉 Open https://studio.foxglove.dev in your browser."
echo "👉 Connect to 'Rosbridge' at ws://localhost:9090"
echo "Press Ctrl+C to stop."

# Wait for processes
wait $PID_SIM $PID_BRIDGE
