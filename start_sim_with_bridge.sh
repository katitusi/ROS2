#!/bin/bash
set -e

echo "🚀 Starting Complete Simulation Stack..."

# Source environment
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Function to handle cleanup
cleanup() {
    echo "🛑 Stopping all processes..."
    kill $(jobs -p) 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Start rosbridge in background
echo "🌐 Starting Rosbridge (port 9090)..."
ros2 run rosbridge_server rosbridge_websocket \
    --ros-args \
    -p port:=9090 \
    -p address:=0.0.0.0 \
    -p unregister_timeout:=9999999.0 \
    -p use_compression:=false &
ROSBRIDGE_PID=$!

# Start rosapi in background
echo "📡 Starting rosapi node..."
ros2 run rosapi rosapi_node &
ROSAPI_PID=$!

# Wait for rosbridge to start
sleep 3
echo "✅ Rosbridge ready at ws://localhost:9090"

# Start robot simulation
echo "🤖 Starting ReBeL Simulation..."
ros2 launch irc_ros_moveit_config rebel.launch.py \
    hardware_protocol:=mock_hardware \
    use_rviz:=false

# This line is reached when the launch file exits
cleanup
