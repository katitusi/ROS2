#!/bin/bash
set -e

echo "🤖 Setting up Igus ReBeL ROS2 packages..."

cd /ws/src

# Clone the repository if it doesn't exist
if [ ! -d "iRC_ROS" ]; then
    git clone https://github.com/CommonplaceRobotics/iRC_ROS.git
    echo "✅ Cloned iRC_ROS repository"
else
    echo "⚠️ iRC_ROS repository already exists"
fi

# Install dependencies
echo "📦 Installing dependencies..."
cd /ws
apt-get update
# Install missing dependencies manually to ensure build success
apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-launch-param-builder
rosdep update
# Skip warehouse_ros_mongo as it often causes installation issues and is optional for demos
rosdep install --from-paths src --ignore-src -r -y --skip-keys "warehouse_ros_mongo"

echo "🔨 Building workspace..."
colcon build --symlink-install

echo "✅ Setup complete! Don't forget to source the setup file:"
echo "source install/setup.bash"
