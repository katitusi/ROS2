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
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "🔨 Building workspace..."
colcon build --symlink-install

echo "✅ Setup complete! Don't forget to source the setup file:"
echo "source install/setup.bash"
