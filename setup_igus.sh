#!/bin/bash
set -e

echo "🤖 Igus ReBeL ROS2 Pakete werden eingerichtet..."

cd /ws/src

# Repository klonen, falls es nicht existiert
if [ ! -d "iRC_ROS" ]; then
    git clone https://github.com/CommonplaceRobotics/iRC_ROS.git
    echo "✅ iRC_ROS Repository geklont"
else
    echo "⚠️ iRC_ROS Repository existiert bereits"
fi

# Abhängigkeiten installieren
echo "📦 Installiere Abhängigkeiten..."
cd /ws
apt-get update
# Fehlende Abhängigkeiten manuell installieren, um Build-Erfolg sicherzustellen
apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-launch-param-builder
rosdep update
# warehouse_ros_mongo überspringen, da es oft Installationsprobleme verursacht und optional für Demos ist
rosdep install --from-paths src --ignore-src -r -y --skip-keys "warehouse_ros_mongo"

echo "🔨 Baue Workspace..."
colcon build --symlink-install

echo "✅ Einrichtung abgeschlossen! Vergessen Sie nicht, die Setup-Datei zu sourcen:"
echo "source install/setup.bash"
