#!/bin/bash
set -e

echo "🚀 Starte vollständigen Simulations-Stack..."

# Umgebung sourcen
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Funktion zum Bereinigen
cleanup() {
    echo "🛑 Stoppe alle Prozesse..."
    kill $(jobs -p) 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Rosbridge im Hintergrund starten
echo "🌐 Starte Rosbridge (Port 9090)..."
ros2 run rosbridge_server rosbridge_websocket \
    --ros-args \
    -p port:=9090 \
    -p address:=0.0.0.0 \
    -p unregister_timeout:=9999999.0 \
    -p use_compression:=false &
ROSBRIDGE_PID=$!

# rosapi im Hintergrund starten
echo "📡 Starte rosapi Node..."
ros2 run rosapi rosapi_node &
ROSAPI_PID=$!

# Warten, bis Rosbridge gestartet ist
sleep 3
echo "✅ Rosbridge bereit unter ws://localhost:9090"

# Roboter-Simulation starten
echo "🤖 Starte ReBeL Simulation..."
ros2 launch irc_ros_moveit_config rebel.launch.py \
    hardware_protocol:=mock_hardware \
    use_rviz:=false

# Diese Zeile wird erreicht, wenn die Launch-Datei beendet wird
cleanup
