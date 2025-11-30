#!/bin/bash
set -e

# Funktion zum Bereinigen beim Beenden
cleanup() {
    echo "Stoppe Prozesse..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT

echo "🚀 Starte Igus ReBeL Simulation mit Web-Visualisierung..."

# ROS2 Umgebung sourcen
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# 1. Rosbridge Server im Hintergrund starten
echo "🌐 Starte Rosbridge Server (ws://0.0.0.0:9090)..."
# Mit expliziten Parametern starten, um Verbindungsabbrüche zu beheben
# Tornado-Backend verwenden, das oft stabiler ist als Standard
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0 -p unregister_timeout:=9999999.0 -p use_compression:=false &
PID_BRIDGE=$!

# rosapi Node starten (erforderlich für Foxglove um Topics/Services aufzulisten)
echo "ℹ️ Starte rosapi Node..."
ros2 run rosapi rosapi_node &
PID_API=$!

# Kurz warten, bis Bridge gestartet ist
sleep 2

# 2. Roboter-Simulation starten
echo "🤖 Starte Igus ReBeL Simulation (Mock Hardware)..."
# Wir verwenden 'ros2 launch' direkt.
# Hinweis: Wir brauchen 'rviz' hier nicht, wenn wir Foxglove verwenden, aber die Launch-Datei könnte es starten.
# Wir können versuchen, rviz zu deaktivieren, wenn die Launch-Datei es unterstützt, rebel.launch.py scheint 'use_rviz' Argument zu haben.
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware use_rviz:=false &
PID_SIM=$!

echo "✅ System läuft!"
echo "👉 Öffnen Sie https://studio.foxglove.dev in Ihrem Browser."
echo "👉 Verbinden Sie sich mit 'Rosbridge' unter ws://localhost:9090"
echo "Drücken Sie Strg+C zum Stoppen."

# Auf Prozesse warten
wait $PID_SIM $PID_BRIDGE $PID_API
