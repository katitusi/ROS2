# Igus ReBeL 6DOF Simulation in Docker

## Voraussetzungen
1. **Docker Desktop** muss installiert und gestartet sein.
2. **VcXsrv (XLaunch)** für die grafische Ausgabe (RViz) unter Windows.

## VcXsrv Konfiguration
1. Starten Sie **XLaunch**.
2. Wählen Sie "Multiple windows".
3. "Start no client".
4. **Wichtig**: Aktivieren Sie **"Disable access control"**.
5. Klicken Sie auf Finish.

## Simulation starten
Öffnen Sie ein Terminal im Ordner `ROS2` und führen Sie folgenden Befehl aus:

```powershell
docker-compose run --rm ros2-dev bash -c 'source install/setup.bash && ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware'
```

## Steuerung des Roboters
1. Nach dem Start öffnet sich das **RViz** Fenster.
2. Sie sehen das 3D-Modell des Roboters.
3. Nutzen Sie das **MotionPlanning** Plugin (normalerweise unten links).
4. Im Reiter **Planning**:
   - Ziehen Sie die Kugel am Ende des Roboters (End-Effector) an die gewünschte Position.
   - Klicken Sie auf **Plan**, um die Trajektorie zu berechnen.
   - Klicken Sie auf **Execute**, um die Bewegung auszuführen.

## Beenden
Um die Simulation zu stoppen, drücken Sie `Strg+C` im Terminal oder führen Sie in einem neuen Fenster aus:
```powershell
docker stop $(docker ps -q --filter ancestor=ros2-workspace:dev)
```
