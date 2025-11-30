# 🤖 Igus ReBeL Simulationsleitfaden

Dieser Leitfaden hilft Ihnen, die Simulation des Igus ReBeL 6DOF-Roboters in Docker zu starten.

## 1. Vorbereitung (Windows)

Für die Ausführung grafischer Anwendungen (RViz, Gazebo) aus Docker unter Windows benötigen Sie einen X-Server.

1. **Laden Sie [VcXsrv](https://sourceforge.net/projects/vcxsrv/) herunter und installieren Sie es**.
2. **Starten Sie XLaunch** (wird mit VcXsrv mitgeliefert) mit folgenden Einstellungen:
   - **Display settings:** Multiple windows
   - **Client startup:** Start no client
   - **Extra settings:** ✅ **Disable access control** (Unbedingt aktivieren!)
   - Klicken Sie auf Finish.

## 2. Installation der Roboterpakete

Wir haben ein Skript für die automatische Installation vorbereitet.

1. Öffnen Sie PowerShell.
2. Laden Sie die Befehle und starten Sie die Installation:

```powershell
. .\ros2-docker.ps1
Build-ROS2          # Image mit GUI-Unterstützung neu bauen (dauert eine Weile)
Setup-IgusRebel     # Pakete herunterladen und Workspace bauen
```

## 3. Simulation starten

Jetzt können Sie die Demo starten.

1. Starten Sie den Dev-Container:
   ```powershell
   Start-ROS2Dev
   ```

2. Starten Sie im Container die MoveIt-Demo:
   ```bash
   ros2 launch irc_ros_moveit_config demo.launch.py
   ```

Wenn alles korrekt konfiguriert ist, sollte sich ein RViz-Fenster mit dem Robotermodell öffnen, in dem Sie Bewegungen planen können.

## 🛠️ Troubleshooting

### Fehler: "Can't open display"
Wenn sich RViz nicht öffnet:
1. Stellen Sie sicher, dass VcXsrv läuft.
2. Stellen Sie sicher, dass die Option "Disable access control" aktiviert wurde.
3. Versuchen Sie, Ihre IP-Adresse herauszufinden (Befehl `ipconfig` in PowerShell, suchen Sie nach dem WSL- oder Ethernet-Adapter) und setzen Sie die DISPLAY-Variable manuell, bevor Sie den Container starten:
   ```powershell
   $env:DISPLAY="IHRE_IP:0.0"
   Start-ROS2Dev
   ```

### Build-Fehler
Wenn `Setup-IgusRebel` mit einem Fehler abbricht, versuchen Sie, in den Container zu gehen und manuell zu bauen:
```bash
colcon build --packages-select irc_ros_moveit_config irc_ros_description
```
