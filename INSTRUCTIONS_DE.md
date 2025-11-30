# Igus ReBeL Demo-Anwendungen - Anleitung (Deutsch)

Dieses Projekt enthält drei Demo-Anwendungen für den igus ReBeL 6DOF Roboter mit ROS2 Humble in Docker.

## 📋 Voraussetzungen
- **Docker Desktop** installiert und laufend
- **Foxglove Studio** (im Browser): [https://studio.foxglove.dev](https://studio.foxglove.dev)
- Mindestens 8 GB RAM
- Windows 10/11 mit PowerShell

## 🎯 Verfügbare Demos

### 1. **Simple MoveIt Demo** - Grundlegende Bewegungssteuerung
Einfache Demo zur Steuerung des Roboters über MoveIt2-Schnittstelle.

**Start:**
```powershell
.\start_simple_demo.bat
```

**Funktionen:**
- Interaktive Bewegungsplanung mit MoveIt2
- Visualisierung in RViz (im Container) oder Foxglove Studio
- Grundlegende Joint- und Cartesian-Steuerung

---

### 2. **Safety Demo** - Menschliche Distanzüberwachung
Demo zur Überwachung der menschlichen Nähe mit automatischer Roboter-Notabschaltung.

**Start:**
```powershell
.\start_safety_demo.bat
```

**Komponenten:**
- **Human Distance Publisher**: Simuliert Distanzsensor (publiziert auf `/human_distance`)
- **ReBeL Mover**: MoveIt2-Controller mit Enable/Disable-Service
- **LLM Safety Supervisor**: Überwacht Distanz mit konfigurierbaren Schwellwerten

**Sicherheitsschwellwerte:**
- `WARN_THRESHOLD`: 1.0m (Warnung)
- `DANGER_THRESHOLD`: 0.6m (Roboter-Stopp)

**Testen:**
```bash
# Im Container (neues Terminal):
docker exec -it <container-id> bash
source /ws/install/setup.bash

# Distanz publizieren (z.B. 0.5m = Gefahr):
ros2 topic pub /human_distance std_msgs/msg/Float32 "data: 0.5" --once
```

---

### 3. **Dance Demo** - 30-Sekunden Choreographie
Vollständige Tanzchoreographie mit 6 Phasen (inspiriert von Boston Dynamics).

**Start:**
```powershell
.\start_dance_demo.bat
```

**Choreographie-Phasen:**
1. **Opening - Greeting** (0-2s): Begrüßungsgeste nach oben
2. **Wave Motion** (2-8s): Fließende Wellenbewegungen
3. **Figure-8 Pattern** (8-14s): Sanfte kreisförmige Muster
4. **Robot Twist** (14-20s): Vollständige Drehung
5. **Grand Finale** (20-28s): Schnelle Kombinationsbewegungen
6. **Bow** (28-30s): Abschluss-Verbeugung

**Hinweis:** Die tatsächliche Dauer beträgt ~117s (Bewegungen auf 30% Geschwindigkeit verlangsamt).

---

## 🖥️ Visualisierung mit Foxglove Studio

### Erstmaliges Setup:
1. Öffnen Sie [https://studio.foxglove.dev](https://studio.foxglove.dev) im Browser
2. Klicken Sie auf **Open connection**
3. Wählen Sie **Rosbridge** als Verbindungstyp
4. Adresse: `ws://localhost:9090`
5. **Wichtig**: Compression auf **none** setzen
6. Klicken Sie **Open**

### Interface einrichten:
1. **3D Panel** hinzufügen
2. In den Einstellungen:
   - Display Frame: `world`
   - Topics → Robot Model aktivieren
3. Optional: **Plot** Panel für `/human_distance` (Safety Demo)

---

## 🛠️ Manuelle Steuerung

### Container-ID finden:
```powershell
docker ps
```

### In Container einloggen:
```powershell
docker exec -it <container-id> bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

### Verfügbare Topics anzeigen:
```bash
ros2 topic list
```

### Safety Demo Services:
```bash
# Roboter aktivieren
ros2 service call /rebel_mover/enable std_srvs/srv/SetBool "data: true"

# Roboter deaktivieren
ros2 service call /rebel_mover/enable std_srvs/srv/SetBool "data: false"
```

### Dance Demo Service:
```bash
# Tanz starten (startet automatisch nach 2s)
ros2 service call /rebel_dance_demo/start_dance std_srvs/srv/Trigger
```

---

## 📦 Package-Struktur

```
ROS2/
├── src/
│   ├── rebel_demo/                  # Simple MoveIt Demo
│   │   └── simple_moveit_demo.py
│   ├── rebel_safety_demo/           # Safety Demo
│   │   ├── human_distance_publisher.py
│   │   ├── rebel_mover.py
│   │   └── llm_safety_supervisor.py
│   └── rebel_dance_demo/            # Dance Demo
│       └── rebel_dancer.py
├── start_simple_demo.bat            # Launcher Simple Demo
├── start_safety_demo.bat            # Launcher Safety Demo
└── start_dance_demo.bat             # Launcher Dance Demo
```

---

## 🔧 Fehlerbehebung

### Port 9090 bereits belegt:
```powershell
# Bestehenden Container stoppen
docker ps
docker stop <container-id>
```

### Roboter bewegt sich nicht:
- Überprüfen Sie, ob `/joint_states` publiziert wird:
  ```bash
  ros2 topic echo /joint_states --once
  ```
- Stellen Sie sicher, dass Joint-Namen korrekt sind (ohne Unterstriche: `joint1`, `joint2`, etc.)

### Docker-Image neu erstellen:
```powershell
docker-compose build
```

---

## 🚀 Schnellstart für alle Demos

1. **Simple Demo starten**: Doppelklick auf `start_simple_demo.bat`
2. **Safety Demo starten**: Doppelklick auf `start_safety_demo.bat`
3. **Dance Demo starten**: Doppelklick auf `start_dance_demo.bat`
4. **Foxglove öffnen**: Browser → `https://studio.foxglove.dev` → Connect zu `ws://localhost:9090`
5. **Beenden**: `Strg+C` im Batch-Fenster

---

## 📄 Weitere Dokumentation
- [SAFETY_DEMO_TASK.md](SAFETY_DEMO_TASK.md) - Technische Spezifikation Safety Demo
- [DANCE_DEMO_TASK.md](DANCE_DEMO_TASK.md) - Technische Spezifikation Dance Demo
- [DOCKER_GUIDE.md](DOCKER_GUIDE.md) - Docker-spezifische Anleitung
- [IGUS_REBEL_GUIDE.md](IGUS_REBEL_GUIDE.md) - Igus ReBeL Hardware-Anleitung

---

**Viel Erfolg mit den Demos! 🤖**
