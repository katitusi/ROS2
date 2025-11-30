# Igus ReBeL Demo-Anwendungen - Anleitung (Deutsch)

Dieses Projekt enthält drei Demo-Anwendungen für den igus ReBeL 6DOF Roboter mit ROS2 Humble in Docker.

## 📋 Voraussetzungen
- **Docker Desktop** installiert und laufend
- **Foxglove Studio** (im Browser): [https://studio.foxglove.dev](https://studio.foxglove.dev)
- Mindestens 8 GB RAM
- Windows 10/11 mit PowerShell

## 🚀 Schritt 1: Simulation starten

**WICHTIG**: Vor dem Start einer Demo muss die Simulation gestartet werden!

### Methode 1: Doppelklick (einfachste Methode)
Doppelklick auf `start_sim.bat` im Projektordner

### Methode 2: PowerShell Befehl
```powershell
Start-Process -FilePath ".\start_sim.bat"
```

**Was wird gestartet:**
- Docker Container mit ROS2 Humble
- Igus ReBeL Simulation (mock hardware)
- Rosbridge Server (Port 9090 für Foxglove)
- MoveIt2 Motion Planning

**Warten Sie**, bis Sie die Meldung sehen:
```
✅ Rosbridge ready at ws://localhost:9090
🤖 Starting ReBeL Simulation...
```

---

## 🎯 Schritt 2: Demo auswählen und starten

### 1. **Simple MoveIt Demo** - Grundlegende Bewegungssteuerung
Einfache Demo zur Steuerung des Roboters über MoveIt2-Schnittstelle.

**Start (Methode 1 - Doppelklick):**
Doppelklick auf `start_simple_demo.bat`

**Start (Methode 2 - PowerShell):**
```powershell
Start-Process -FilePath ".\start_simple_demo.bat"
```

**Funktionen:**
- Interaktive Bewegungsplanung mit MoveIt2
- Visualisierung in RViz (im Container) oder Foxglove Studio
- Grundlegende Joint- und Cartesian-Steuerung

---

### 2. **Safety Demo** - Menschliche Distanzüberwachung
Demo zur Überwachung der menschlichen Nähe mit automatischer Roboter-Notabschaltung.

**Start (Methode 1 - Doppelklick):**
Doppelklick auf `start_safety_demo.bat`

**Start (Methode 2 - PowerShell):**
```powershell
Start-Process -FilePath ".\start_safety_demo.bat"
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

### 3. **Dance Demo** - 6-Sekunden ULTRA-Hochgeschwindigkeits-Choreographie ⚡
Vollständige Tanzchoreographie mit 6 Phasen (inspiriert von Boston Dynamics).

**Start (Methode 1 - Doppelklick):**
Doppelklick auf `start_dance_demo.bat`

**Start (Methode 2 - PowerShell):**
```powershell
Start-Process -FilePath ".\start_dance_demo.bat"
```

**Choreographie-Phasen:**
1. **Opening - Greeting** (0-0.4s): Blitz-Begrüßung
2. **Wave Motion** (0.4-1.6s): Rasante Wellenbewegungen
3. **Figure-8 Pattern** (1.6-2.8s): Schnelle Acht-Muster
4. **Robot Twist** (2.8-4s): Blitz-Drehung
5. **Grand Finale** (4-5.6s): ULTRA-SCHNELLE Kombination
6. **Bow** (5.6-6s): Express-Verbeugung

**Hinweis:** Tatsächliche Dauer ~24s (100% MAXIMUM Geschwindigkeit, 5x schneller als Original!).

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

## 🚀 Schnellstart (Zusammenfassung)

### Reihenfolge:
1. **Simulation starten**: Doppelklick auf `start_sim.bat` (oder PowerShell Befehl)
2. **Warten** bis "Rosbridge ready" erscheint
3. **Demo starten**: Doppelklick auf gewünschte Demo:
   - `start_simple_demo.bat` - Simple MoveIt Demo
   - `start_safety_demo.bat` - Safety Demo
   - `start_dance_demo.bat` - Dance Demo
4. **Visualisierung** (optional): Browser → `https://studio.foxglove.dev` → Connect zu `ws://localhost:9090`
5. **Beenden**: `Strg+C` im jeweiligen Batch-Fenster

### BAT-Dateien Speicherort:
```
.\
├── start_sim.bat              (1. Zuerst starten!)
├── start_simple_demo.bat      (2. Dann eine Demo wählen)
├── start_safety_demo.bat
└── start_dance_demo.bat
```

---

## 🔨 Docker Image Build

Wenn Sie das Docker-Image neu bauen oder aktualisieren möchten:

### Option 1: Mit docker-compose
```powershell
docker-compose build ros2-dev
```

### Option 2: Mit Build-ROS2 Funktion
```powershell
. .\ros2-docker.ps1
Build-ROS2
```

### Option 3: Direkt mit Docker
```powershell
docker build --target dev -t ros2-workspace:dev .
```

---

## 📄 Weitere Dokumentation
- [SAFETY_DEMO_TASK.md](SAFETY_DEMO_TASK.md) - Technische Spezifikation Safety Demo
- [DANCE_DEMO_TASK.md](DANCE_DEMO_TASK.md) - Technische Spezifikation Dance Demo
- [DOCKER_GUIDE.md](DOCKER_GUIDE.md) - Docker-spezifische Anleitung
- [IGUS_REBEL_GUIDE.md](IGUS_REBEL_GUIDE.md) - Igus ReBeL Hardware-Anleitung

---

**Viel Erfolg mit den Demos! 🤖**
