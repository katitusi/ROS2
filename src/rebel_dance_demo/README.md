# ReBeL Dance Demo 🕺

30-sekündige Tanzchoreographie für den igus ReBeL-Roboter, inspiriert von Tänzen der Boston Dynamics-Roboter und industrieller Manipulatoren.

## Beschreibung

Das Paket `rebel_dance_demo` implementiert einen beeindruckenden 30-sekündigen Tanz für den 6-DOF-Robotermanipulator ReBeL mit fließenden Wellenbewegungen, Drehungen und dynamischen Kombinationen.

## Tanzstruktur

### 🎭 Phase 1: Begrüßung (0-2 Sek)
- Ausgangsposition
- Fließendes Winken nach oben
- Begrüßung des Publikums

### 🌊 Phase 2: Wellenbewegungen (2-8 Sek)
- Sinusförmige Bewegungen durch alle Gelenke
- "Wellen"-Effekt von der Basis zur Spitze
- 3 vollständige Zyklen fließender Übergänge

### ∞ Phase 3: Achterschleife (8-14 Sek)
- Endeffektor zeichnet eine Acht im Raum
- Fließende Bögen in der Luft
- 2 vollständige Achterzyklen

### 🌀 Phase 4: Verdrehung (14-20 Sek)
- Sequenzielle Rotation der Gelenke
- "Verdreh"-Effekt des Roboters
- Abwechselnde Bewegungen der Gelenke 1, 4, 6

### 💥 Phase 5: Großes Finale (20-28 Sek)
- Serie schneller dynamischer Posen
- 5 effektvolle Positionen
- Erhöhte Geschwindigkeit bis zu 50%
- Zusätzliche "Verzierungen"

### 🙇 Phase 6: Verbeugung (28-30 Sek)
- Rückkehr zur neutralen Pose
- Verbeugung vor dem "Publikum"
- Endposition

## Build

```bash
cd /ws
colcon build --packages-select rebel_dance_demo
source install/setup.bash
```

## Ausführung

### Simulation

**Terminal 1** - Robotersimulation starten:
```bash
docker-compose run --rm --service-ports ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
```

**Terminal 2** - Tanz starten:
```bash
docker-compose exec ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch rebel_dance_demo dance_demo_sim.launch.py
```

### Echter Roboter

⚠️ **ACHTUNG**: Stellen Sie sicher, dass der Arbeitsbereich frei ist!

**Terminal 1** - Hardware-Interface starten:
```bash
ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2
```

**Terminal 2** - Tanz starten:
```bash
ros2 launch rebel_dance_demo dance_demo_real.launch.py
```

## Manuelle Steuerung

Tanz über Service starten:
```bash
ros2 service call /rebel_dance_demo/start_dance std_srvs/srv/Trigger
```

## Merkmale

- **Dauer**: ~30 Sekunden
- **Anzahl der Phasen**: 6
- **Anzahl der Posen**: 15 einzigartige Positionen
- **Geschwindigkeit (Simulation)**: 30-50% der Maximalgeschwindigkeit
- **Geschwindigkeit (Echter Roboter)**: 20-40% der Maximalgeschwindigkeit
- **Flüssigkeit**: Interpolation zwischen allen Posen

## Technische Details

### Choreographische Posen

Alle Posen sind in Radiant für 6 Gelenke definiert:

```python
neutral = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]           # Neutral
greeting = [0.0, -0.8, 1.2, 0.0, 0.5, 0.0]         # Begrüßung
wave_1 = [0.3, -0.3, 0.6, 0.2, 0.4, 0.5]           # Welle 1
twist_1 = [1.0, -0.4, 0.7, 0.0, 0.3, 0.0]          # Verdrehung 1
finale_1 = [0.5, -0.9, 1.3, 0.8, 0.6, 1.0]         # Finale 1
bow = [0.0, 0.3, -0.3, 0.0, -0.2, 0.0]             # Verbeugung
# ... und weitere
```

### Sicherheitseinstellungen

**Für Simulation:**
- Geschwindigkeit: 30% (Flüssigkeit)
- Beschleunigung: 30%
- Finale-Teil: bis zu 50%

**Für echten Roboter:**
- Geschwindigkeit: 20% (Sicherheit)
- Beschleunigung: 20%
- Finale-Teil: bis zu 40%

## Visualisierung

### RViz

Um den Tanz in RViz zu beobachten:
```bash
ros2 launch irc_ros_moveit_config moveit_rviz.launch.py
```

### Foxglove Studio

1. Öffnen Sie [https://studio.foxglove.dev](https://studio.foxglove.dev)
2. Verbinden Sie sich mit `ws://localhost:9090` (rosbridge)
3. Fügen Sie ein 3D-Panel hinzu
4. Aktivieren Sie Robot Model

## API

### Services

- `/rebel_dance_demo/start_dance` (`std_srvs/srv/Trigger`)
  - Startet die Tanzsequenz
  - Gibt Erfolg/Misserfolg und Nachricht zurück

### Methoden (intern)

- `perform_dance()` - führt die vollständige Sequenz aus
- `move_to_pose(joints, velocity, accel)` - Bewegung zur Pose
- `interpolate_poses(start, end, steps)` - Interpolation zwischen Posen
- `log_progress(start_time)` - Protokollierung des Fortschritts

## Besonderheiten

✨ **Flüssigkeit** - Alle Bewegungen sind fließend ohne Ruckeln  
🎯 **Präzision** - Alle Posen sind sorgfältig abgestimmt  
⚡ **Dynamik** - Kombination aus langsamen und schnellen Bewegungen  
🔄 **Wiederholbarkeit** - Tanz ist jedes Mal identisch  
🛡️ **Sicherheit** - Begrenzte Geschwindigkeiten für echten Roboter  

## Inspiration

Der Tanz ist inspiriert von:
- Tänzen der Boston Dynamics-Roboter (Spot, Atlas)
- Choreographien von ABB- und KUKA-Industriemanipulatoren
- Klassischen Tanzbewegungen (Wellen, Achter, Drehungen)

## Anpassung

### Geschwindigkeit ändern

In der Datei `rebel_dancer.py` ändern:
```python
self.default_velocity = 0.5      # Schneller (50%)
self.default_acceleration = 0.5  # Schnellere Beschleunigung
```

### Eigene Posen hinzufügen

Im Wörterbuch `self.poses` hinzufügen:
```python
'my_pose': [joint1, joint2, joint3, joint4, joint5, joint6],
```

Und in der Sequenz verwenden:
```python
self.move_to_pose(self.poses['my_pose'])
```

### Sequenz ändern

Ändern Sie die Reihenfolge der Phasen in der Methode `perform_dance()`.

## Anforderungen

- ROS 2 Humble
- MoveIt2
- igus ReBeL-Roboter (Simulation oder echt)
- iRC_ROS-Pakete

## Abhängigkeiten

- `rclpy`
- `std_srvs`
- `moveit_commander`
- `moveit_ros_planning_interface`
- `geometry_msgs`

## Fehlerbehebung

### Roboter bewegt sich nicht

Prüfen Sie, ob Simulation/Hardware-Interface läuft:
```bash
ros2 topic list | grep joint
```

### Tanz wird unterbrochen

- Prüfen Sie, ob alle Posen erreichbar sind
- Reduzieren Sie die Geschwindigkeit
- Prüfen Sie die Logs auf Planungsfehler

### Bewegungen zu schnell/langsam

Ändern Sie `self.default_velocity` im Code.

## Sicherheit für echten Roboter

⚠️ **PFLICHT:**
- Freier Arbeitsbereich (mindestens 1,5m Radius)
- Zugänglicher Not-Aus-Knopf
- Beobachter neben dem Roboter
- Erster Lauf mit minimaler Geschwindigkeit (10-20%)
- Überprüfung der Trajektorien in der Simulation vor dem echten Roboter

## Leistung

- Trajektorienplanung: ~0,5-2 Sek pro Pose
- Gesamtausführungszeit: ~30-35 Sekunden
- CPU-Auslastung: moderat
- Speicherauslastung: gering

## Lizenz

Apache-2.0

## Autor

ROS2 Developer

---

**Viel Spaß beim Tanzen! 🎉🤖💃**
