# ReBeL Safety Demo

Demonstration eines Sicherheitssystems für den igus ReBeL-Roboter mit optionaler LLM-Integration.

## Beschreibung

Das Paket `rebel_safety_demo` implementiert ein einfaches Sicherheitssystem "Mensch nähert sich → Roboter weicht zurück":

- **human_distance_publisher**: Simuliert einen Distanzsensor zum Menschen (2,0m → 0,2m über 20 Sekunden)
- **rebel_mover**: MoveIt-Controller zur Steuerung der ReBeL-Roboterpositionen
- **llm_safety_supervisor**: Sicherheitssupervisor mit Schwellwertlogik und optionaler LLM-Integration

## Paketstruktur

```
rebel_safety_demo/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── rebel_safety_demo
├── rebel_safety_demo/
│   ├── __init__.py
│   ├── human_distance_publisher.py
│   ├── rebel_mover.py
│   └── llm_safety_supervisor.py
└── launch/
    ├── safety_demo_sim.launch.py
    └── safety_demo_real.launch.py
```

## Build

```bash
cd /ws
colcon build --packages-select rebel_safety_demo
source install/setup.bash
```

## Ausführung

### 1. Robotersimulation starten

Im ersten Terminal:

```bash
docker-compose run --rm --service-ports ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
```

### 2. Safety Demo starten

Im zweiten Terminal:

```bash
docker-compose exec ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

## Funktionslogik

### Sicherheitsschwellen

- **WARN_DISTANCE**: 1,0 m
- **DANGER_DISTANCE**: 0,6 m

### Verhalten

- Distanz > 1,0 m → Roboter in Position HOME (neutrale Pose)
- Distanz ≤ 0,6 m → Roboter in Position SAFE_RETRACT (weicht zurück)

### Roboterpositionen

- **HOME**: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` - neutrale Pose
- **SAFE_RETRACT**: `[0.0, -0.5, 0.8, 0.0, 0.3, 0.0]` - weicht zurück

### Sicherheitseinstellungen

- Geschwindigkeit: 10% der Maximalgeschwindigkeit
- Beschleunigung: 10% der Maximalbeschleunigung

## LLM-Integration (optional)

Um eine externe LLM anzubinden, setzen Sie die Umgebungsvariable:

```bash
export LLM_ENDPOINT="http://localhost:5000/llm/decide"
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

### Format der Anfrage an die LLM

POST-Anfrage:
```json
{
  "distance_m": 0.8,
  "state": "HUMAN_CLOSE"
}
```

### Erwartete Antwort von der LLM

```json
{
  "command": "SAFE_RETRACT"
}
```

oder

```json
{
  "command": "HOME"
}
```

### Mögliche LLM-Endpoint-Varianten

- Telegram-Bot
- Lokale LLaMA
- OpenAI API Proxy
- Beliebiger HTTP-Service

## Topics und Services

### Topics

- `/human_distance` (`std_msgs/Float32`) - Distanz zum Menschen in Metern

### Services

- `/rebel_safety_demo/set_mode` (`std_srvs/SetBool`) - Steuerung des Robotermodus
  - `data=true` → SAFE_RETRACT
  - `data=false` → HOME

## Ausführung auf echtem Roboter

⚠️ **ACHTUNG**: Für Arbeit mit echtem Roboter!

```bash
# Hardware-Interface starten
ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2

# In anderem Terminal Demo starten
ros2 launch rebel_safety_demo safety_demo_real.launch.py
```

## Testen

### Distanz anzeigen

```bash
ros2 topic echo /human_distance
```

### Service manuell aufrufen

```bash
# Nach SAFE_RETRACT bewegen
ros2 service call /rebel_safety_demo/set_mode std_srvs/srv/SetBool "{data: true}"

# Nach HOME bewegen
ros2 service call /rebel_safety_demo/set_mode std_srvs/srv/SetBool "{data: false}"
```

## Abhängigkeiten

- `rclpy`
- `std_msgs`
- `std_srvs`
- `moveit_commander`
- `moveit_ros_planning_interface`
- `requests` (für LLM-Integration, optional)

## Lizenz

Apache-2.0
