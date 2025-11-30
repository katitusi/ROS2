# ROS2 Projekt 🤖

Projekt auf ROS2 (Robot Operating System 2) mit vollständiger Docker-Unterstützung.

## 🚀 Schnellstart

### Docker (empfohlen)

```bash
# Repository klonen
git clone https://github.com/katitusi/ROS2.git
cd ROS2

# Demo talker/listener starten
docker-compose up talker listener

# Oder in Dev-Container eintreten
docker-compose run --rm ros2-dev
```

### Lokale Installation

```bash
# Anforderungen: ROS2 Humble, Python 3.8+, colcon

# Pakete bauen
colcon build

# Umgebung aktivieren
source install/setup.bash
```

---

## 📦 Docker Setup

### Architektur

Das Projekt verwendet ein **Multi-Stage Dockerfile**:

- **base** — minimales ROS2 Image mit CycloneDDS
- **dev** — Entwicklungsumgebung mit ccache und tools
- **builder** — Stage für Workspace-Build
- **runtime** — kompaktes Production-Image

### Container starten

#### 1️⃣ Development Container (mit Volume)

```bash
docker-compose run --rm ros2-dev
```

Im Container:
```bash
cd /ws
colcon build
ros2 run <package> <node>
```

#### 2️⃣ Demo Talker/Listener

```bash
# Beide Nodes starten
docker-compose up talker listener

# Oder einzeln
docker-compose up talker
docker-compose up listener
```

#### 3️⃣ Production Runtime

```bash
docker-compose run --rm ros2-runtime
```

---

## ⚙️ Konfiguration

### CycloneDDS (DDS Middleware)

Das Projekt verwendet **CycloneDDS** anstelle von Fast-DDS:

- Konfiguration: `cyclonedds.xml`
- Unterstützung für Multicast (Linux) und Unicast (Windows/Mac)

**Für Windows/Mac**: Auskommentierung der `<Peers>` Sektion in `cyclonedds.xml` aufheben:

```xml
<Peers>
  <Peer address="172.20.0.2"/>
  <Peer address="172.20.0.3"/>
</Peers>
```

Und Bridge-Netzwerk in `docker-compose.yml` verwenden.

### GPU-Unterstützung (NVIDIA)

Für Gazebo/RViz mit GPU:

```bash
# Erforderlich: nvidia-docker2
docker-compose run --rm ros2-dev
```

GPU aktiviert über `deploy.resources.reservations` in compose.

---

## 🏗️ Projektstruktur

```
ROS2/
├── src/                    # Quellcode der ROS2-Pakete
├── build/                  # Build-Artefakte (ignoriert)
├── install/                # Installierte Pakete (ignoriert)
├── log/                    # Logs (ignoriert)
├── Dockerfile              # Multi-Stage Docker-Image
├── docker-compose.yml      # Compose für dev/runtime
├── cyclonedds.xml          # DDS-Konfiguration
├── .dockerignore           # Ausschlüsse für Docker Build
└── README.md               # Dokumentation
```

---

## 🛠️ Nützliche Befehle

### Docker

```bash
# Images neu bauen
docker-compose build

# In laufenden Container eintreten
docker exec -it ros2-dev bash

# Logs anzeigen
docker-compose logs -f talker

# Volumes bereinigen
docker-compose down -v
```

### ROS2

```bash
# Node-Liste
ros2 node list

# Topic-Liste
ros2 topic list

# Topic ausgeben
ros2 topic echo /chatter

# Node-Informationen
ros2 node info /talker
```

---

## 🌍 Multi-Platform Build (ARM64 + x86_64)

```bash
# Builder erstellen
docker buildx create --use

# Für ARM64 bauen (Jetson/RaspberryPi)
docker buildx build --platform linux/arm64 -t ros2-workspace:arm64 .

# Für beide Plattformen bauen
docker buildx build --platform linux/amd64,linux/arm64 -t your-registry/ros2:latest --push .
```

---

## 📚 Zusätzliche Ressourcen

- [ROS2 Dokumentation](https://docs.ros.org/en/humble/)
- [CycloneDDS GitHub](https://github.com/eclipse-cyclonedds/cyclonedds)
- [Docker Multi-Stage Builds](https://docs.docker.com/build/building/multi-stage/)

---

## 🤝 Mitwirken

1. Repository forken
2. Feature-Branch erstellen (`git checkout -b feature/amazing`)
3. Änderungen committen (`git commit -m 'Add amazing feature'`)
4. In Branch pushen (`git push origin feature/amazing`)
5. Pull Request öffnen
