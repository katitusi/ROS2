# 🐳 Docker für ROS2 - Detaillierte Dokumentation

## 📋 Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Schnellstart](#schnellstart)
3. [Architektur](#architektur)
4. [Netzwerkkonfiguration](#netzwerkkonfiguration)
5. [GPU-Unterstützung](#gpu-unterstützung)
6. [Multi-Platform-Build](#multi-platform-build)
7. [Troubleshooting](#troubleshooting)

---

## Überblick

Das Projekt verwendet ein **multi-stage Dockerfile** zur Optimierung der Image-Größe und Trennung von Dev/Production-Umgebungen.

### Images

- **base** (200MB) - minimales ROS2 + CycloneDDS
- **dev** (400MB) - Development Tools + ccache
- **builder** - Zwischenstufe für Workspace-Build
- **runtime** (250MB) - Production-Image

---

## Schnellstart

### Windows (PowerShell)

```powershell
# Helper-Funktionen laden
. .\ros2-docker.ps1

# Demo starten
Start-ROS2Demo

# Oder in den Dev-Container eintreten
Start-ROS2Dev
```

### Linux/Mac

```bash
# Demo starten
docker-compose up talker listener

# Oder Skript verwenden
./start.sh
```

---

## Architektur

### Multi-stage Build

```dockerfile
FROM ros:humble-ros-base AS base
  ↓ Installation von ROS-Paketen
FROM base AS dev
  ↓ dev tools (ccache, gdb, vcstool)
FROM dev AS builder
  ↓ colcon build
FROM base AS runtime
  ↓ Kopieren von install/
```

**Vorteile:**
- Kleines Runtime-Image
- Caching von Layern
- Schnelle Entwicklung (dev stage)

---

## Netzwerkkonfiguration

### Linux - Host Network (empfohlen)

```yaml
services:
  talker:
    network_mode: host
```

**Vorteile:**
- Multicast DDS funktioniert sofort
- Keine Probleme mit Discovery

**Nachteile:**
- Nur für Linux

### Windows/Mac - Bridge Network

1. In `docker-compose.yml` auskommentieren:

```yaml
networks:
  ros2_net:
    driver: bridge
```

2. In `cyclonedds.xml` hinzufügen:

```xml
<Peers>
  <Peer address="172.20.0.2"/>
  <Peer address="172.20.0.3"/>
</Peers>
```

3. Ersetzen Sie `network_mode: host` durch:

```yaml
networks:
  - ros2_net
```

---

## GPU-Unterstützung

### NVIDIA Docker (Linux)

1. Installieren Sie `nvidia-docker2`:

```bash
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

2. GPU ist bereits in `docker-compose.yml` konfiguriert:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu]
```

3. Starten:

```bash
docker-compose run --rm ros2-dev
```

4. GPU im Container überprüfen:

```bash
nvidia-smi
```

### Für Gazebo/RViz

```bash
# X11 Forwarding erlauben
xhost +local:docker

# Mit GUI starten
docker-compose run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2-dev
```

---

## Multi-Platform-Build

### ARM64 (Jetson, RaspberryPi, Apple Silicon)

```bash
# Buildx Builder erstellen
docker buildx create --name ros2-builder --use

# Für ARM64 bauen
docker buildx build \
  --platform linux/arm64 \
  --target runtime \
  -t your-registry/ros2:arm64 \
  --push .

# Oder für beide Plattformen
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t your-registry/ros2:latest \
  --push .
```

### Apple Silicon (M1/M2/M3)

Auf Mac mit ARM-Prozessor werden Images nativ für ARM64 gebaut:

```bash
docker-compose build
# Wird automatisch für arm64 gebaut
```

---

## Troubleshooting

### Problem: Nodes sehen sich nicht gegenseitig

**Ursache:** Multicast wird im Bridge Network blockiert

**Lösung:**
1. Verwenden Sie `network_mode: host` (Linux)
2. Oder konfigurieren Sie CycloneDDS Unicast (siehe oben)
3. Oder verwenden Sie ROS_DOMAIN_ID:

```yaml
environment:
  - ROS_DOMAIN_ID=42
```

### Problem: Langsamer Build

**Lösung:** Verwenden Sie ccache (bereits im Dev-Image konfiguriert)

```bash
# Im Container
export PATH="/usr/lib/ccache:$PATH"
colcon build
```

Volume `ccache` speichert den Cache zwischen Starts.

### Problem: Port-Konflikte

**Lösung:** Ändern Sie die Ports in `docker-compose.yml`:

```yaml
ports:
  - "11311:11311"  # ROS Master
  - "8080:8080"    # Ihr Webserver
```

### Problem: GPU funktioniert nicht

**Überprüfung:**

```bash
# Auf dem Host
nvidia-smi

# Im Container
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

Falls es nicht funktioniert:
- Prüfen Sie, ob `nvidia-docker2` installiert ist
- Starten Sie den Docker-Daemon neu
- Prüfen Sie, ob die CUDA-Version kompatibel ist

---

## Best Practices

### 1. Entwicklung mit Volume

Mounten Sie nur `src/`, nicht `build/` und `install/`:

```yaml
volumes:
  - ./src:/ws/src:rw
  - ./install:/ws/install:rw
```

### 2. Umgebungsvariablen

Erstellen Sie eine `.env`-Datei:

```env
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
DISPLAY=:0
```

### 3. Logs

Speichern Sie Logs außerhalb des Containers:

```yaml
volumes:
  - ./log:/ws/log:rw
```

### 4. CI/CD

Verwenden Sie die Builder-Stage:

```yaml
# .gitlab-ci.yml
build:
  image: docker:latest
  script:
    - docker build --target builder -t ros2:builder .
    - docker run --rm ros2:builder colcon test
```

---

## Zusätzliche Links

- [ROS2 Docker Official](https://hub.docker.com/_/ros)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
- [Docker Multi-stage](https://docs.docker.com/build/building/multi-stage/)
