# 🐳 Docker для ROS2 - Подробная документация

## 📋 Содержание

1. [Обзор](#обзор)
2. [Быстрый старт](#быстрый-старт)
3. [Архитектура](#архитектура)
4. [Сетевая конфигурация](#сетевая-конфигурация)
5. [GPU поддержка](#gpu-поддержка)
6. [Multi-platform сборка](#multi-platform-сборка)
7. [Troubleshooting](#troubleshooting)

---

## Обзор

Проект использует **multi-stage Dockerfile** для оптимизации размера образов и разделения dev/production окружений.

### Образы

- **base** (200MB) - минимальный ROS2 + CycloneDDS
- **dev** (400MB) - development tools + ccache
- **builder** - промежуточный для сборки workspace
- **runtime** (250MB) - production образ

---

## Быстрый старт

### Windows (PowerShell)

```powershell
# Загрузить helper функции
. .\ros2-docker.ps1

# Запустить demo
Start-ROS2Demo

# Или войти в dev контейнер
Start-ROS2Dev
```

### Linux/Mac

```bash
# Запустить demo
docker-compose up talker listener

# Или использовать скрипт
./start.sh
```

---

## Архитектура

### Multi-stage Build

```dockerfile
FROM ros:humble-ros-base AS base
  ↓ установка ROS пакетов
FROM base AS dev
  ↓ dev tools (ccache, gdb, vcstool)
FROM dev AS builder
  ↓ colcon build
FROM base AS runtime
  ↓ копирование install/
```

**Преимущества:**
- Маленький runtime образ
- Кэширование слоёв
- Быстрая разработка (dev stage)

---

## Сетевая конфигурация

### Linux - Host Network (рекомендуется)

```yaml
services:
  talker:
    network_mode: host
```

**Плюсы:**
- Multicast DDS работает из коробки
- Нет проблем с discovery

**Минусы:**
- Только для Linux

### Windows/Mac - Bridge Network

1. В `docker-compose.yml` раскомментируйте:

```yaml
networks:
  ros2_net:
    driver: bridge
```

2. В `cyclonedds.xml` добавьте:

```xml
<Peers>
  <Peer address="172.20.0.2"/>
  <Peer address="172.20.0.3"/>
</Peers>
```

3. Замените `network_mode: host` на:

```yaml
networks:
  - ros2_net
```

---

## GPU поддержка

### NVIDIA Docker (Linux)

1. Установите `nvidia-docker2`:

```bash
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

2. GPU уже настроен в `docker-compose.yml`:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu]
```

3. Запустите:

```bash
docker-compose run --rm ros2-dev
```

4. Проверьте GPU внутри:

```bash
nvidia-smi
```

### Для Gazebo/RViz

```bash
# Разрешить X11 forwarding
xhost +local:docker

# Запустить с GUI
docker-compose run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2-dev
```

---

## Multi-platform сборка

### ARM64 (Jetson, RaspberryPi, Apple Silicon)

```bash
# Создать buildx builder
docker buildx create --name ros2-builder --use

# Собрать для ARM64
docker buildx build \
  --platform linux/arm64 \
  --target runtime \
  -t your-registry/ros2:arm64 \
  --push .

# Или для обеих платформ
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t your-registry/ros2:latest \
  --push .
```

### Apple Silicon (M1/M2/M3)

На Mac с ARM процессором образы будут использовать native ARM64:

```bash
docker-compose build
# Автоматически соберёт для arm64
```

---

## Troubleshooting

### Проблема: Nodes не видят друг друга

**Причина:** Multicast блокируется в bridge network

**Решение:**
1. Используйте `network_mode: host` (Linux)
2. Или настройте CycloneDDS unicast (см. выше)
3. Или используйте ROS_DOMAIN_ID:

```yaml
environment:
  - ROS_DOMAIN_ID=42
```

### Проблема: Медленная сборка

**Решение:** Используйте ccache (уже настроен в dev образе)

```bash
# Внутри контейнера
export PATH="/usr/lib/ccache:$PATH"
colcon build
```

Volume `ccache` сохраняет кэш между запусками.

### Проблема: Port conflicts

**Решение:** Измените порты в `docker-compose.yml`:

```yaml
ports:
  - "11311:11311"  # ROS Master
  - "8080:8080"    # Ваш веб-сервер
```

### Проблема: GPU не работает

**Проверка:**

```bash
# На хосте
nvidia-smi

# В контейнере
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

Если не работает:
- Проверьте `nvidia-docker2` установлен
- Перезапустите Docker daemon
- Проверьте версию CUDA совместима

---

## Best Practices

### 1. Разработка с volume

Монтируйте только `src/`, не `build/` и `install/`:

```yaml
volumes:
  - ./src:/ws/src:rw
  - ./install:/ws/install:rw
```

### 2. Переменные окружения

Создайте `.env` файл:

```env
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
DISPLAY=:0
```

### 3. Логи

Сохраняйте логи вне контейнера:

```yaml
volumes:
  - ./log:/ws/log:rw
```

### 4. CI/CD

Используйте builder stage:

```yaml
# .gitlab-ci.yml
build:
  image: docker:latest
  script:
    - docker build --target builder -t ros2:builder .
    - docker run --rm ros2:builder colcon test
```

---

## Дополнительные ссылки

- [ROS2 Docker Official](https://hub.docker.com/_/ros)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
- [Docker Multi-stage](https://docs.docker.com/build/building/multi-stage/)
