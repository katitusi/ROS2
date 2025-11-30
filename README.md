# ROS2 Project 🤖

Проект на ROS2 (Robot Operating System 2) с полной поддержкой Docker.

## 🚀 Быстрый старт

### Docker (рекомендуется)

```bash
# Клонировать репозиторий
git clone https://github.com/katitusi/ROS2.git
cd ROS2

# Запустить demo talker/listener
docker-compose up talker listener

# Или войти в dev-контейнер
docker-compose run --rm ros2-dev
```

### Локальная установка

```bash
# Требования: ROS2 Humble, Python 3.8+, colcon

# Собрать пакеты
colcon build

# Активировать окружение
source install/setup.bash
```

---

## 📦 Docker Setup

### Архитектура

Проект использует **multi-stage Dockerfile**:

- **base** — минимальный ROS2 образ с CycloneDDS
- **dev** — development окружение с ccache и tools
- **builder** — stage для сборки workspace
- **runtime** — компактный production образ

### Запуск контейнеров

#### 1️⃣ Development контейнер (с volume)

```bash
docker-compose run --rm ros2-dev
```

Внутри контейнера:
```bash
cd /ws
colcon build
ros2 run <package> <node>
```

#### 2️⃣ Demo Talker/Listener

```bash
# Запустить оба узла
docker-compose up talker listener

# Или по отдельности
docker-compose up talker
docker-compose up listener
```

#### 3️⃣ Production runtime

```bash
docker-compose run --rm ros2-runtime
```

---

## ⚙️ Конфигурация

### CycloneDDS (DDS Middleware)

Проект использует **CycloneDDS** вместо Fast-DDS:

- Конфиг: `cyclonedds.xml`
- Поддержка multicast (Linux) и unicast (Windows/Mac)

**Для Windows/Mac**: раскомментируйте секцию `<Peers>` в `cyclonedds.xml`:

```xml
<Peers>
  <Peer address="172.20.0.2"/>
  <Peer address="172.20.0.3"/>
</Peers>
```

И используйте bridge network в `docker-compose.yml`.

### GPU Support (NVIDIA)

Для Gazebo/RViz с GPU:

```bash
# Требуется: nvidia-docker2
docker-compose run --rm ros2-dev
```

GPU активирован через `deploy.resources.reservations` в compose.

---

## 🏗️ Структура проекта

```
ROS2/
├── src/                    # Исходный код ROS2 пакетов
├── build/                  # Артефакты сборки (игнорируется)
├── install/                # Установленные пакеты (игнорируется)
├── log/                    # Логи (игнорируется)
├── Dockerfile              # Multi-stage Docker образ
├── docker-compose.yml      # Compose для dev/runtime
├── cyclonedds.xml          # Конфигурация DDS
├── .dockerignore           # Исключения для Docker build
└── README.md               # Документация
```

---

## 🛠️ Полезные команды

### Docker

```bash
# Собрать образы заново
docker-compose build

# Войти в running контейнер
docker exec -it ros2-dev bash

# Посмотреть логи
docker-compose logs -f talker

# Очистить volumes
docker-compose down -v
```

### ROS2

```bash
# Список нод
ros2 node list

# Список топиков
ros2 topic list

# Echo топика
ros2 topic echo /chatter

# Информация о ноде
ros2 node info /talker
```

---

## 🌍 Multi-platform build (ARM64 + x86_64)

```bash
# Создать builder
docker buildx create --use

# Собрать для ARM64 (Jetson/RaspberryPi)
docker buildx build --platform linux/arm64 -t ros2-workspace:arm64 .

# Собрать для обеих платформ
docker buildx build --platform linux/amd64,linux/arm64 -t your-registry/ros2:latest --push .
```

---

## 📚 Дополнительные ресурсы

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [CycloneDDS GitHub](https://github.com/eclipse-cyclonedds/cyclonedds)
- [Docker Multi-stage Builds](https://docs.docker.com/build/building/multi-stage/)

---

## 🤝 Contributing

1. Fork репозиторий
2. Создайте feature branch (`git checkout -b feature/amazing`)
3. Commit изменения (`git commit -m 'Add amazing feature'`)
4. Push в branch (`git push origin feature/amazing`)
5. Откройте Pull Request
