# Решение проблемы с портом 9090 (Rosbridge)

## Проблема
При запуске симуляции командой:
```bash
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
```
Rosbridge НЕ запускается автоматически, поэтому порт 9090 не работает.

---

## ✅ Решение 1: Использовать готовый скрипт (РЕКОМЕНДУЕТСЯ)

### Вариант A: start_web_sim.sh
Запускает симуляцию + rosbridge + rosapi в одном процессе:

```bash
docker-compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"
```

### Вариант B: start_sim_with_bridge.sh
Альтернативный скрипт (создан только что):

```bash
docker-compose run --rm --service-ports ros2-dev bash -c "chmod +x ./start_sim_with_bridge.sh && ./start_sim_with_bridge.sh"
```

**Преимущества:**
- ✅ Всё в одной команде
- ✅ Автоматический cleanup при Ctrl+C
- ✅ Готов для Foxglove Studio

---

## ✅ Решение 2: Два терминала (ручной запуск)

### Терминал 1: Запуск симуляции
```bash
docker-compose run --rm --service-ports ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
```

### Терминал 2: Запуск rosbridge (в том же контейнере)
```powershell
# В новом окне PowerShell:
docker exec -it ros2-dev bash
source /opt/ros/humble/setup.bash

# Запустить rosbridge
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p address:=0.0.0.0 -p use_compression:=false
```

### Терминал 3: Запуск rosapi (опционально, для Foxglove)
```powershell
docker exec -it ros2-dev bash
source /opt/ros/humble/setup.bash
ros2 run rosapi rosapi_node
```

---

## ✅ Решение 3: Создать task в VS Code

Добавить в `.vscode/tasks.json`:

```json
{
    "label": "Run Simulation with Rosbridge",
    "type": "shell",
    "command": "docker-compose run --rm --service-ports ros2-dev bash -c './start_web_sim.sh'",
    "isBackground": true,
    "problemMatcher": []
}
```

---

## Проверка что rosbridge работает

### Способ 1: Проверить порт
```bash
# В Windows PowerShell:
Test-NetConnection localhost -Port 9090
```

### Способ 2: Проверить топики
```bash
# Внутри контейнера:
ros2 topic list | grep rosbridge
```

### Способ 3: Подключиться из браузера
1. Откройте https://studio.foxglove.dev
2. Выберите "Open connection" → "Rosbridge"
3. Введите `ws://localhost:9090`
4. Compression: **none**
5. Нажмите "Open"

Если подключение успешно - rosbridge работает! ✅

---

## Типичные ошибки

### Ошибка: "Connection refused" на порту 9090
**Причина:** Rosbridge не запущен  
**Решение:** Используйте один из методов выше

### Ошибка: "Cannot connect to rosbridge"
**Причина:** Порт не пробрасывается из контейнера  
**Решение:** Убедитесь, что используете флаг `--service-ports`:
```bash
docker-compose run --rm --service-ports ros2-dev bash
```

### Ошибка: Foxglove показывает "No topics"
**Причина:** rosapi не запущен  
**Решение:** Запустите rosapi:
```bash
ros2 run rosapi rosapi_node
```

---

## Быстрая шпаргалка

### Для safety_demo с веб-визуализацией:
```bash
# Терминал 1: Симуляция + rosbridge
docker-compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"

# Терминал 2: Safety demo
docker exec -it ros2-dev bash
cd /ws && source install/setup.bash
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

### Для dance_demo с веб-визуализацией:
```bash
# Терминал 1: Симуляция + rosbridge
docker-compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"

# Терминал 2: Dance demo
docker exec -it ros2-dev bash
cd /ws && source install/setup.bash
ros2 launch rebel_dance_demo dance_demo_sim.launch.py
```

---

## Структура запуска

```
start_web_sim.sh запускает:
├── rosbridge_websocket (порт 9090) [фон]
├── rosapi_node [фон]
└── rebel.launch.py (симуляция) [передний план]
```

При нажатии Ctrl+C все процессы корректно завершаются.
