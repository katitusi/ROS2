# ReBeL Safety Demo

Демонстрация системы безопасности для робота igus ReBeL с опциональной интеграцией LLM.

## Описание

Пакет `rebel_safety_demo` реализует простую систему безопасности "человек приближается → робот отступает":

- **human_distance_publisher**: Симулирует датчик расстояния до человека (2.0м → 0.2м за 20 секунд)
- **rebel_mover**: MoveIt-контроллер для управления позициями робота ReBeL
- **llm_safety_supervisor**: Супервизор безопасности с пороговой логикой и опциональной LLM интеграцией

## Структура пакета

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

## Сборка

```bash
cd /ws
colcon build --packages-select rebel_safety_demo
source install/setup.bash
```

## Запуск

### 1. Запустите симуляцию робота

В первом терминале:

```bash
docker-compose run --rm --service-ports ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
```

### 2. Запустите safety demo

Во втором терминале:

```bash
docker-compose exec ros2-dev bash
cd /ws
source install/setup.bash
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

## Логика работы

### Пороги безопасности

- **WARN_DISTANCE**: 1.0 м
- **DANGER_DISTANCE**: 0.6 м

### Поведение

- Расстояние > 1.0 м → робот в позиции HOME (нейтральная поза)
- Расстояние ≤ 0.6 м → робот в позиции SAFE_RETRACT (отступает назад)

### Позиции робота

- **HOME**: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` - нейтральная поза
- **SAFE_RETRACT**: `[0.0, -0.5, 0.8, 0.0, 0.3, 0.0]` - отступает назад

### Настройки безопасности

- Скорость: 10% от максимальной
- Ускорение: 10% от максимального

## Интеграция LLM (опционально)

Для подключения внешнего LLM установите переменную окружения:

```bash
export LLM_ENDPOINT="http://localhost:5000/llm/decide"
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

### Формат запроса к LLM

POST запрос:
```json
{
  "distance_m": 0.8,
  "state": "HUMAN_CLOSE"
}
```

### Ожидаемый ответ от LLM

```json
{
  "command": "SAFE_RETRACT"
}
```

или

```json
{
  "command": "HOME"
}
```

### Возможные варианты LLM endpoint

- Telegram-бот
- Локальный LLaMA
- OpenAI API прокси
- Любой HTTP сервис

## Топики и сервисы

### Топики

- `/human_distance` (`std_msgs/Float32`) - расстояние до человека в метрах

### Сервисы

- `/rebel_safety_demo/set_mode` (`std_srvs/SetBool`) - управление режимом робота
  - `data=true` → SAFE_RETRACT
  - `data=false` → HOME

## Запуск на реальном роботе

⚠️ **ВНИМАНИЕ**: Для работы с реальным роботом!

```bash
# Запустите hardware interface
ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2

# В другом терминале запустите demo
ros2 launch rebel_safety_demo safety_demo_real.launch.py
```

## Тестирование

### Просмотр расстояния

```bash
ros2 topic echo /human_distance
```

### Ручной вызов сервиса

```bash
# Переместить в SAFE_RETRACT
ros2 service call /rebel_safety_demo/set_mode std_srvs/srv/SetBool "{data: true}"

# Переместить в HOME
ros2 service call /rebel_safety_demo/set_mode std_srvs/srv/SetBool "{data: false}"
```

## Зависимости

- `rclpy`
- `std_msgs`
- `std_srvs`
- `moveit_commander`
- `moveit_ros_planning_interface`
- `requests` (для LLM интеграции, опционально)

## Лицензия

Apache-2.0
