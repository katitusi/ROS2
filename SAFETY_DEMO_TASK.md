# Техническое задание: ROS 2 пакет `rebel_safety_demo`

## Контекст
Вы работаете внутри Docker dev-контейнера с ROS 2 Humble + MoveIt2.

Рабочая область находится в `/ws` и уже собрана с помощью `colcon`.
Робот igus ReBeL доступен через пакеты `iRC_ROS` и конфигурацию MoveIt2:

- URDF + ros2_control загружаются из `irc_ros_description`
- Конфигурация MoveIt2: `irc_ros_moveit_config`
- Имя робота: `igus_rebel_6dof`
- Имя MoveIt группы: `rebel_6dof`
- Звено эффектора: `tcp`

## Предположение
Отдельный процесс уже запускает стек робота (симуляция или реальный робот), например:

- **Симуляция:**
  ```bash
  ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
  ```

- **Реальный робот:**
  ```bash
  ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2
  ```

## Задача
Создать ROS 2 Python пакет `rebel_safety_demo`, который реализует простое демо "человек приближается → робот отступает" с опциональным LLM-супервизором.

---

## Архитектура

### 1) Мок-издатель "расстояния до человека"
- **Имя ноды:** `human_distance_publisher`
- **Файл:** `rebel_safety_demo/human_distance_publisher.py`
- **Публикует в топик:** `/human_distance`
- **Тип сообщения:** `std_msgs/msg/Float32`
- **Частота:** 10 Hz
- **Поведение:**
  - Публикует расстояние в метрах
  - Симулирует паттерн:
    - Начинает с 2.0 м
    - Линейно уменьшается до 0.2 м за ~20 секунд
    - Сбрасывается обратно до 2.0 м и повторяется
  - Логирует текущее расстояние в консоль

### 2) MoveIt-контроллер движений для ReBeL
- **Имя ноды:** `rebel_mover`
- **Файл:** `rebel_safety_demo/rebel_mover.py`
- **Использует:** `rclpy` + `moveit_commander` (MoveIt2 Python интерфейс)
- **Создает:** `MoveGroupCommander` для группы `rebel_6dof`
- **Конфигурация:**
  - `set_max_velocity_scaling_factor(0.1)` (10% скорости)
  - `set_max_acceleration_scaling_factor(0.1)` (10% ускорения)
- **Определенные позы** (жестко заданные массивы из 6 значений углов сочленений):
  - `home_pose`: близко к нулям (безопасная нейтральная поза)
  - `safe_retract_pose`: робот отодвигается назад (например, плечевой сустав слегка назад, локоть вверх)
- **ROS 2 сервис:**
  - **Имя сервиса:** `/rebel_safety_demo/set_mode`
  - **Тип сервиса:** `std_srvs/srv/SetBool`
    - `request.data = True`  → переместиться в `safe_retract_pose`
    - `request.data = False` → переместиться в `home_pose`
  - **Реализация:**
    - Планирует и выполняет целевую позицию сочленений
    - Возвращает `success=true` если движение выполнено, `message` с текстом лога

### 3) LLM-супервизор безопасности
- **Имя ноды:** `llm_safety_supervisor`
- **Файл:** `rebel_safety_demo/llm_safety_supervisor.py`
- **Подписывается на:** `/human_distance` (`std_msgs/Float32`)
- **Вызывает сервис:** `/rebel_safety_demo/set_mode`
- **Внутренний конечный автомат** с тремя состояниями:
  - `IDLE`
  - `HUMAN_CLOSE`
  - `SAFE_RETRACTED`
- **Логика:**
  - Пороги: `WARN_DISTANCE = 1.0` м, `DANGER_DISTANCE = 0.6` м
  - Если расстояние > WARN_DISTANCE → желаемый режим = HOME (сервис SetBool с False)
  - Если расстояние <= DANGER_DISTANCE → желаемый режим = SAFE_RETRACT (сервис SetBool с True)
- **ОПЦИОНАЛЬНАЯ интеграция LLM:**
  - Читает переменную окружения `LLM_ENDPOINT` (строка URL). Если не задана, пропускает LLM и использует только пороги.
  - Если `LLM_ENDPOINT` задана:
    - При получении нового расстояния строит небольшой JSON payload:
      ```json
      { "distance_m": <float>, "state": "<current_state>" }
      ```
    - Отправляет HTTP POST (используя библиотеку `requests`) на `LLM_ENDPOINT`
    - Ожидает JSON ответ:
      ```json
      { "command": "HOME" }
      ```
      или
      ```json
      { "command": "SAFE_RETRACT" }
      ```
    - Маппинг:
      - `HOME`         → вызов сервиса с `data=False`
      - `SAFE_RETRACT` → вызов сервиса с `data=True`
    - Если HTTP вызов не удается или ответ невалидный, откат к простой пороговой логике
  - **Логирование:**
    - Каждое решение: печатает расстояние, текущее состояние, выбранную команду и использовался ли LLM или fallback

### 4) Launch файлы

#### `launch/safety_demo_sim.launch.py`
Запускает:
- `human_distance_publisher`
- `rebel_mover`
- `llm_safety_supervisor`

Цель: симуляция (mock hardware уже запущен в другом терминале)

#### `launch/safety_demo_real.launch.py`
- Те же три ноды
- Добавить заметное предупреждение в логе при запуске, что этот launch предназначен для РЕАЛЬНОГО робота, и скорости уже уменьшены

### 5) Структура пакета

Создать в `/ws/src/rebel_safety_demo`:

```
rebel_safety_demo/
├── package.xml
├── setup.py
├── setup.cfg
├── rebel_safety_demo/
│   ├── __init__.py
│   ├── human_distance_publisher.py
│   ├── rebel_mover.py
│   └── llm_safety_supervisor.py
└── launch/
    ├── safety_demo_sim.launch.py
    └── safety_demo_real.launch.py
```

Тип сборки: `ament_python`

### 6) Зависимости

В `package.xml` и `setup.py` добавить runtime зависимости:
- `rclpy`
- `std_msgs`
- `std_srvs`
- `moveit_commander`
- `moveit_ros_planning_interface`
- `requests` (для HTTP вызовов к LLM)

### 7) Entry points

В `setup.py` добавить console scripts:
```python
'console_scripts': [
    'rebel_human_distance_publisher = rebel_safety_demo.human_distance_publisher:main',
    'rebel_mover = rebel_safety_demo.rebel_mover:main',
    'llm_safety_supervisor = rebel_safety_demo.llm_safety_supervisor:main',
],
```

---

## Результат

После выполнения задания покажите полное содержимое:
- `rebel_safety_demo/human_distance_publisher.py`
- `rebel_safety_demo/rebel_mover.py`
- `rebel_safety_demo/llm_safety_supervisor.py`
- `launch/safety_demo_sim.launch.py`
- Соответствующие части:
  - `setup.py` (entry_points)
  - `package.xml` (dependencies)

---

## Шаги запуска

После создания пакета:

```bash
cd /ws
colcon build --packages-select rebel_safety_demo
source install/setup.bash

# Симуляция ReBeL уже запущена в другом терминале
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

## Интеграция LLM (опционально)

LLM можно подключить через переменную окружения `LLM_ENDPOINT`:

```bash
export LLM_ENDPOINT="http://localhost:5000/llm/decide"
ros2 launch rebel_safety_demo safety_demo_sim.launch.py
```

LLM endpoint должен принимать POST запрос с JSON:
```json
{
  "distance_m": 0.8,
  "state": "HUMAN_CLOSE"
}
```

И возвращать:
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

Это может быть:
- Telegram-бот
- Локальный LLaMA
- OpenAI-прокси
- Любой другой HTTP endpoint
