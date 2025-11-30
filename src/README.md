# ROS2 Workspace Source

Эта директория содержит исходный код ROS2 пакетов.

## Структура

```
src/
├── your_package_1/
│   ├── package.xml
│   ├── setup.py
│   └── your_package_1/
│       ├── __init__.py
│       └── node.py
└── your_package_2/
    ├── package.xml
    ├── CMakeLists.txt
    └── src/
        └── node.cpp
```

## Создание нового пакета

### Python пакет

```bash
cd /ws/src
ros2 pkg create --build-type ament_python my_python_pkg
```

### C++ пакет

```bash
cd /ws/src
ros2 pkg create --build-type ament_cmake my_cpp_pkg
```

## Сборка

```bash
cd /ws
colcon build
source install/setup.bash
```

## Запуск

```bash
ros2 run <package_name> <node_name>
```
