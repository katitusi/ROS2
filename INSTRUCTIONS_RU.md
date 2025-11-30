# Симуляция Igus ReBeL 6DOF в Docker

## Предварительные требования
1. **Docker Desktop** должен быть установлен и запущен.
2. **VcXsrv (XLaunch)** для отображения графического интерфейса (RViz) на Windows.

## Настройка VcXsrv
1. Запустите **XLaunch**.
2. Выберите "Multiple windows".
3. "Start no client".
4. **Важно**: Поставьте галочку **"Disable access control"**.
5. Нажмите Finish.

## Запуск симуляции
Откройте терминал в папке `ROS2` и выполните команду:

```powershell
docker-compose run --rm ros2-dev bash -c 'source install/setup.bash && ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware'
```

## Управление роботом
1. После запуска откроется окно **RViz**.
2. Вы увидите 3D модель робота.
3. Используйте плагин **MotionPlanning** (обычно слева внизу).
4. Во вкладке **Planning**:
   - Перетащите шар на конце робота (end-effector) в желаемое положение.
   - Нажмите **Plan**, чтобы увидеть траекторию.
   - Нажмите **Execute**, чтобы робот выполнил движение.

## Остановка
Чтобы остановить симуляцию, нажмите `Ctrl+C` в терминале или выполните в другом окне:
```powershell
docker stop $(docker ps -q --filter ancestor=ros2-workspace:dev)
```
