@echo off
echo ========================================
echo   Safety Demo - Igus ReBeL
echo   Human Distance Monitoring System
echo ========================================
echo.

REM Find the running ROS2 container
for /f %%i in ('docker ps -q --filter "ancestor=ros2-workspace:dev"') do set CONTAINER_ID=%%i

if "%CONTAINER_ID%"=="" (
    echo [ERROR] No ROS2 container is running!
    echo.
    echo Please start the simulation first:
    echo   docker-compose up -d
    echo.
    pause
    exit /b 1
)

echo [OK] Found container: %CONTAINER_ID%
echo.

REM Check if rebel_safety_demo package exists
echo [INFO] Checking if rebel_safety_demo package is built...
docker exec %CONTAINER_ID% bash -c "test -d /ws/install/rebel_safety_demo && echo 'EXISTS' || echo 'MISSING'" > temp_check.txt
set /p PACKAGE_STATUS=<temp_check.txt
del temp_check.txt

if "%PACKAGE_STATUS%"=="MISSING" (
    echo [WARN] rebel_safety_demo package not found. Building...
    docker exec %CONTAINER_ID% bash -c "cd /ws && colcon build --packages-select rebel_safety_demo"
    echo.
)

echo [INFO] Starting Safety Demo...
echo.
echo ^>^>^> Demo Components:
echo     - Human Distance Publisher (simulates distance sensor)
echo     - ReBeL Mover (MoveIt2 controller with enable/disable)
echo     - LLM Safety Supervisor (monitors distance thresholds)
echo.
echo ^>^>^> Safety Thresholds:
echo     - WARN:   1.0m (warning logged)
echo     - DANGER: 0.6m (robot stopped automatically)
echo.
echo ^>^>^> Test Commands (in new terminal):
echo     docker exec -it %CONTAINER_ID% bash
echo     source /ws/install/setup.bash
echo     ros2 topic pub /human_distance std_msgs/msg/Float32 "data: 0.5" --once
echo.
echo To visualize in Foxglove Studio:
echo     1. Open https://studio.foxglove.dev
echo     2. Connect to ws://localhost:9090
echo     3. Add Plot panel for /human_distance topic
echo.
echo Press Ctrl+C to stop the demo
echo ========================================
echo.

docker exec -it %CONTAINER_ID% bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch rebel_safety_demo safety_demo_sim.launch.py"
