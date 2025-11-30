@echo off
echo ========================================
echo   Simple MoveIt Demo - Igus ReBeL
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

REM Check if simple_moveit_demo package exists
echo [INFO] Checking if rebel_demo package is built...
docker exec %CONTAINER_ID% bash -c "test -f /ws/install/rebel_demo/lib/rebel_demo/simple_moveit_demo && echo 'EXISTS' || echo 'MISSING'" > temp_check.txt
set /p PACKAGE_STATUS=<temp_check.txt
del temp_check.txt

if "%PACKAGE_STATUS%"=="MISSING" (
    echo [WARN] rebel_demo package not found. Building...
    docker exec %CONTAINER_ID% bash -c "cd /ws && colcon build --packages-select rebel_demo"
    echo.
)

echo [INFO] Starting Simple MoveIt Demo...
echo.
echo ^>^>^> Demo will launch with:
echo     - MoveIt2 motion planning
echo     - RViz visualization
echo     - Interactive markers for robot control
echo.
echo To visualize in Foxglove Studio:
echo     1. Open https://studio.foxglove.dev
echo     2. Connect to ws://localhost:9090
echo     3. Add 3D panel and enable Robot Model
echo.
echo Press Ctrl+C to stop the demo
echo ========================================
echo.

docker exec -it %CONTAINER_ID% bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch rebel_demo simple_moveit_demo.launch.py"
