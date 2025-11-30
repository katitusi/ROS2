@echo off
echo ========================================
echo   Dance Demo - Igus ReBeL
echo   30-Second Robot Choreography
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

REM Check if pymoveit2 is installed
echo [INFO] Checking pymoveit2 installation...
docker exec %CONTAINER_ID% bash -c "dpkg -l | grep ros-humble-pymoveit2 > /dev/null 2>&1 && echo 'EXISTS' || echo 'MISSING'" > temp_check.txt
set /p PYMOVEIT_STATUS=<temp_check.txt
del temp_check.txt

if "%PYMOVEIT_STATUS%"=="MISSING" (
    echo [WARN] pymoveit2 not found. Installing...
    docker exec %CONTAINER_ID% bash -c "apt-get update && apt-get install -y ros-humble-pymoveit2"
    echo.
)

REM Check if rebel_dance_demo package exists
echo [INFO] Checking if rebel_dance_demo package is built...
docker exec %CONTAINER_ID% bash -c "test -d /ws/install/rebel_dance_demo && echo 'EXISTS' || echo 'MISSING'" > temp_check.txt
set /p PACKAGE_STATUS=<temp_check.txt
del temp_check.txt

if "%PACKAGE_STATUS%"=="MISSING" (
    echo [WARN] rebel_dance_demo package not found. Building...
    docker exec %CONTAINER_ID% bash -c "cd /ws && colcon build --packages-select rebel_dance_demo"
    echo.
)

echo [INFO] Starting Dance Demo...
echo.
echo ^>^>^> Choreography Phases:
echo     1. Opening - Greeting      (0-2s)
echo     2. Wave Motion             (2-8s)
echo     3. Figure-8 Pattern        (8-14s)
echo     4. Robot Twist             (14-20s)
echo     5. Grand Finale            (20-28s)
echo     6. Bow                     (28-30s)
echo.
echo ^>^>^> Note: Actual duration ~117 seconds
echo     (movements slowed to 30%% velocity for safety)
echo.
echo To visualize in Foxglove Studio:
echo     1. Open https://studio.foxglove.dev
echo     2. Connect to ws://localhost:9090
echo     3. Add 3D panel and watch the robot dance!
echo.
echo Dance will start automatically after launch.
echo Press Ctrl+C to stop the demo
echo ========================================
echo.

docker exec -it %CONTAINER_ID% bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch rebel_dance_demo dance_demo_sim.launch.py"
