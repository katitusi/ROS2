@echo off
echo ========================================
echo   Starting Igus ReBeL Simulation
echo   Docker + ROS2 + Foxglove
echo ========================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker is not running!
    echo Please start Docker Desktop first.
    echo.
    pause
    exit /b 1
)

echo [INFO] Starting ROS2 simulation container...
echo.
echo ^>^>^> Container will start:
echo     - Igus ReBeL robot simulation (mock hardware)
echo     - ROS2 Humble environment
echo     - Rosbridge server on port 9090
echo     - MoveIt2 motion planning
echo.
echo ^>^>^> After container starts:
echo     - Open https://studio.foxglove.dev in your browser
echo     - Connect to ws://localhost:9090
echo     - Add 3D panel to visualize robot
echo.
echo ^>^>^> To run demos, use in separate terminals:
echo     - start_simple_demo.bat  (Basic MoveIt control)
echo     - start_safety_demo.bat  (Human distance monitoring)
echo     - start_dance_demo.bat   (30-second choreography)
echo.
echo Press Ctrl+C to stop the simulation
echo ========================================
echo.

docker compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"
