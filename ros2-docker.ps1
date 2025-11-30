# ============================================
# PowerShell-Skripte für ROS2 Docker-Verwaltung
# Verwenden Sie diese Befehle in PowerShell unter Windows
# ============================================
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass  
# Images bauen
function Build-ROS2 {
    docker-compose build
}

# Dev-Container starten
function Start-ROS2Dev {
    docker-compose run --rm ros2-dev
}

# Demo talker/listener starten
function Start-ROS2Demo {
    docker-compose up talker listener
}

# Container stoppen
function Stop-ROS2 {
    docker-compose down
}

# Logs anzeigen
function Show-ROS2Logs {
    docker-compose logs -f
}

# In Container eintreten
function Enter-ROS2Shell {
    docker exec -it ros2-dev bash
}

# Igus ReBeL Setup
function Setup-IgusRebel {
    docker-compose run --rm ros2-dev bash -c "chmod +x /ws/setup_igus.sh && /ws/setup_igus.sh"
}

# Alles bereinigen
function Remove-ROS2 {
    docker-compose down -v --rmi all
}

# Funktionen exportieren
Write-Host "ROS2 Docker Befehle geladen!" -ForegroundColor Green
Write-Host ""
Write-Host "Verfügbare Befehle:" -ForegroundColor Cyan
Write-Host "  Build-ROS2        - Docker Images bauen"
Write-Host "  Start-ROS2Dev     - Dev-Container starten"
Write-Host "  Start-ROS2Demo    - Demo talker/listener starten"
Write-Host "  Setup-IgusRebel   - Igus ReBeL Pakete herunterladen und einrichten"
Write-Host "  Stop-ROS2         - Alle Container stoppen"
Write-Host "  Show-ROS2Logs     - Logs anzeigen"
Write-Host "  Enter-ROS2Shell   - In Dev-Container eintreten"
Write-Host "  Remove-ROS2       - Alle Container und Images entfernen"
Write-Host ""
Write-Host "Verwendung:" -ForegroundColor Yellow
Write-Host "  . .\ros2-docker.ps1    # Befehle laden"
Write-Host "  Setup-IgusRebel        # Roboter installieren"

