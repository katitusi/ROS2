# ============================================
# PowerShell скрипты для управления ROS2 Docker
# Используйте эти команды в PowerShell на Windows
# ============================================
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass  
# Собрать образы
function Build-ROS2 {
    docker-compose build
}

# Запустить dev контейнер
function Start-ROS2Dev {
    docker-compose run --rm ros2-dev
}

# Запустить demo talker/listener
function Start-ROS2Demo {
    docker-compose up talker listener
}

# Остановить контейнеры
function Stop-ROS2 {
    docker-compose down
}

# Показать логи
function Show-ROS2Logs {
    docker-compose logs -f
}

# Войти в контейнер
function Enter-ROS2Shell {
    docker exec -it ros2-dev bash
}

# Настройка Igus ReBeL
function Setup-IgusRebel {
    docker-compose run --rm ros2-dev bash -c "chmod +x /ws/setup_igus.sh && /ws/setup_igus.sh"
}

# Очистить всё
function Remove-ROS2 {
    docker-compose down -v --rmi all
}

# Экспортируем функции
Write-Host "ROS2 Docker команды загружены!" -ForegroundColor Green
Write-Host ""
Write-Host "Доступные команды:" -ForegroundColor Cyan
Write-Host "  Build-ROS2        - Собрать Docker образы"
Write-Host "  Start-ROS2Dev     - Запустить dev контейнер"
Write-Host "  Start-ROS2Demo    - Запустить demo talker/listener"
Write-Host "  Setup-IgusRebel   - Скачать и настроить Igus ReBeL пакеты"
Write-Host "  Stop-ROS2         - Остановить все контейнеры"
Write-Host "  Show-ROS2Logs     - Показать логи"
Write-Host "  Enter-ROS2Shell   - Войти в dev контейнер"
Write-Host "  Remove-ROS2       - Удалить все контейнеры и образы"
Write-Host ""
Write-Host "Использование:" -ForegroundColor Yellow
Write-Host "  . .\ros2-docker.ps1    # Загрузить команды"
Write-Host "  Setup-IgusRebel        # Установить робота"

