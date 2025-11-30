#!/bin/bash
# ============================================
# Quick start script for ROS2 Docker
# ============================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🤖 ROS2 Docker Quick Start${NC}"
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}❌ Docker не запущен!${NC}"
    exit 1
fi

echo -e "${YELLOW}Выберите действие:${NC}"
echo "1) Запустить dev контейнер"
echo "2) Запустить demo talker/listener"
echo "3) Собрать образы"
echo "4) Остановить всё"
echo ""
read -p "Ваш выбор (1-4): " choice

case $choice in
    1)
        echo -e "${GREEN}🚀 Запуск dev контейнера...${NC}"
        docker-compose run --rm ros2-dev
        ;;
    2)
        echo -e "${GREEN}🎤 Запуск demo talker/listener...${NC}"
        docker-compose up talker listener
        ;;
    3)
        echo -e "${GREEN}🔨 Сборка образов...${NC}"
        docker-compose build
        ;;
    4)
        echo -e "${YELLOW}🛑 Остановка контейнеров...${NC}"
        docker-compose down
        ;;
    *)
        echo -e "${RED}❌ Неверный выбор${NC}"
        exit 1
        ;;
esac
