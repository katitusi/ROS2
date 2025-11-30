#!/bin/bash
# ============================================
# Schnellstart-Skript für ROS2 Docker
# ============================================

set -e

# Farben
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # Keine Farbe

echo -e "${GREEN}🤖 ROS2 Docker Schnellstart${NC}"
echo ""

# Überprüfen, ob Docker läuft
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}❌ Docker läuft nicht!${NC}"
    exit 1
fi

echo -e "${YELLOW}Wählen Sie eine Aktion:${NC}"
echo "1) Dev-Container starten"
echo "2) Demo talker/listener starten"
echo "3) Images bauen"
echo "4) Alles stoppen"
echo ""
read -p "Ihre Wahl (1-4): " choice

case $choice in
    1)
        echo -e "${GREEN}🚀 Starte Dev-Container...${NC}"
        docker-compose run --rm ros2-dev
        ;;
    2)
        echo -e "${GREEN}🎤 Starte Demo talker/listener...${NC}"
        docker-compose up talker listener
        ;;
    3)
        echo -e "${GREEN}🔨 Baue Images...${NC}"
        docker-compose build
        ;;
    4)
        echo -e "${YELLOW}🛑 Stoppe Container...${NC}"
        docker-compose down
        ;;
    *)
        echo -e "${RED}❌ Ungültige Wahl${NC}"
        exit 1
        ;;
esac
