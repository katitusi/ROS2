# ============================================
# Упрощённый Makefile для ROS2 Docker
# ============================================

.PHONY: help build up down logs shell test clean

help: ## Показать эту справку
	@echo "Доступные команды:"
	@echo "  make build    - Собрать Docker образы"
	@echo "  make up       - Запустить dev контейнер"
	@echo "  make demo     - Запустить demo talker/listener"
	@echo "  make down     - Остановить все контейнеры"
	@echo "  make logs     - Показать логи"
	@echo "  make shell    - Войти в dev контейнер"
	@echo "  make clean    - Удалить все контейнеры и образы"

build: ## Собрать образы
	docker-compose build

up: ## Запустить dev контейнер
	docker-compose run --rm ros2-dev

demo: ## Запустить demo
	docker-compose up talker listener

down: ## Остановить контейнеры
	docker-compose down

logs: ## Показать логи
	docker-compose logs -f

shell: ## Войти в запущенный контейнер
	docker exec -it ros2-dev bash

clean: ## Очистить всё
	docker-compose down -v --rmi all
