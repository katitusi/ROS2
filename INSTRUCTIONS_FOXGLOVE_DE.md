# Igus ReBeL Web-Visualisierung (Foxglove Studio)

Diese Methode ermöglicht die Steuerung des Roboters über den Browser, ohne Installation eines X-Servers (VcXsrv).

## Starten
1. Öffnen Sie ein Terminal im Ordner `ROS2`.
2. Führen Sie das Skript aus (beachten Sie das Flag `--service-ports`):
   ```powershell
   docker-compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"
   ```

## Foxglove Studio Konfiguration
1. Öffnen Sie Ihren Browser (Chrome/Edge) und gehen Sie auf [https://studio.foxglove.dev](https://studio.foxglove.dev).
2. Klicken Sie auf **Open connection**.
3. Wählen Sie **Rosbridge**.
4. URL: `ws://localhost:9090`.
5. Klicken Sie auf **Open**.

## Interface Einrichtung
1. Fügen Sie ein **3D** Panel hinzu.
2. In den Paneleinstellungen (rechts):
   - **Display Frame**: `world`
3. Sie sollten nun das Roboter-Modell sehen.

## Beenden
Drücken Sie `Strg+C` im Terminal.
