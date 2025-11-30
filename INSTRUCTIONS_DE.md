# Igus ReBeL Simulation starten (Docker + Web Visualisierung)

Diese Methode verwendet **Foxglove Studio** (im Browser) zur Visualisierung. Dies vermeidet Probleme mit der X-Server-Konfiguration (VcXsrv) unter Windows.

## 1. Voraussetzungen
*   Installiertes und laufendes **Docker Desktop**.

## 2. Simulation starten
Öffnen Sie ein Terminal (PowerShell) im Projektordner und führen Sie folgenden Befehl aus:

```powershell
docker-compose run --rm --service-ports ros2-dev bash -c "./start_web_sim.sh"
```

*   Dieser Befehl startet den Container.
*   Startet die Robotersimulation (Mock Hardware).
*   Startet den `rosbridge` Server auf Port `9090`.

## 3. Visualisierung verbinden (Foxglove Studio)
1.  Öffnen Sie Ihren Browser (Chrome, Edge) und gehen Sie auf: [https://studio.foxglove.dev](https://studio.foxglove.dev)
2.  Klicken Sie auf **Open connection**.
3.  Wählen Sie den Verbindungstyp: **Rosbridge**.
4.  Geben Sie die Adresse ein: `ws://localhost:9090`
5.  **Wichtig**: Stellen Sie in den Verbindungseinstellungen (rechts) sicher, dass **Compression** auf **none** gesetzt ist.
6.  Klicken Sie auf **Open**.

## 4. Interface Einrichtung
1.  Fügen Sie ein **3D** Panel hinzu.
2.  In den 3D-Panel-Einstellungen:
    *   **Display Frame**: Wählen Sie `world`.
    *   Aktivieren Sie unter **Topics** das `Robot Model` (wird normalerweise aus `/robot_description` geladen).
3.  Sie sollten nun das 3D-Modell des Roboters sehen.

## 5. Beenden
Um die Simulation zu stoppen, drücken Sie `Strg+C` in dem Terminal, in dem Docker läuft.
