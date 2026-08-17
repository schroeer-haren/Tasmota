# Dolphin-Poolroboter: Build und Patch

Dieser Zweig enthält einen Patch an der mitgelieferten NimBLE-Bibliothek,
damit ein Berry-Skript gleichzeitig BLE-Client und BLE-Server sein kann.

## Warum

Maytronics-Netzteile der Reihe *Dynamic IOT* kehren die üblichen Rollen um:
Die Verbindung baut der Controller auf (Central), die **Kommandos** erwartet
das Netzteil aber als Notification vom **lokalen GATT-Server** des
Controllers.

NimBLE liefert jedes Ereignis einer Verbindung ausschließlich an deren
Initiator. Bei einer selbst aufgebauten Verbindung landen deshalb auch
`BLE_GAP_EVENT_SUBSCRIBE` und `BLE_GAP_EVENT_NOTIFY_TX` in
`NimBLEClient::handleGapEvent` und fallen dort still in den Default-Zweig.

Folge ohne Patch: Der Server erfährt nie, dass der Peer abonniert hat, seine
Abonnentenliste bleibt leer, und jede Notification wird verworfen – ohne dass
irgendetwas fehlschlägt.

`NimBLEServer` enthält die Gegenrichtung bereits (`BLE_GAP_EVENT_NOTIFY_RX`
wird an den Client weitergereicht, wenn der Server die Verbindung hält).
Dieser Zweig ergänzt das Spiegelbild.

## Schlanker Build

`dolphin/user_config_override.h` schaltet ab, was dieses Gerät nicht braucht:
sämtliche Sensortreiber, Displays, Infrarot, Energiezähler, Licht- und
Rollladensteuerung, Regeln, Skripte, KNX, Domoticz, Zigbee und mehr. Behalten
werden WLAN, MQTT, Webserver, Dateisystem, Zeit, Berry und der BLE-Teil.

Der Grund ist Arbeitsspeicher, nicht Flash: Mit dem vollen Bluetooth-Build
blieben nur 22 bis 27 kB frei, und bei so wenig Rest wirft Tasmota als Erstes
den Webserver ab -- die Oberfläche wird zäh und bricht weg, während das Gerät
weiterläuft. Mit dieser Konfiguration sind es rund 34 kB, und die Firmware
schrumpft von 1,8 auf 1,5 MB.

Die Datei muss nach `tasmota/user_config_override.h` kopiert werden; dort ist
sie von Tasmota aus `.gitignore` ausgenommen, deshalb liegt sie hier.

## Bauen

Das Berry-Modul `BLE` mit Server-Rolle steckt in MI32 legacy und fehlt im
offiziellen `tasmota32-bluetooth.bin`, weil dort `USE_BLE_ESP32` gesetzt ist.
`-DUSE_MI_EXT_GUI` verhindert das und schaltet MI32 legacy scharf.

```bash
cp dolphin/platformio_tasmota_cenv.ini .
cp dolphin/user_config_override.h tasmota/
pio run -e tasmota32-dolphin
```

Danach am Gerät `SetOption115 1` (BLE einschalten) und `SetOption65 1`
(Schnellstart-Wiederherstellung aus, sonst löscht sie beim Entwickeln die
Einstellungen).

Das Berry-Skript, die Protokollschicht und die vollständige Dokumentation
liegen im Homelab-Repo unter `tasmota/dolphin/` und
`docs/dolphin-poolroboter.md`.
