/*
  user_config_override.h -- schlanker Build fuer den Dolphin-Poolroboter

  Tasmota bringt hunderte Sensortreiber, Displays, Infrarot, Energiezaehler,
  Regeln, KNX und Domoticz mit. Für diesen Anwendungsfall wird nichts davon
  gebraucht: Der ESP32 spricht Bluetooth mit dem Netzteil, führt ein
  Berry-Skript aus und meldet sich über MQTT.

  Der Grund für das Abspecken ist Arbeitsspeicher, nicht Flash. Mit dem
  vollen Bluetooth-Build blieben rund 22 bis 27 kB frei, und bei so wenig
  Rest wirft Tasmota als Erstes den Webserver ab -- die Oberfläche wird zäh
  und bricht weg, während das Gerät selbst weiterläuft.

  Behalten wird: WLAN, MQTT, Webserver, Dateisystem, Zeit, Berry und der
  BLE-Teil (MI32 legacy mit dem Berry-Modul BLE).
*/

#ifndef _USER_CONFIG_OVERRIDE_H_
#define _USER_CONFIG_OVERRIDE_H_

// -- Fremde Anbindungen: nicht gebraucht, wir sprechen MQTT --------------
#undef USE_DOMOTICZ
#undef USE_HOME_ASSISTANT
#undef USE_KNX
#undef USE_TELEGRAM
#undef USE_EMULATION
#undef USE_EMULATION_HUE
#undef USE_EMULATION_WEMO
#undef USE_ALEXA_AVS

// -- Automatisierung: dafuer ist Berry da --------------------------------
#undef USE_RULES
#undef USE_SCRIPT
#undef USE_EXPRESSION
#undef USE_TIMERS
#undef USE_TIMERS_WEB
#undef USE_SUNRISE

// -- Licht, Anzeigen, Motoren --------------------------------------------
#undef USE_LIGHT
#undef USE_WS2812
#undef USE_MY92X1
#undef USE_SM16716
#undef USE_SM2135
#undef USE_SM2335
#undef USE_BP1658CJ
#undef USE_BP5758D
#undef USE_PWM_DIMMER
#undef USE_SONOFF_L1
#undef USE_ELECTRIQ_MOODL
#undef USE_SHUTTER
#undef USE_DEVICE_GROUPS
#undef USE_BUZZER
#undef USE_DISPLAY
#undef USE_DISPLAY_LCD
#undef USE_DISPLAY_SSD1306
#undef USE_DISPLAY_MATRIX
#undef USE_DISPLAY_SEVENSEG
#undef USE_DISPLAY_TM1637

// -- Funkbruecken und Fremdprotokolle ------------------------------------
#undef USE_IR_REMOTE
#undef USE_IR_RECEIVE
#undef USE_SONOFF_RF
#undef USE_RF_FLASH
#undef USE_SONOFF_SC
#undef USE_TUYA_MCU
#undef USE_ARMTRONIX_DIMMERS
#undef USE_PS_16_DZ
#undef USE_SONOFF_IFAN
#undef USE_ZIGBEE
#undef USE_TASMESH
#undef USE_DALI
#undef USE_MATTER_DEVICE

// -- Energiezaehler -------------------------------------------------------
#undef USE_ENERGY_SENSOR
#undef USE_PZEM004T
#undef USE_PZEM_AC
#undef USE_PZEM_DC
#undef USE_MCP39F501
#undef USE_BL09XX
#undef USE_TELEINFO
#undef USE_SDM120
#undef USE_SDM630
#undef USE_DDS2382
#undef USE_DDSU666
#undef USE_SOLAX_X1
#undef USE_LE01MR
#undef USE_BL0940
#undef USE_IEM3000
#undef USE_WE517

// -- Sensorik: kein einziger Sensor haengt an diesem Geraet ---------------
#undef USE_I2C
#undef USE_SPI
#undef USE_DS18x20
#undef USE_DHT
#undef USE_MAX31855
#undef USE_MAX31865
#undef USE_MHZ19
#undef USE_SENSEAIR
#undef USE_SPS30
#undef USE_PMS5003
#undef USE_NOVA_SDS
#undef USE_HPMA
#undef USE_SR04
#undef USE_ME007
#undef USE_DYP
#undef USE_SERIAL_BRIDGE
#undef USE_TCP_BRIDGE
#undef USE_MP3_PLAYER
#undef USE_DFPLAYER
#undef USE_AZ7798
#undef USE_PN532_HSU
#undef USE_RDM6300
#undef USE_IBEACON
#undef USE_GPS
#undef USE_HM10
#undef USE_HRXL
#undef USE_TASMOTA_CLIENT
#undef USE_OPENTHERM
#undef USE_MIEL_HVAC
#undef USE_WIEGAND
#undef USE_AS608
#undef USE_MAGIC_SWITCH
#undef USE_LMT01
#undef USE_WINDMETER
#undef USE_THERMOSTAT
#undef USE_PROMETHEUS
#undef USE_INFLUXDB

// -- Sonstiges, das Speicher kostet --------------------------------------
#undef USE_SENDMAIL
#undef USE_ESP32MAIL
#undef USE_WEBCAM
#undef USE_I2S
#undef USE_I2S_AUDIO
#undef USE_SDCARD
#undef USE_UFILESYS_SDCARD
#undef USE_GPIO_VIEWER
#undef USE_BERRY_ULP
#undef USE_ETHERNET
#undef USE_ARDUINO_OTA
#undef USE_DEEPSLEEP
#undef USE_KEELOQ
#undef USE_LORAWAN_BRIDGE
#undef USE_SPI_LORA

/*
  Ausdruecklich NICHT abgeschaltet, weil gebraucht:
    USE_MQTT_*        -- der Poolroboter meldet sich am Pool-Strang
    USE_WEBSERVER     -- Bedienung und Diagnose
    USE_UFILESYS      -- die Berry-Dateien liegen im Dateisystem
    USE_BERRY         -- der Treiber selbst
    USE_MI_ESP32      -- BLE mit dem Berry-Modul `BLE`
    USE_NTP / Zeit    -- fuer timezone und real_time_clock des Netzteils
    USE_TASMOTA_DISCOVERY -- der Broker kennt das Geraet darueber
*/

#endif  // _USER_CONFIG_OVERRIDE_H_
