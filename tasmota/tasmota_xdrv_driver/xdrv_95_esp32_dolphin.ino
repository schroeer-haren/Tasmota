/*
  xdrv_95_esp32_dolphin.ino - Maytronics Dolphin Plus Poolroboter ueber BLE

  Copyright (C) 2026 Philipp Schroeer

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Das Netzteil ("PWS") des Maytronics-Poolroboters verlangt vom Gegenueber
  BEIDE BLE-Rollen gleichzeitig, und das ist die Umkehrung der ueblichen
  Anordnung:

    Client : wir bauen die Verbindung auf und abonnieren fd5abba1 des PWS.
             Darueber kommen alle ANTWORTEN.
    Server : wir betreiben denselben Dienst selbst. Darueber gehen alle
             KOMMANDOS hinaus -- als Notification, nicht als Write.

  Dieser Treiber bildet die MyDolphin-Plus-App nach, und zwar absichtlich bis
  in die Wartezeiten. Die Sequenz ist aus einem HCI-Mitschnitt einer echten
  App-Sitzung und aus dem dekompilierten App-Code belegt:

     1. Scannen, bis das PWS wirklich gehoert wurde -- nie blind verbinden.
     2. Eigenen GATT-Server aufbauen: fd5abba0 / fd5abba1, NOTIFY-only.
     3. +500 ms  -> verbinden.
     4. +500 ms  -> Dienste ermitteln.
     5. CCCD-Abo auf fd5abba1 des PWS; die Antwort darf ueber 300 ms brauchen.
     6. +1000 ms -> MTU 517 anfordern (das PWS antwortet mit 512).
     7. +10 ms   -> erster Rahmen (timezone), dann die restliche Init-Folge.
     8. Danach NUR EMPFANG. Das PWS schickt system_status von sich aus, im
        Leerlauf alle 20 bis 35 Sekunden. Wer pollt, macht es falsch.

  Warum nicht ueber MI32/Berry (der Vorgaenger dieses Treibers):
    - MI32 kann waehrend einer Verbindung nicht scannen; Schritt 1 ist dort
      unmoeglich, und ein blinder Verbindungsversuch scheitert regelmaessig.
    - Die lokale BLE-Adresse laesst sich nicht setzen.
    - MTU und Verbindungsparameter sind nicht erreichbar.
    - In MI32 startet erst BLE_OP_SET_ADV die GATT-Services -- man muss also
      werben, obwohl die App das nachweislich nie tut.
    - Berry kostet allein rund 75 kB Heap.

  Rahmenformat:  AB | 03 | ziel(2) | opcode | laenge(2) | nutzlast | pruef(2)
  Pruefsumme  :  16-Bit-Summe aller vorhergehenden Bytes, big-endian.
  Auf dem Draht steht der Rahmen in einer ASCII-Huelle, und der Praefix vor
  dem Doppelpunkt ist der KOMMANDONAME -- nicht "03". Ein falscher Praefix
  wird stillschweigend verworfen; das hat einmal einen ganzen Tag gekostet.
*/

#ifdef ESP32
#ifdef USE_DOLPHIN

#define XDRV_95                95

#include <NimBLEDevice.h>

/*********************************************************************************************\
 * Protokoll
\*********************************************************************************************/

#define DLP_SVC_UUID           "fd5abba0-3935-11e5-85a6-0002a5d5c51b"
#define DLP_CHR_UUID           "fd5abba1-3935-11e5-85a6-0002a5d5c51b"

#define DLP_SOP                0xAB     // Rahmenanfang
#define DLP_SOURCE             0x03     // Quell-Byte, das wir senden
#define DLP_HEADER_LEN         7        // SOP, Quelle, Ziel(2), Opcode, Laenge(2)
#define DLP_CRC_LEN            2

// Ziele. FFF8 und FFFA beantwortet das Netzteil selbst; FFFD und FFF9 gehen
// ueber die Bruecke zum Roboter und schweigen, wenn der nicht angeschlossen
// ist. Das ist beim Suchen von Fehlern der wichtigste Unterschied.
#define DLP_DEST_SYSTEM        0xFFF8   // Start, Stopp, Status  (PWS-lokal)
#define DLP_DEST_ROBOT         0xFFF9   // Uhrzeit, Wochenplan   (Roboter)
#define DLP_DEST_STORAGE       0xFFFD   // sm/mu-Daten           (Roboter)
#define DLP_DEST_FEATURES      0xFFFA   // pws_features          (PWS-lokal)
#define DLP_DEST_CLOUD         0xFFFE   // Zeitzone, Cloud       (PWS-lokal)

#define DLP_OP_STATUS          0x07
#define DLP_OP_START           0x06
#define DLP_OP_STOP            0x05
#define DLP_OP_SM_DATA         0x02     // Speicherblock des Netzteils
#define DLP_OP_MU_DATA         0x01     // Speicherblock der Motoreinheit
#define DLP_OP_WEEKLY          0x45     // Wochenplan schreiben (FFF9)

// Der Wochenplan in der Antwort auf get_sm_data. Die Offsets gelten fuer die
// Nutzlast OHNE das fuehrende ACK-Byte -- wer den Rohblock zaehlt, liegt um
// genau eins daneben.
#define DLP_PLAN_REPEAT        72       // 0 = jede Woche wiederholen AN (!)
#define DLP_PLAN_DAYS          73       // sieben Bloecke a 5 Byte
#define DLP_PLAN_FREQUENCY     213      // "triggered_by": 0 und 4 = keine Vorgabe

// Zeiten der App, aus dem Mitschnitt gemessen. Nicht "grosszuegig gerundet" --
// sie stehen hier, damit sich jemand spaeter fragt, warum ausgerechnet diese
// Werte, und die Antwort im Mitschnitt findet.
#define DLP_WAIT_AFTER_SERVER  500      // ms bis zum Verbinden
#define DLP_WAIT_AFTER_CONNECT 500      // ms bis discoverAttributes
#define DLP_WAIT_AFTER_CCCD    1000     // ms bis zur MTU-Anforderung
#define DLP_WAIT_AFTER_MTU     10       // ms bis zum ersten Rahmen
#define DLP_REPLY_TIMEOUT      2000     // ms je Auftrag, danach verworfen
#define DLP_SCAN_TIME          20       // s Suchdauer je Runde
#define DLP_SILENCE_MAX        180      // s ohne Antwort -> Verbindung neu
#define DLP_ASK_AFTER          90       // s ohne Push -> einmal nachfragen

// Abstand zwischen zwei Verbindungsversuchen, wachsend. Die App verbindet
// EINMAL und haelt; ein Treiber, der alle zehn Sekunden neu verbindet, zeigt
// der Firmware etwas, das sie nie zu sehen bekommt. Genau das steht im
// Verdacht, das PWS in den Zustand "verbunden und trotzdem stumm" zu bringen.
const uint16_t kDolphinBackoff[] = { 10, 15, 30, 60, 120, 300, 600, 1800 };

/*********************************************************************************************\
 * Zustand
\*********************************************************************************************/

enum DolphinPhase {
  DLP_PHASE_OFF,           // stillgelegt (DolphinQuiet 1)
  DLP_PHASE_IDLE,          // wartet auf den naechsten Versuch
  DLP_PHASE_SCAN,          // sucht das PWS im Werbepaket
  DLP_PHASE_CONNECT,       // verbindet
  DLP_PHASE_DISCOVER,      // ermittelt Dienste
  DLP_PHASE_SUBSCRIBE,     // schreibt das CCCD
  DLP_PHASE_MTU,           // handelt die MTU aus
  DLP_PHASE_INIT,          // schickt die Init-Folge
  DLP_PHASE_READY          // laeuft, empfaengt Pushes
};

const char kDolphinPhase[] PROGMEM =
  "Stillgelegt|Wartet|Sucht|Verbindet|Dienste|Abo|MTU|Init|Bereit";

struct DolphinQueueItem {
  uint16_t dest;
  uint8_t  opcode;
  // Die fertige Huelle, "name:hex". 128 Byte, nicht 80: Der Wochenplan ist
  // mit Abstand der laengste Rahmen -- "set_weekly_timer:" plus 46 Byte als
  // Hex sind 109 Zeichen. Alles andere liegt bei hoechstens 53.
  //
  // Bis zum 23. August 2026 stand hier 80. Der Rahmen wurde beim Einreihen
  // still gekuerzt, das Netzteil bekam eine falsche Laenge mit falscher
  // Pruefsumme und verwarf ihn kommentarlos -- kein ACK, keine Aenderung.
  // Auffindbar war das nur, indem man die Laengen ALLER Kommandos vergleicht:
  // Es funktionierte alles ausser dem einen, das laenger als der Puffer war.
  char     wire[128];
};

#define DLP_QUEUE_MAX          8
// Der Speicherblock des Netzteils ist rund 256 Byte lang, macht mit Kopf und
// Pruefsumme 265 Byte -- als Hex-Text 530 Zeichen, und er kommt fragmentiert.
// Wer hier zu knapp rechnet, verliert genau die eine Antwort, die den
// Wochenplan traegt.
#define DLP_RX_MAX             800
#define DLP_FRAME_MAX          320

struct DOLPHIN {
  NimBLEClient*             client = nullptr;
  NimBLEServer*             server = nullptr;
  NimBLECharacteristic*     chr = nullptr;
  NimBLERemoteCharacteristic* remote = nullptr;

  DolphinPhase phase = DLP_PHASE_IDLE;
  uint32_t     timer = 0;              // millis, wann die Phase weitergeht
  uint32_t     last_reply = 0;         // millis der letzten Antwort
  uint32_t     subscribed_at = 0;      // millis des Abos
  uint32_t     last_send = 0;

  uint16_t     backoff_index = 0;
  uint16_t     wait_seconds = 0;
  uint16_t     attempts = 0;
  uint16_t     mtu = 0;

  bool         found = false;          // im Scan gesehen
  bool         peer_subscribed = false;// das PWS hat unsere Char abonniert
  bool         debug = false;
  bool         quiet = false;
  bool         publish = true;         // bei Zustandsaenderung sofort senden
  bool         publish_pending = false;// im BLE-Task gesetzt, im Loop gesendet

  uint32_t     sent = 0;
  uint32_t     received = 0;
  uint32_t     seen = 0;                // Werbepakete insgesamt, je Suchlauf
  char         seen_list[220] = { 0 };  // die ersten Adressen eines Suchlaufs
  uint32_t     dropped = 0;

  DolphinQueueItem queue[DLP_QUEUE_MAX];
  uint8_t      queue_len = 0;
  bool         inflight = false;
  uint16_t     inflight_dest = 0;
  uint8_t      inflight_op = 0;
  uint32_t     inflight_since = 0;

  char         rx[DLP_RX_MAX];
  uint16_t     rx_len = 0;

  // Wochenplan, so wie ihn das Netzteil fuehrt
  bool         plan_read = false;      // erst lesen, dann schreiben duerfen
  bool         plan_repeat = false;
  uint8_t      plan_frequency = 0;
  uint8_t      plan_on[8] = { 0 };     // Index 1..7 = Sonntag..Samstag
  uint8_t      plan_hour[8] = { 0 };
  uint8_t      plan_minute[8] = { 0 };
  uint8_t      plan_mode[8] = { 0 };

  // Zuletzt gemeldeter Zustand
  bool         have_status = false;
  uint8_t      last_published_sm = 0xFE;   // fuer die Aenderungserkennung
  uint8_t      last_published_mu = 0xFE;
  uint8_t      last_published_mode = 0xFE;
  uint8_t      mu_state = 0xFF;
  uint8_t      sm_state = 0xFF;
  uint8_t      filter = 0xFF;
  uint8_t      mode = 0;
  uint16_t     cycle[11] = { 0 };      // Dauer je Reinigungsmodus, in Minuten
} Dolphin;

// Aus der Registry/Konfiguration -- vorerst fest, spaeter ueber ein Kommando.
static const char kDolphinMac[] = "30:09:f9:7b:90:e6";

/*********************************************************************************************\
 * Rahmen bauen
\*********************************************************************************************/

// Baut den kompletten Rahmen als Hex-Text und stellt den Kommandonamen voran.
// Das Netzteil verwirft jeden Rahmen mit falschem Praefix stillschweigend --
// deshalb steht der Name hier neben jedem Vorgang und nicht irgendwo zentral.
void DolphinBuildFrame(char* out, size_t out_len, const char* name,
                       uint16_t dest, uint8_t opcode,
                       const uint8_t* payload, uint8_t payload_len) {
  // 64 Byte Nutzlast reichen mit Reserve: der laengste Vorgang ist der
  // Wochenplan mit 37.
  uint8_t frame[DLP_HEADER_LEN + 64 + DLP_CRC_LEN];
  uint8_t n = 0;

  frame[n++] = DLP_SOP;
  frame[n++] = DLP_SOURCE;
  frame[n++] = dest >> 8;
  frame[n++] = dest & 0xFF;
  frame[n++] = opcode;
  frame[n++] = 0;                       // Laenge, big-endian
  frame[n++] = payload_len;
  for (uint8_t i = 0; i < payload_len; i++) { frame[n++] = payload[i]; }

  uint16_t sum = 0;
  for (uint8_t i = 0; i < n; i++) { sum += frame[i]; }
  frame[n++] = sum >> 8;
  frame[n++] = sum & 0xFF;

  size_t p = snprintf_P(out, out_len, PSTR("%s:"), name);
  for (uint8_t i = 0; i < n && p + 2 < out_len; i++) {
    p += snprintf_P(out + p, out_len - p, PSTR("%02x"), frame[i]);
  }
}

bool DolphinEnqueue(const char* name, uint16_t dest, uint8_t opcode,
                    const uint8_t* payload, uint8_t payload_len) {
  if (Dolphin.queue_len >= DLP_QUEUE_MAX) {
    Dolphin.dropped++;
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: Warteschlange voll, %s verworfen"), name);
    return false;
  }
  // Passt der Rahmen ueberhaupt? Ein zu knapper Puffer kuerzt sonst
  // stillschweigend, und das Ergebnis sieht aus wie ein stummes Netzteil.
  size_t brauchbar = strlen(name) + 1 + ((DLP_HEADER_LEN + payload_len + DLP_CRC_LEN) * 2);
  if (brauchbar >= sizeof(Dolphin.queue[0].wire)) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: %s braucht %d Zeichen, Puffer hat %d -- nicht gesendet"),
           name, brauchbar, sizeof(Dolphin.queue[0].wire));
    return false;
  }

  DolphinQueueItem* item = &Dolphin.queue[Dolphin.queue_len++];
  item->dest = dest;
  item->opcode = opcode;
  DolphinBuildFrame(item->wire, sizeof(item->wire), name, dest, opcode, payload, payload_len);
  return true;
}

/*********************************************************************************************\
 * Senden
\*********************************************************************************************/

void DolphinSendWire(const char* wire) {
  if (!Dolphin.chr || !Dolphin.peer_subscribed) { return; }
  Dolphin.chr->setValue((uint8_t*)wire, strlen(wire));
  Dolphin.chr->notify();
  Dolphin.sent++;
  Dolphin.last_send = millis();
  AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: sende %s"), wire);
}

// Serielle Warteschlange nach dem Vorbild der App: immer nur EIN Auftrag
// unterwegs, der naechste erst nach der Antwort oder nach der Frist.
// Ausdruecklich OHNE erneutes Senden desselben Rahmens -- die App tut das
// nicht, und jeder Wiederholung ist zusaetzliche Last auf einer Firmware,
// die daran zu ersticken scheint.
void DolphinQueueWork(void) {
  if (!Dolphin.peer_subscribed) { return; }

  if (Dolphin.inflight) {
    if (TimePassedSince(Dolphin.inflight_since) < DLP_REPLY_TIMEOUT) { return; }
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: keine Antwort auf %04X/%02X, verworfen"),
           Dolphin.inflight_dest, Dolphin.inflight_op);
    Dolphin.inflight = false;
  }

  if (0 == Dolphin.queue_len) { return; }

  DolphinQueueItem item = Dolphin.queue[0];
  for (uint8_t i = 1; i < Dolphin.queue_len; i++) { Dolphin.queue[i - 1] = Dolphin.queue[i]; }
  Dolphin.queue_len--;

  Dolphin.inflight = true;
  Dolphin.inflight_dest = item.dest;
  Dolphin.inflight_op = item.opcode;
  Dolphin.inflight_since = millis();
  DolphinSendWire(item.wire);
}

/*********************************************************************************************\
 * Init-Folge -- Reihenfolge und Namen exakt wie die App
\*********************************************************************************************/

void DolphinInitSequence(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("DLP: Initialisierung wie die App"));

  // Zeitzone: Abweichung von UTC in Minuten, big-endian.
  int32_t offset = (int32_t)(Rtc.local_time - Rtc.utc_time) / 60;
  uint8_t tz[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xFF) };
  DolphinEnqueue("timezone", DLP_DEST_CLOUD, 0xc2, tz, 2);

  uint32_t utc = Rtc.utc_time;
  uint8_t clock[4] = { (uint8_t)(utc >> 24), (uint8_t)(utc >> 16),
                       (uint8_t)(utc >> 8), (uint8_t)(utc & 0xFF) };
  DolphinEnqueue("real_time_clock", DLP_DEST_ROBOT, 0x09, clock, 4);

  uint8_t sm[3] = { 0x02, 0x00, 0xff };
  DolphinEnqueue("get_sm_data", DLP_DEST_STORAGE, 0x02, sm, 3);

  uint8_t mu[3] = { 0x01, 0x00, 0xff };
  DolphinEnqueue("get_mu_data", DLP_DEST_STORAGE, 0x01, mu, 3);

  DolphinEnqueue("system_status", DLP_DEST_SYSTEM, DLP_OP_STATUS, nullptr, 0);
  DolphinEnqueue("pws_features", DLP_DEST_FEATURES, 0x1a, nullptr, 0);
  DolphinEnqueue("cloud_connection_status", DLP_DEST_CLOUD, 0xdf, nullptr, 0);
}

/*********************************************************************************************\
 * Vorgaenge
\*********************************************************************************************/

// Laeuft gerade ein Durchgang? Das Netzteil meldet 1 (ein) oder 5 (reinigt),
// die Motoreinheit 0 bis 3 (Initialisierung, Vermessung, Reinigung,
// Rückholung). In all diesen Zustaenden waere ein Start sinnlos.
bool DolphinRunning(void) {
  if (!Dolphin.have_status) { return false; }
  if ((1 == Dolphin.sm_state) || (5 == Dolphin.sm_state)) { return true; }
  return (Dolphin.mu_state <= 3);
}

// Start und Stopp beantwortet das Netzteil NICHT, sie wirken aber trotzdem.
// Den Status danach nicht abfragen: Er kommt etwa eine Sekunde spaeter von
// selbst.
bool DolphinStart(void) {
  if (DolphinRunning()) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Start abgelehnt, laeuft bereits"));
    return false;
  }
  return DolphinEnqueue("start_up_dolphin", DLP_DEST_SYSTEM, DLP_OP_START, nullptr, 0);
}

bool DolphinStop(void) {
  if (!DolphinRunning()) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Stopp abgelehnt, laeuft nicht"));
    return false;
  }
  return DolphinEnqueue("shutdown_dolphin", DLP_DEST_SYSTEM, DLP_OP_STOP, nullptr, 0);
}

void DolphinTelemetry(void) {
  uint8_t sm[3] = { 0x02, 0x00, 0xff };
  uint8_t mu[3] = { 0x01, 0x00, 0xff };
  DolphinEnqueue("get_sm_data", DLP_DEST_STORAGE, DLP_OP_SM_DATA, sm, 3);
  DolphinEnqueue("get_mu_data", DLP_DEST_STORAGE, DLP_OP_MU_DATA, mu, 3);
  DolphinEnqueue("system_status", DLP_DEST_SYSTEM, DLP_OP_STATUS, nullptr, 0);
}

// Der Wochenplan geht nur als GANZES zum Netzteil: Wiederholung und alle
// sieben Tage in einem Rahmen. Wer schreibt, ohne gelesen zu haben, schickt
// fuer jedes Feld, das er nicht angefasst hat, einen geratenen Wert mit --
// und genau daran ist am Geraet schon einmal die Wiederholung ausgegangen,
// ohne dass sie jemand angefasst haette.
bool DolphinWritePlan(void) {
  if (!Dolphin.plan_read) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: noch kein Wochenplan gelesen -- nicht gesetzt"));
    return false;
  }

  uint8_t payload[37];
  payload[0] = Dolphin.plan_repeat ? 0 : 1;      // invertiert, siehe oben
  payload[1] = 0;                                // triggered_by, wie die App
  // Die Bloecke gehen in der Reihenfolge MONTAG bis SAMSTAG, dann SONNTAG
  // hinaus -- nicht nach der Nummerierung des Geraets (1 = Sonntag).
  //
  // Belegt am HCI-Mitschnitt: die App sendet
  //   …450025 01 00 | 02 01 0d 00 01 | 03 … | 07 … | 01 01 0d 00 01
  // also 02,03,04,05,06,07,01. Am 23. August 2026 stand hier zuerst
  // 01..07, und das Netzteil hat den Rahmen kommentarlos verworfen: Er ging
  // hinaus, blieb aber unbeantwortet, und das anschliessende Zuruecklesen
  // lieferte unveraendert den alten Plan.
  static const uint8_t kOrder[7] = { 2, 3, 4, 5, 6, 7, 1 };
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t day = kOrder[i];
    uint8_t* b = &payload[2 + i * 5];
    b[0] = day;
    b[1] = Dolphin.plan_on[day] ? 1 : 0;
    // Die Zeit geht auch fuer einen abgeschalteten Tag mit. Sie mit ff:ff zu
    // fuellen wirft sie im Geraet weg -- daran wurde aus "Montag 13:00,
    // abgeschaltet" nach dem Neuladen "Montag 09:00".
    b[2] = Dolphin.plan_hour[day];
    b[3] = Dolphin.plan_minute[day];
    b[4] = Dolphin.plan_mode[day] ? Dolphin.plan_mode[day] : 1;
  }

  DolphinEnqueue("set_weekly_timer", DLP_DEST_ROBOT, DLP_OP_WEEKLY, payload, sizeof(payload));
  // Gesetzten Stand zuruecklesen, statt ihn zu glauben.
  uint8_t sm[3] = { 0x02, 0x00, 0xff };
  DolphinEnqueue("get_sm_data", DLP_DEST_STORAGE, DLP_OP_SM_DATA, sm, 3);
  return true;
}

// Bei Zustandsaenderung von selbst veroeffentlichen. Abschaltbar mit
// DolphinPublish 0 -- dann bleibt es beim normalen Telemetrie-Takt.
// ACHTUNG: Diese Funktion laeuft im NimBLE-Task, nicht in Tasmotas
// Hauptschleife. Von hier aus darf NICHT veroeffentlicht werden --
// MqttPublishSensor() schreibt in denselben Antwortpuffer, den gerade ein
// laufendes Kommando benutzt. Am 23. August 2026 sichtbar geworden: Ein
// DolphinSend antwortete mit "{}" statt "Done", weil die Statusantwort des
// Netzteils dazwischenfunkte. Deshalb hier nur eine Marke setzen; gesendet
// wird im Sekundentakt aus dem Loop.
void DolphinPublishOnChange(void) {
  if (!Dolphin.publish) { return; }
  if ((Dolphin.sm_state == Dolphin.last_published_sm) &&
      (Dolphin.mu_state == Dolphin.last_published_mu) &&
      (Dolphin.mode == Dolphin.last_published_mode)) { return; }
  Dolphin.last_published_sm = Dolphin.sm_state;
  Dolphin.last_published_mu = Dolphin.mu_state;
  Dolphin.last_published_mode = Dolphin.mode;
  Dolphin.publish_pending = true;
}

/*********************************************************************************************\
 * Empfangen
\*********************************************************************************************/

uint8_t DolphinHexNibble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return 0xFF;
}

void DolphinHandleFrame(const uint8_t* frame, uint16_t len) {
  if (len < DLP_HEADER_LEN + DLP_CRC_LEN) { return; }

  uint16_t sum = 0;
  for (uint16_t i = 0; i < len - DLP_CRC_LEN; i++) { sum += frame[i]; }
  uint16_t crc = (frame[len - 2] << 8) | frame[len - 1];
  if (sum != crc) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: Pruefsumme falsch, Rahmen verworfen"));
    return;
  }

  uint16_t dest = (frame[2] << 8) | frame[3];
  uint8_t  opcode = frame[4];
  uint16_t plen = (frame[5] << 8) | frame[6];
  const uint8_t* payload = &frame[DLP_HEADER_LEN];

  Dolphin.received++;
  Dolphin.last_reply = millis();

  AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: empfangen src=%02X ziel=%04X op=%02X len=%d"),
         frame[1], dest, opcode, plen);

  // Zuordnung ueber Ziel und Opcode, wie die App. Eine fremde Antwort
  // beendet den laufenden Auftrag nicht.
  if (Dolphin.inflight && Dolphin.inflight_dest == dest && Dolphin.inflight_op == opcode) {
    Dolphin.inflight = false;
  }

  // Der Wochenplan steckt in der Antwort auf get_sm_data. Von dort kommt er --
  // damit die Anzeige auch nach einem Neustart stimmt und auch dann, wenn der
  // Plan ueber die App gesetzt wurde.
  if (DLP_DEST_STORAGE == dest && DLP_OP_SM_DATA == opcode && plen > DLP_PLAN_DAYS + 35) {
    const uint8_t* d = payload + 1;               // ohne ACK-Byte
    // 0 heisst AN. Das ist kein Tippfehler: belegt an der Feldkarte der App
    // (ble_iot_protocol.json), am Parser (bArrI[72] == 0) und an zwei echten
    // Umschaltungen im HCI-Mitschnitt.
    Dolphin.plan_repeat = (0 == d[DLP_PLAN_REPEAT]);
    Dolphin.plan_frequency = (plen > DLP_PLAN_FREQUENCY + 1) ? d[DLP_PLAN_FREQUENCY] : 0;
    for (uint8_t i = 0; i < 7; i++) {
      const uint8_t* b = &d[DLP_PLAN_DAYS + i * 5];
      uint8_t day = b[0];
      if ((day < 1) || (day > 7)) { continue; }
      Dolphin.plan_on[day] = b[1];
      // ff:ff heisst "abgeschaltet, keine Zeit hinterlegt". Dann die zuletzt
      // bekannte Zeit behalten, statt eine zu erfinden -- sonst behauptet die
      // Anzeige einen Wert, den das Geraet nie gemeldet hat.
      if ((0xFF != b[2]) || (0xFF != b[3])) {
        Dolphin.plan_hour[day] = b[2];
        Dolphin.plan_minute[day] = b[3];
      }
      Dolphin.plan_mode[day] = b[4];
    }
    bool first = !Dolphin.plan_read;
    Dolphin.plan_read = true;
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Wochenplan gelesen, Wiederholen=%d Frequenz=%d"),
           Dolphin.plan_repeat, Dolphin.plan_frequency);
    // Den Plan einmal aktiv hinausgeben, sobald er da ist -- und nach jedem
    // Setzen, denn dann wird er zurueckgelesen.
    if (Dolphin.publish && (first || Dolphin.have_status)) { Dolphin.publish_pending = true; }
  }

  if (DLP_DEST_SYSTEM == dest && DLP_OP_STATUS == opcode && plen > 4) {
    // Das erste Byte der Nutzlast ist ein ACK; die Feldpositionen gelten
    // danach: 0 Roboterzustand, 1 Netzteilzustand, 2 Filter, 3 Modus.
    const uint8_t* d = payload + 1;
    Dolphin.mu_state = d[0];
    Dolphin.sm_state = d[1];
    Dolphin.filter   = d[2];
    Dolphin.mode     = d[3];
    Dolphin.have_status = true;
    // Die Zykluszeiten stehen ab Byte 30, elf Stueck, je 16 Bit big-endian:
    // 0096 = 150 Minuten = 2,5 Std fuer "alle Flächen". Genau diese Dauer
    // zeigt die App neben jedem Wochentag an.
    if (plen > (30 + 22)) {
      for (uint8_t i = 0; i < 11; i++) {
        Dolphin.cycle[i] = (d[30 + (i * 2)] << 8) | d[31 + (i * 2)];
      }
    }
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Status sm=%d mu=%d Modus=%d Filter=%d"),
           Dolphin.sm_state, Dolphin.mu_state, Dolphin.mode, Dolphin.filter);
    // Sofort senden, sobald sich wirklich etwas geaendert hat -- damit am
    // Broker niemand auf die TelePeriod warten muss. Das Netzteil schickt
    // seinen Status ohnehin nach jedem zustandsaendernden Kommando etwa eine
    // Sekunde spaeter; genau diese Nachricht soll durchgereicht werden.
    // Ohne die Aenderungspruefung waere es alle 20 bis 35 Sekunden eine
    // Nachricht, auch wenn sich nichts tut.
    DolphinPublishOnChange();
  }
}

// Eingehender Text. Alles vor dem ersten ':' wird verworfen; Folgestuecke
// langer Antworten kommen OHNE Doppelpunkt und werden angehaengt.
void DolphinReceiveText(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (':' == c) { Dolphin.rx_len = 0; continue; }
    if (Dolphin.rx_len < DLP_RX_MAX - 1) { Dolphin.rx[Dolphin.rx_len++] = c; }
  }

  // So viele vollstaendige Rahmen herausloesen, wie im Puffer stecken.
  while (Dolphin.rx_len >= (DLP_HEADER_LEN * 2)) {
    uint8_t head[DLP_HEADER_LEN];
    bool bad = false;
    for (uint8_t i = 0; i < DLP_HEADER_LEN; i++) {
      uint8_t hi = DolphinHexNibble(Dolphin.rx[i * 2]);
      uint8_t lo = DolphinHexNibble(Dolphin.rx[i * 2 + 1]);
      if (0xFF == hi || 0xFF == lo) { bad = true; break; }
      head[i] = (hi << 4) | lo;
    }
    if (bad || DLP_SOP != head[0]) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: unbrauchbare Daten verworfen"));
      Dolphin.rx_len = 0;
      return;
    }

    uint16_t plen = (head[5] << 8) | head[6];
    uint16_t total = (DLP_HEADER_LEN + plen + DLP_CRC_LEN) * 2;
    if (Dolphin.rx_len < total) { return; }        // Fragment, auf den Rest warten

    uint8_t frame[DLP_FRAME_MAX];
    uint16_t bytes = total / 2;
    if (bytes > sizeof(frame)) {
      // Nicht stillschweigend verwerfen: Ein zu grosser Rahmen sieht sonst
      // aus wie "das Netzteil hat nicht geantwortet" und schickt jeden, der
      // den Fehler sucht, an die falsche Stelle.
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: Rahmen zu gross (%d Byte, Puffer %d) -- verworfen"),
             bytes, sizeof(frame));
      Dolphin.rx_len = 0;
      return;
    }
    for (uint16_t i = 0; i < bytes; i++) {
      frame[i] = (DolphinHexNibble(Dolphin.rx[i * 2]) << 4) | DolphinHexNibble(Dolphin.rx[i * 2 + 1]);
    }
    DolphinHandleFrame(frame, bytes);

    memmove(Dolphin.rx, Dolphin.rx + total, Dolphin.rx_len - total);
    Dolphin.rx_len -= total;
  }
}

/*********************************************************************************************\
 * NimBLE-Rueckrufe
\*********************************************************************************************/

class DolphinServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: Gegenstelle am eigenen Server"));
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: Gegenstelle vom eigenen Server getrennt (%d)"), reason);
    Dolphin.peer_subscribed = false;
  }
  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
    Dolphin.mtu = MTU;
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: MTU %d"), MTU);
  }
};

class DolphinCharCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pChr, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    // subValue: 0 = abbestellt, 1 = Notifications, 2 = Indications
    Dolphin.peer_subscribed = (subValue > 0);
    Dolphin.subscribed_at = millis();
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Netzteil abonniert unsere Characteristic (%d)"), subValue);
  }
};

class DolphinClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: verbunden"));
  }
  void onConnectFail(NimBLEClient* pClient, int reason) override {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: Verbindung fehlgeschlagen (%d)"), reason);
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: getrennt (%d)"), reason);
    Dolphin.peer_subscribed = false;
    Dolphin.remote = nullptr;
    if (DLP_PHASE_OFF != Dolphin.phase) { Dolphin.phase = DLP_PHASE_IDLE; }
  }
  void onMTUChange(NimBLEClient* pClient, uint16_t MTU) override {
    Dolphin.mtu = MTU;
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: MTU der Verbindung %d"), MTU);
  }
};

class DolphinScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    // Mitzaehlen, was ueberhaupt hereinkommt. Ohne das sehen zwei ganz
    // verschiedene Lagen im Log gleich aus, naemlich still: "der Scan laeuft,
    // das Netzteil sendet nur nicht" und "der Scan laeuft gar nicht". Am
    // 23. August 2026 hat genau diese Verwechslung eine Stunde gekostet.
    Dolphin.seen++;
    // Die Adressen sammeln statt einzeln loggen: Auf Log-Stufe 4 ueberschreibt
    // Tasmotas Ringpuffer solche Zeilen, bevor man sie abholen kann. Am Ende
    // des Suchlaufs steht die Liste in EINER Zeile.
    size_t used = strlen(Dolphin.seen_list);
    if (used < sizeof(Dolphin.seen_list) - 26) {
      snprintf_P(Dolphin.seen_list + used, sizeof(Dolphin.seen_list) - used,
                 PSTR("%s%s/%d"), used ? PSTR(" ") : PSTR(""),
                 dev->getAddress().toString().c_str(), dev->getRSSI());
    }
    // Adressen als Typ vergleichen, nicht als Text: Gross-/Kleinschreibung
    // und Adresstyp haengen sonst an der Formatierung der Bibliothek.
    if (dev->getAddress() == NimBLEAddress(kDolphinMac, BLE_ADDR_PUBLIC)) {
      Dolphin.found = true;
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: Netzteil gehoert, %d dBm"), dev->getRSSI());
      NimBLEDevice::getScan()->stop();
    }
  }
};

static DolphinServerCallbacks  DolphinServerCb;
static DolphinCharCallbacks    DolphinCharCb;
static DolphinClientCallbacks  DolphinClientCb;
static DolphinScanCallbacks    DolphinScanCb;

// Der Empfang laeuft ueber die Client-Verbindung: Antworten kommen als
// Notification auf fd5abba1 des PWS.
//
// Als statische Methode und NICHT als freie Funktion: Der Arduino-
// Praeprozessor sammelt Prototypen aus allen .ino-Dateien und schreibt sie an
// den Anfang von tasmota.ino -- auch aus Dateien, deren Inhalt per #ifdef
// abgeschaltet ist. Eine freie Funktion mit NimBLE-Typen in der Signatur
// bricht dort jeden Build, auch den ohne USE_DOLPHIN. Methoden werden nicht
// extrahiert.
struct DolphinNotify {
  static void handle(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
    DolphinReceiveText(data, len);
  }
};

/*********************************************************************************************\
 * Aufbau -- die Sequenz der App, Schritt fuer Schritt
\*********************************************************************************************/

void DolphinBuildServer(void) {
  if (Dolphin.server) { return; }

  Dolphin.server = NimBLEDevice::createServer();
  Dolphin.server->setCallbacks(&DolphinServerCb);

  NimBLEService* svc = Dolphin.server->createService(DLP_SVC_UUID);
  // NOTIFY-only, genau wie der Server der App (Properties 0x10). Die App
  // legt das CCCD sogar nur lesbar an und lehnt den Schreibversuch des PWS
  // ab -- es funktioniert trotzdem. Wir sind hier freundlicher.
  Dolphin.chr = svc->createCharacteristic(DLP_CHR_UUID, NIMBLE_PROPERTY::NOTIFY);
  Dolphin.chr->setCallbacks(&DolphinCharCb);
  svc->start();

  // Werben, obwohl die App es nachweislich NIE tut (kein startAdvertising in
  // ihrem ganzen Quelltext, im HCI-Mitschnitt nur Google Fast Pair).
  //
  // Der Grund ist NimBLE, nicht das Protokoll: Server und Client sind hier
  // getrennte Objekte. Baut der Client die Verbindung auf, bekommt der Server
  // von ihr nichts mit -- er erfaehrt also auch nicht, dass das Netzteil
  // unsere Characteristic abonniert hat, und jede Notification verfaellt
  // still. Genau dagegen gibt es den NimBLE-Patch dieses Zweigs; er reicht
  // BLE_GAP_EVENT_SUBSCRIBE weiter, wenn der Server die Verbindung kennt.
  // Ohne Advertising kennt er sie nicht.
  //
  // Am 23. August 2026 beide Wege am Geraet gemessen: mit Werbung kommt das
  // Abo (Subscribed true, Status fliesst), ohne Werbung bleibt es aus --
  // Verbindung und MTU stehen dabei einwandfrei. Wer das aendern will, muss
  // in NimBLE dafuer sorgen, dass der Server die fremde Verbindung adoptiert.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(DLP_SVC_UUID);
  adv->enableScanResponse(false);
  adv->start();

  AddLog(LOG_LEVEL_INFO, PSTR("DLP: eigener GATT-Server steht und wirbt"));
}

void DolphinStartScan(void) {
  Dolphin.found = false;
  Dolphin.seen = 0;
  Dolphin.seen_list[0] = 0;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&DolphinScanCb, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  scan->start(DLP_SCAN_TIME * 1000, false, true);
  Dolphin.phase = DLP_PHASE_SCAN;
  Dolphin.timer = millis() + (DLP_SCAN_TIME * 1000);
  AddLog(LOG_LEVEL_INFO, PSTR("DLP: suche das Netzteil (%d s)"), DLP_SCAN_TIME);
}

void DolphinDisconnect(void) {
  if (Dolphin.client && Dolphin.client->isConnected()) {
    Dolphin.client->disconnect();
  }
  Dolphin.peer_subscribed = false;
  Dolphin.remote = nullptr;
  Dolphin.inflight = false;
  Dolphin.queue_len = 0;
  Dolphin.rx_len = 0;
}

uint16_t DolphinBackoff(void) {
  uint16_t i = Dolphin.backoff_index;
  if (i >= (sizeof(kDolphinBackoff) / sizeof(kDolphinBackoff[0]))) {
    i = (sizeof(kDolphinBackoff) / sizeof(kDolphinBackoff[0])) - 1;
  }
  return kDolphinBackoff[i];
}

void DolphinStateMachine(void) {
  if (DLP_PHASE_OFF == Dolphin.phase) { return; }
  // Ein Treffer im Scan beendet das Warten sofort -- sonst sitzt der Treiber
  // die vollen 20 Sekunden ab, obwohl er das Netzteil laengst gehoert hat.
  if ((DLP_PHASE_SCAN == Dolphin.phase) && Dolphin.found) { Dolphin.timer = 0; }
  if (Dolphin.timer && (millis() < Dolphin.timer)) { return; }

  switch (Dolphin.phase) {

    case DLP_PHASE_SCAN:
      // Der Timer oben laesst diese Phase erst nach der vollen Suchdauer
      // laufen; gefunden wird aber oft nach zwei Sekunden. Deshalb hier
      // gesondert.
      if (Dolphin.found) {
        NimBLEDevice::getScan()->stop();
        Dolphin.phase = DLP_PHASE_CONNECT;
        Dolphin.timer = millis() + DLP_WAIT_AFTER_SERVER;
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR("DLP: Netzteil nicht gehoert, %d andere Geraete: %s"),
               Dolphin.seen, Dolphin.seen_list);
        Dolphin.phase = DLP_PHASE_IDLE;
        Dolphin.timer = 0;
        if (Dolphin.backoff_index < 7) { Dolphin.backoff_index++; }
      }
      break;

    case DLP_PHASE_CONNECT: {
      if (!Dolphin.client) {
        Dolphin.client = NimBLEDevice::createClient();
        Dolphin.client->setClientCallbacks(&DolphinClientCb, false);
        // Verbindungsparameter der App: 45 ms, keine Latenz, 5 s Timeout.
        Dolphin.client->setConnectionParams(36, 36, 0, 500);
      }
      NimBLEAddress addr(kDolphinMac, BLE_ADDR_PUBLIC);
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: verbinde (Versuch %d)"), Dolphin.attempts + 1);
      Dolphin.attempts++;
      if (!Dolphin.client->connect(addr, true, false, true)) {
        AddLog(LOG_LEVEL_INFO, PSTR("DLP: Verbindung kam nicht zustande"));
        Dolphin.phase = DLP_PHASE_IDLE;
        Dolphin.timer = 0;
        if (Dolphin.backoff_index < 7) { Dolphin.backoff_index++; }
        break;
      }
      Dolphin.phase = DLP_PHASE_DISCOVER;
      Dolphin.timer = millis() + DLP_WAIT_AFTER_CONNECT;
      break;
    }

    case DLP_PHASE_DISCOVER: {
      NimBLERemoteService* svc = Dolphin.client->getService(DLP_SVC_UUID);
      if (!svc) {
        AddLog(LOG_LEVEL_INFO, PSTR("DLP: Dienst fd5abba0 nicht gefunden"));
        DolphinDisconnect();
        Dolphin.phase = DLP_PHASE_IDLE;
        Dolphin.timer = 0;
        break;
      }
      Dolphin.remote = svc->getCharacteristic(DLP_CHR_UUID);
      if (!Dolphin.remote) {
        AddLog(LOG_LEVEL_INFO, PSTR("DLP: Characteristic fd5abba1 nicht gefunden"));
        DolphinDisconnect();
        Dolphin.phase = DLP_PHASE_IDLE;
        Dolphin.timer = 0;
        break;
      }
      Dolphin.phase = DLP_PHASE_SUBSCRIBE;
      Dolphin.timer = 0;
      break;
    }

    case DLP_PHASE_SUBSCRIBE:
      if (!Dolphin.remote->subscribe(true, DolphinNotify::handle)) {
        AddLog(LOG_LEVEL_INFO, PSTR("DLP: Abo beim Netzteil abgelehnt"));
        DolphinDisconnect();
        Dolphin.phase = DLP_PHASE_IDLE;
        Dolphin.timer = 0;
        break;
      }
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: beim Netzteil abonniert"));
      Dolphin.phase = DLP_PHASE_MTU;
      Dolphin.timer = millis() + DLP_WAIT_AFTER_CCCD;
      break;

    case DLP_PHASE_MTU:
      Dolphin.client->exchangeMTU();
      Dolphin.phase = DLP_PHASE_INIT;
      Dolphin.timer = millis() + DLP_WAIT_AFTER_MTU;
      break;

    case DLP_PHASE_INIT:
      DolphinInitSequence();
      Dolphin.phase = DLP_PHASE_READY;
      Dolphin.timer = 0;
      Dolphin.attempts = 0;
      break;

    case DLP_PHASE_READY:
      DolphinQueueWork();
      break;

    default:
      break;
  }
}

/*********************************************************************************************\
 * Sekundentakt
\*********************************************************************************************/

void DolphinEverySecond(void) {
  // Aus dem BLE-Task angemeldete Veroeffentlichungen hier abarbeiten, wo wir
  // sicher in Tasmotas Hauptschleife sind.
  if (Dolphin.publish_pending) {
    Dolphin.publish_pending = false;
    MqttPublishSensor();
  }
  if (DLP_PHASE_OFF == Dolphin.phase) { return; }

  if (DLP_PHASE_IDLE == Dolphin.phase) {
    Dolphin.wait_seconds++;
    if (Dolphin.wait_seconds >= DolphinBackoff()) {
      Dolphin.wait_seconds = 0;
      DolphinDisconnect();
      DolphinStartScan();
    }
    return;
  }

  if (DLP_PHASE_READY != Dolphin.phase) { return; }

  // Die Fehlversuche nur vergessen, wenn ueber diese Verbindung wirklich
  // etwas hereinkam. Sonst entsteht der Kreislauf, der das PWS ueber Tage
  // gehaemmert hat: verbinden gelingt, es bleibt still, wir trennen, und
  // zehn Sekunden spaeter geht es von vorn los.
  if (Dolphin.last_reply) { Dolphin.backoff_index = 0; }

  uint32_t quiet_since = Dolphin.last_reply ? Dolphin.last_reply : Dolphin.subscribed_at;
  if (Dolphin.peer_subscribed && quiet_since &&
      (TimePassedSince(quiet_since) > (DLP_SILENCE_MAX * 1000))) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: %d s ohne Antwort, Verbindung wird neu aufgebaut"),
           DLP_SILENCE_MAX);
    if (Dolphin.backoff_index < 7) { Dolphin.backoff_index++; }
    DolphinDisconnect();
    Dolphin.phase = DLP_PHASE_IDLE;
    Dolphin.wait_seconds = 0;
    Dolphin.last_reply = 0;
    Dolphin.subscribed_at = 0;
    return;
  }

  // Nachfragen ist das Sicherheitsnetz, nicht der Takt: Das PWS schickt den
  // Status von selbst. Die App fragt ihn nach der Init kein einziges Mal
  // mehr aktiv ab.
  if (Dolphin.last_reply && (TimePassedSince(Dolphin.last_reply) > (DLP_ASK_AFTER * 1000)) &&
      !Dolphin.inflight && (0 == Dolphin.queue_len)) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLP: lange kein Push, frage nach"));
    DolphinEnqueue("system_status", DLP_DEST_SYSTEM, DLP_OP_STATUS, nullptr, 0);
  }
}

/*********************************************************************************************\
 * Kommandos
\*********************************************************************************************/

#define D_CMND_DOLPHIN "Dolphin"

const char kDolphinCommands[] PROGMEM = D_CMND_DOLPHIN "|"
  "Status|Connect|Quiet|Debug|Init|Start|Stop|Telemetry|Plan|Publish|Send";

void (* const DolphinCommand[])(void) PROGMEM = {
  &CmndDolphinStatus, &CmndDolphinConnect, &CmndDolphinQuiet,
  &CmndDolphinDebug, &CmndDolphinInit, &CmndDolphinStart, &CmndDolphinStop,
  &CmndDolphinTelemetry, &CmndDolphinPlan, &CmndDolphinPublish,
  &CmndDolphinSend };

const char kDolphinWeekday[] PROGMEM = "Sonntag|Montag|Dienstag|Mittwoch|Donnerstag|Freitag|Samstag";

// Zustaende, wie sie das Netzteil meldet. mu_state 7 heisst "keine Verbindung
// zum Roboter" -- am eigenen Geraet belegt: ohne angeschlossenen Roboter
// stand dort 7, mit angeschlossenem 2.
const char kDolphinSmState[] PROGMEM =
  "aus|ein|Wochenprogramm wartet|Startverzögerung|Programmierung|reinigt|Schlaf";
const char kDolphinMuState[] PROGMEM =
  "Initialisierung|Vermessung|Reinigung|Rückholung|fertig|Störung|Programmierung|keine Verbindung";
// Reinigungsmodi. Welche dieser Modi der Roboter wirklich annimmt, entscheidet
// die App anhand einer Faehigkeitsliste aus der Cloud -- ueber Bluetooth ist
// das nicht zu erfahren. Deshalb wird der Modus angezeigt, aber nicht gesetzt.
const char kDolphinMode[] PROGMEM =
  "|alle Flächen|kurz|Cove|nur Boden|Wasserlinie|Ultra|Punkt|nur Wände|TicTac|eigen|Pickup";

// Wann ist der naechste Durchgang faellig? Der Plan steht im Netzteil und
// laeuft auch dann, wenn der ESP aus ist -- hier wird nur gerechnet.
//
// Tasmotas RtcTime.day_of_week zaehlt 1 = Sonntag bis 7 = Samstag, genau wie
// das Netzteil. Das ist Glueck, kein Verdienst: Wer die Zaehlung anfasst,
// muss beide Seiten pruefen.
// Dauer des eingestellten Modus in Minuten, 0 wenn unbekannt.
uint16_t DolphinModeMinutes(void) {
  if ((Dolphin.mode < 1) || (Dolphin.mode > 11)) { return 0; }
  return Dolphin.cycle[Dolphin.mode - 1];
}

bool DolphinNextRun(char* out, size_t out_len, uint32_t* minutes_away) {
  if (!Dolphin.plan_read) { return false; }

  uint8_t  today = RtcTime.day_of_week;
  uint32_t now_min = (RtcTime.hour * 60) + RtcTime.minute;

  for (uint8_t offset = 0; offset < 8; offset++) {
    uint8_t day = ((today - 1 + offset) % 7) + 1;
    if (!Dolphin.plan_on[day]) { continue; }
    uint32_t start = (Dolphin.plan_hour[day] * 60) + Dolphin.plan_minute[day];
    // Heute zaehlt nur, was noch bevorsteht.
    if ((0 == offset) && (start <= now_min)) { continue; }
    uint32_t away = (offset * 1440) + start - now_min;
    char name[12];
    GetTextIndexed(name, sizeof(name), day - 1, kDolphinWeekday);
    if (0 == offset) {
      snprintf_P(out, out_len, PSTR("heute %02d:%02d"),
                 Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
    } else if (1 == offset) {
      snprintf_P(out, out_len, PSTR("morgen %02d:%02d"),
                 Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
    } else {
      snprintf_P(out, out_len, PSTR("%s %02d:%02d"), name,
                 Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
    }
    if (minutes_away) { *minutes_away = away; }
    return true;
  }
  return false;
}

void DolphinShow(bool json) {
  char phase_text[16];
  GetTextIndexed(phase_text, sizeof(phase_text), Dolphin.phase, kDolphinPhase);

  if (json) {
    ResponseAppend_P(PSTR(",\"Dolphin\":{\"Phase\":\"%s\",\"Connected\":%s,\"Subscribed\":%s"
                          ",\"Sent\":%d,\"Received\":%d,\"MTU\":%d"),
                     phase_text,
                     (Dolphin.client && Dolphin.client->isConnected()) ? PSTR("true") : PSTR("false"),
                     Dolphin.peer_subscribed ? PSTR("true") : PSTR("false"),
                     Dolphin.sent, Dolphin.received, Dolphin.mtu);
    ResponseAppend_P(PSTR(",\"Seen\":%d"), Dolphin.seen);
    if (Dolphin.last_reply) {
      ResponseAppend_P(PSTR(",\"LastReplySec\":%d"), TimePassedSince(Dolphin.last_reply) / 1000);
    }
    if (Dolphin.have_status) {
      // Klartext UND Rohwert: Die Oberflaeche zeigt den Text, ein Skript am
      // Broker rechnet lieber mit der Zahl. Beides gehoert in dieselbe
      // Nachricht, sonst muss die Gegenseite die Tabelle nachbauen.
      char state[24];
      GetTextIndexed(state, sizeof(state), Dolphin.sm_state, kDolphinSmState);
      char robot[24];
      GetTextIndexed(robot, sizeof(robot), Dolphin.mu_state, kDolphinMuState);
      char mode[20];
      GetTextIndexed(mode, sizeof(mode), Dolphin.mode, kDolphinMode);
      ResponseAppend_P(PSTR(",\"SMState\":%d,\"State\":\"%s\",\"MUState\":%d,\"Robot\":\"%s\""
                            ",\"CleaningMode\":%d,\"Mode\":\"%s\",\"Running\":%s"),
                       Dolphin.sm_state, state, Dolphin.mu_state, robot,
                       Dolphin.mode, mode, DolphinRunning() ? PSTR("true") : PSTR("false"));
    }
    // Der Wochenplan gehoert auch in die Telemetrie: Was die Oberflaeche
    // zeigt, muss ueber MQTT genauso zu sehen sein.
    if (Dolphin.plan_read) {
      ResponseAppend_P(PSTR(",\"Plan\":{\"Gelesen\":true,\"Wiederholen\":%s,\"Frequenz\":%d,\"Tage\":{"),
                       Dolphin.plan_repeat ? PSTR("true") : PSTR("false"),
                       Dolphin.plan_frequency);
      for (uint8_t day = 1; day <= 7; day++) {
        char name[12];
        GetTextIndexed(name, sizeof(name), day - 1, kDolphinWeekday);
        ResponseAppend_P(PSTR("%s\"%d\":{\"Tag\":\"%s\",\"An\":%s,\"Start\":\"%02d:%02d\"}"),
                         (day > 1) ? PSTR(",") : PSTR(""), day, name,
                         Dolphin.plan_on[day] ? PSTR("true") : PSTR("false"),
                         Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
      }
      ResponseAppend_P(PSTR("}"));
      char next[32];
      uint32_t away = 0;
      if (DolphinNextRun(next, sizeof(next), &away)) {
        ResponseAppend_P(PSTR(",\"Naechster\":\"%s\",\"NaechsterInMin\":%d"), next, away);
      }
      ResponseAppend_P(PSTR("}"));
    } else {
      ResponseAppend_P(PSTR(",\"Plan\":{\"Gelesen\":false}"));
    }
    ResponseAppend_P(PSTR("}"));
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(PSTR("{s}Dolphin{m}%s{e}"), phase_text);
    if (Dolphin.have_status) {
      char state[24];
      GetTextIndexed(state, sizeof(state), Dolphin.sm_state, kDolphinSmState);
      WSContentSend_PD(PSTR("{s}Dolphin Netzteil{m}%s{e}"), state);
      char robot[24];
      GetTextIndexed(robot, sizeof(robot), Dolphin.mu_state, kDolphinMuState);
      WSContentSend_PD(PSTR("{s}Dolphin Roboter{m}%s{e}"), robot);
      // Kein Filterwert: Dieser Roboter hat gar keine Filteranzeige -- in
      // seiner Konfiguration fehlt die Gruppe FILTER_BAG_INDICATION. Das Feld
      // steht zwar im Status, meldet aber durchgehend 0, und eine Null zu
      // zeigen behauptet einen Messwert, den es nicht gibt.
    }
    if (Dolphin.plan_read) {
      // Der Plan steht im Netzteil und laeuft auch dann, wenn der ESP aus
      // ist. Hier wird er nur angezeigt.
      char line[160];
      line[0] = 0;
      for (uint8_t i = 0; i < 7; i++) {
        uint8_t day = (i < 6) ? (i + 2) : 1;          // Montag zuerst, Sonntag zuletzt
        if (!Dolphin.plan_on[day]) { continue; }
        char name[12];
        GetTextIndexed(name, sizeof(name), day - 1, kDolphinWeekday);
        snprintf_P(line + strlen(line), sizeof(line) - strlen(line),
                   PSTR("%s%.2s %02d:%02d"), strlen(line) ? PSTR(", ") : PSTR(""),
                   name, Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
      }
      WSContentSend_PD(PSTR("{s}Dolphin Zeitplan{m}%s{e}"),
                       strlen(line) ? line : PSTR("kein Tag aktiv"));
      WSContentSend_PD(PSTR("{s}Dolphin Wiederholen{m}%s{e}"),
                       Dolphin.plan_repeat ? PSTR("jede Woche") : PSTR("einmalig"));
      char next[32];
      uint32_t away = 0;
      if (DolphinNextRun(next, sizeof(next), &away)) {
        WSContentSend_PD(PSTR("{s}Dolphin nächster Start{m}%s (in %d:%02d h){e}"),
                         next, away / 60, away % 60);
      }
    }
    if (Dolphin.last_reply) {
      WSContentSend_PD(PSTR("{s}Dolphin letzte Antwort{m}%d s{e}"),
                       TimePassedSince(Dolphin.last_reply) / 1000);
    }
    // Bedienung direkt auf der Startseite. la() ist Tasmotas Ajax-Funktion:
    // Sie schickt das Argument an die Hauptseite, wo FUNC_WEB_GET_ARG es
    // auswertet -- kein Seitenwechsel, kein Neuladen. Genauso hat es der
    // Berry-Vorgaenger gemacht.
    //
    // Die Tabelle muss dafuer kurz geschlossen und danach wieder geoeffnet
    // werden: Tasmota rendert den Sensorbereich als eine einzige Tabelle, und
    // ein Knopf gehoert nicht in eine Zelle.
    WSContentSend_P(PSTR("</table>"
      "<div style='display:flex;gap:4px;margin:6px 0 0 0'>"
      "<button onclick=\"la('&dstart=1');\" class='button bgrn'>Start</button>"
      "<button onclick=\"la('&dstop=1');\" class='button bred'>Stopp</button>"
      "</div>"
      "<div style='display:flex;gap:4px;margin:4px 0 6px 0'>"
      "<button onclick=\"la('&dtele=1');\">Daten neu abrufen</button>"
      "<button onclick=\"location.href='dolphin';\">Zeitplan</button>"
      "</div><table style='width:100%%'>"));
#endif
  }
}

void CmndDolphinStatus(void) {
  // DolphinShow haengt mit fuehrendem Komma an -- deshalb hier eine leere
  // Klammer oeffnen und das Komma danach ueberschreiben.
  Response_P(PSTR("{"));
  DolphinShow(true);
  ResponseAppend_P(PSTR("}"));
  char* comma = strchr(TasmotaGlobal.mqtt_data.begin(), ',');
  if (comma && (comma == TasmotaGlobal.mqtt_data.begin() + 1)) {
    memmove(comma, comma + 1, strlen(comma));
  }
}

void CmndDolphinConnect(void) {
  Dolphin.backoff_index = 0;
  Dolphin.wait_seconds = 0;
  DolphinDisconnect();
  DolphinStartScan();
  ResponseCmndDone();
}

void CmndDolphinQuiet(void) {
  if (XdrvMailbox.data_len > 0) {
    Dolphin.quiet = (1 == XdrvMailbox.payload);
    if (Dolphin.quiet) {
      DolphinDisconnect();
      Dolphin.phase = DLP_PHASE_OFF;
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: stillgelegt"));
    } else {
      Dolphin.phase = DLP_PHASE_IDLE;
      Dolphin.wait_seconds = 0;
      Dolphin.backoff_index = 0;
      AddLog(LOG_LEVEL_INFO, PSTR("DLP: wieder in Betrieb"));
    }
  }
  ResponseCmndStateText(Dolphin.quiet);
}

void CmndDolphinDebug(void) {
  if (XdrvMailbox.data_len > 0) { Dolphin.debug = (1 == XdrvMailbox.payload); }
  ResponseCmndStateText(Dolphin.debug);
}

void CmndDolphinInit(void) {
  DolphinInitSequence();
  ResponseCmndDone();
}

void CmndDolphinStart(void) {
  if (DolphinStart()) { ResponseCmndDone(); }
  else { ResponseCmndChar_P(PSTR("läuft bereits oder Zustand unbekannt")); }
}

void CmndDolphinStop(void) {
  if (DolphinStop()) { ResponseCmndDone(); }
  else { ResponseCmndChar_P(PSTR("läuft nicht")); }
}

// Ohne diesen Schalter muesste am Broker jemand auf die TelePeriod warten.
// Mit ihm (Vorgabe: an) geht jede Zustandsaenderung sofort hinaus.
void CmndDolphinPublish(void) {
  if (XdrvMailbox.data_len > 0) { Dolphin.publish = (1 == XdrvMailbox.payload); }
  ResponseCmndStateText(Dolphin.publish);
}

// Eine fertige Huelle direkt ans Netzteil geben, etwa
//   DolphinSend system_status:ab03fff807000002ac
//
// Gedacht fuer Diagnose und fuer Vorgaenge, die dieser Treiber noch nicht
// kennt. Der Praefix vor dem Doppelpunkt MUSS der Kommandoname sein -- das
// Netzteil verwirft alles andere stillschweigend, und genau daran hat dieses
// Projekt einmal einen ganzen Tag verloren.
void CmndDolphinSend(void) {
  if (0 == XdrvMailbox.data_len) {
    ResponseCmndChar_P(PSTR("erwartet <name>:<hex>"));
    return;
  }
  if (!Dolphin.peer_subscribed) {
    ResponseCmndChar_P(PSTR("Netzteil hat nicht abonniert -- nichts gesendet"));
    return;
  }
  char* colon = strchr(XdrvMailbox.data, ':');
  if (nullptr == colon) {
    ResponseCmndChar_P(PSTR("kein Doppelpunkt: der Praefix ist der Kommandoname"));
    return;
  }
  if (Dolphin.queue_len >= DLP_QUEUE_MAX) {
    ResponseCmndChar_P(PSTR("Warteschlange voll"));
    return;
  }
  if (strlen(XdrvMailbox.data) >= sizeof(Dolphin.queue[0].wire)) {
    ResponseCmndChar_P(PSTR("Rahmen zu lang fuer den Puffer"));
    return;
  }

  // Einreihen statt direkt senden. Zwei Gruende: Der Rahmen unterliegt dann
  // derselben seriellen Abarbeitung wie alles andere (immer nur einer
  // unterwegs), und der Aufruf kehrt sofort zurueck. Direktes Senden gab die
  // CPU mitten im Kommando ab -- Tasmotas Schleife lief weiter und
  // ueberschrieb die Antwort, sodass statt "Done" ein leeres "{}" ankam.
  DolphinQueueItem* item = &Dolphin.queue[Dolphin.queue_len++];
  // Ziel und Opcode aus dem Hex lesen, damit die Antwort zugeordnet werden
  // kann: ab | 03 | ziel(2) | opcode -- also Zeichen 4..9 hinter dem Doppelpunkt.
  item->dest = 0;
  item->opcode = 0;
  if (strlen(colon + 1) >= 10) {
    char buf[5] = { colon[5], colon[6], colon[7], colon[8], 0 };
    item->dest = strtoul(buf, nullptr, 16);
    char op[3] = { colon[9], colon[10], 0 };
    item->opcode = strtoul(op, nullptr, 16);
  }
  snprintf(item->wire, sizeof(item->wire), "%s", XdrvMailbox.data);
  ResponseCmndDone();
}

void CmndDolphinTelemetry(void) {
  DolphinTelemetry();
  ResponseCmndDone();
}

// Ohne Argument den Plan ausgeben, mit JSON setzen:
//   DolphinPlan {"aktiv":true,"tage":{"2":{"an":true,"stunde":13,"minute":0}}}
// Nicht genannte Tage bleiben, wie sie sind -- so laesst sich ein einzelner
// Tag aendern, ohne den Rest zu kennen.
void CmndDolphinPlan(void) {
  if (0 == XdrvMailbox.data_len) {
    Response_P(PSTR("{\"DolphinPlan\":{\"Gelesen\":%s,\"Wiederholen\":%s,\"Frequenz\":%d,\"Tage\":{"),
               Dolphin.plan_read ? PSTR("true") : PSTR("false"),
               Dolphin.plan_repeat ? PSTR("true") : PSTR("false"),
               Dolphin.plan_frequency);
    for (uint8_t day = 1; day <= 7; day++) {
      char name[12];
      GetTextIndexed(name, sizeof(name), day - 1, kDolphinWeekday);
      ResponseAppend_P(PSTR("%s\"%d\":{\"Tag\":\"%s\",\"An\":%s,\"Start\":\"%02d:%02d\"}"),
                       (day > 1) ? PSTR(",") : PSTR(""), day, name,
                       Dolphin.plan_on[day] ? PSTR("true") : PSTR("false"),
                       Dolphin.plan_hour[day], Dolphin.plan_minute[day]);
    }
    ResponseAppend_P(PSTR("}}}"));
    return;
  }

  if (!Dolphin.plan_read) {
    ResponseCmndChar_P(PSTR("noch kein Wochenplan vom Netzteil gelesen -- nicht gesetzt"));
    return;
  }

  JsonParser parser(XdrvMailbox.data);
  JsonParserObject root = parser.getRootObject();
  if (!root) {
    ResponseCmndChar_P(PSTR("kein gueltiges JSON"));
    return;
  }

  JsonParserToken active = root[PSTR("aktiv")];
  if (active) { Dolphin.plan_repeat = active.getBool(); }

  JsonParserToken days = root[PSTR("tage")];
  if (days.isObject()) {
    JsonParserObject days_obj = days.getObject();
    for (uint8_t day = 1; day <= 7; day++) {
      char key[4];
      snprintf_P(key, sizeof(key), PSTR("%d"), day);
      JsonParserToken entry = days_obj[key];
      if (!entry.isObject()) { continue; }
      JsonParserObject e = entry.getObject();
      JsonParserToken on = e[PSTR("an")];
      if (on) { Dolphin.plan_on[day] = on.getBool() ? 1 : 0; }
      JsonParserToken hour = e[PSTR("stunde")];
      if (hour) { Dolphin.plan_hour[day] = hour.getUInt() % 24; }
      JsonParserToken minute = e[PSTR("minute")];
      if (minute) { Dolphin.plan_minute[day] = minute.getUInt() % 60; }
    }
  }

  if (!DolphinWritePlan()) {
    ResponseCmndChar_P(PSTR("Plan konnte nicht gesetzt werden"));
    return;
  }
  ResponseCmndDone();
}

/*********************************************************************************************\
 * Eigene Seite fuer den Wochenplan
 *
 * Die Hauptseite bleibt schlank; der Plan bekommt eine eigene Seite mit sieben
 * Zeilen. Montag steht oben, Sonntag unten -- wie in der App und wie es hier
 * jeder erwartet, obwohl das Netzteil bei Sonntag zu zaehlen anfaengt.
\*********************************************************************************************/

#ifdef USE_WEBSERVER

const char HTTP_DOLPHIN_ROW[] PROGMEM =
  "<tr><td style='width:30%%'><b>%s</b></td>"
  "<td style='width:10%%'><input type='checkbox' name='a%d'%s></td>"
  "<td style='width:30%%'><input type='time' name='t%d' value='%02d:%02d'></td>"
  "<td style='width:30%%'>%s</td></tr>";

void DolphinWebPage(void) {
  if (!HttpCheckPriviledgedAccess()) { return; }

  static const uint8_t kOrder[7] = { 2, 3, 4, 5, 6, 7, 1 };

  // Bedienung. Abgelehnt wird mit Begruendung statt mit einem stillen "Done" --
  // kein Start, solange einer laeuft; kein Stopp, wenn nichts laeuft. Was die
  // Oberflaeche nicht kann, kann auch MQTT nicht: Beide rufen dieselben
  // Funktionen.
  const char* hinweis = nullptr;
  if (Webserver->hasArg(F("start"))) {
    hinweis = DolphinStart() ? PSTR("Start gesendet") : PSTR("Start abgelehnt: läuft bereits");
  }
  if (Webserver->hasArg(F("stop"))) {
    hinweis = DolphinStop() ? PSTR("Stopp gesendet") : PSTR("Stopp abgelehnt: läuft nicht");
  }
  if (Webserver->hasArg(F("tele"))) {
    DolphinTelemetry();
    hinweis = PSTR("Telemetrie angefordert");
  }

  if (Webserver->hasArg(F("save"))) {
    for (uint8_t i = 0; i < 7; i++) {
      uint8_t day = kOrder[i];
      char name[8];
      snprintf_P(name, sizeof(name), PSTR("a%d"), day);
      Dolphin.plan_on[day] = Webserver->hasArg(name) ? 1 : 0;
      snprintf_P(name, sizeof(name), PSTR("t%d"), day);
      if (Webserver->hasArg(name)) {
        String value = Webserver->arg(name);        // Format HH:MM
        if (value.length() >= 4) {
          Dolphin.plan_hour[day] = value.substring(0, 2).toInt() % 24;
          Dolphin.plan_minute[day] = value.substring(3, 5).toInt() % 60;
        }
      }
    }
    Dolphin.plan_repeat = Webserver->hasArg(F("rep"));
    DolphinWritePlan();
  }

  WSContentStart_P(PSTR("Dolphin Poolroboter"));
  WSContentSendStyle();

  if (hinweis) { WSContentSend_P(PSTR("<p><b>%s</b></p>"), hinweis); }

  // Diese Seite zeigt NUR den Zeitplan. Zustand, Start und Stopp stehen auf
  // der Startseite -- doppelte Bedienelemente an zwei Orten sind eine Quelle
  // fuer Missverstaendnisse, nicht fuer Komfort.
  if (!Dolphin.plan_read) {
    // Nicht schreiben, bevor gelesen wurde: Der Plan geht nur als Ganzes zum
    // Netzteil, und was wir nicht kennen, wuerden wir mit geratenen Werten
    // ueberschreiben.
    WSContentSend_P(PSTR("<p><b>Noch kein Wochenplan vom Netzteil gelesen.</b><br>"
                         "Solange das so ist, wird nichts gesetzt -- sonst gingen "
                         "Zeiten verloren, die wir nie gesehen haben.</p>"));
    WSContentSpaceButton(BUTTON_MAIN);
    WSContentStop();
    return;
  }

  WSContentSend_P(PSTR("<form method='get' action='dolphin'>"
                       "<table style='width:100%%'>"
                       "<tr><th>Tag</th><th>an</th><th>Beginn</th></tr>"));
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t day = kOrder[i];
    char name[12];
    GetTextIndexed(name, sizeof(name), day - 1, kDolphinWeekday);
    // Ende und Dauer daneben, wie in der App ("Normal | 2.5 Std"). Beides
    // wird gerechnet, nicht vom Netzteil gemeldet: Start plus Dauer des
    // eingestellten Modus.
    char ende[32];
    uint16_t dauer = DolphinModeMinutes();
    if (dauer) {
      uint16_t m = (Dolphin.plan_hour[day] * 60) + Dolphin.plan_minute[day] + dauer;
      snprintf_P(ende, sizeof(ende), PSTR("bis %02d:%02d (%d,%d Std)"),
                 (m / 60) % 24, m % 60, dauer / 60, (dauer % 60) * 10 / 60);
    } else {
      snprintf_P(ende, sizeof(ende), PSTR("&nbsp;"));
    }
    WSContentSend_P(HTTP_DOLPHIN_ROW, name,
                    day, Dolphin.plan_on[day] ? PSTR(" checked") : PSTR(""),
                    day, Dolphin.plan_hour[day], Dolphin.plan_minute[day], ende);
  }
  WSContentSend_P(PSTR("</table><p><label><input type='checkbox' name='rep'%s> "
                       "Jede Woche wiederholen</label></p>"
                       "<p><button name='save' class='button bgrn'>Speichern</button></p></form>"),
                  Dolphin.plan_repeat ? PSTR(" checked") : PSTR(""));

  char next[32];
  uint32_t away = 0;
  if (DolphinNextRun(next, sizeof(next), &away)) {
    WSContentSend_P(PSTR("<p>Nächster Start: <b>%s</b>, in %d:%02d h</p>"),
                    next, away / 60, away % 60);
  }

  WSContentSpaceButton(BUTTON_MAIN);
  WSContentStop();
}

#endif  // USE_WEBSERVER

/*********************************************************************************************\
 * Schnittstelle
\*********************************************************************************************/

void DolphinInit(void) {
  NimBLEDevice::init("Tasmota-Dolphin");
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(517);              // wie die App
  DolphinBuildServer();
  Dolphin.phase = DLP_PHASE_IDLE;
  Dolphin.wait_seconds = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("DLP: Treiber bereit, Netzteil %s"), kDolphinMac);
}

// Vor einem geplanten Neustart die Verbindung ordentlich aufloesen.
//
// Ohne das bleibt am Netzteil eine verwaiste Verbindung zurueck: Es haelt sie
// fuer gueltig, blinkt weiter blau -- und WIRBT NICHT MEHR. Danach findet es
// niemand, auch die App nicht, und nur ein Stromreset holt es zurueck.
// Am 23. August 2026 genau so passiert, ausgeloest von einem Firmware-Update
// ueber die Weboberflaeche. Gegen Stromausfall und Watchdog hilft der Haken
// nicht -- gegen jeden geplanten Neustart schon.
void DolphinSaveBeforeRestart(void) {
  if (Dolphin.client && Dolphin.client->isConnected()) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLP: trenne vor dem Neustart"));
    Dolphin.client->disconnect();
    delay(200);                        // dem Stack Zeit lassen, den Abbau zu senden
  }
}

bool Xdrv95(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      DolphinInit();
      break;
    case FUNC_SAVE_BEFORE_RESTART:
      DolphinSaveBeforeRestart();
      break;
    case FUNC_EVERY_50_MSECOND:
      DolphinStateMachine();
      break;
    case FUNC_EVERY_SECOND:
      DolphinEverySecond();
      break;
    case FUNC_JSON_APPEND:
      DolphinShow(true);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      DolphinShow(false);
      break;
    case FUNC_WEB_ADD_HANDLER:
      WebServer_on(PSTR("/dolphin"), DolphinWebPage);
      break;
    case FUNC_WEB_GET_ARG:
      // Die Knoepfe der Startseite landen hier. Abgelehnt wird mit
      // Begruendung im Log -- kein Start, solange einer laeuft.
      if (Webserver->hasArg(F("dstart"))) { DolphinStart(); }
      if (Webserver->hasArg(F("dstop"))) { DolphinStop(); }
      if (Webserver->hasArg(F("dtele"))) { DolphinTelemetry(); }
      break;
#endif
    case FUNC_COMMAND:
      result = DecodeCommand(kDolphinCommands, DolphinCommand);
      break;
  }

  return result;
}

#endif  // USE_DOLPHIN
#endif  // ESP32
