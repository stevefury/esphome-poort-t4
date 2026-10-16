# Poortmodule dubbele poort – projectcontext voor Claude

Eigenaar: Steve (België). **Antwoord altijd in het Nederlands (Vlaams), kort en concreet.**

## Werkafspraken (strikt volgen)
- **Niets aan zijn werkende opstelling veranderen zonder het eerst te vragen.**
- Keuzes (config, toolchain, aanpak) als **keuze voorleggen**, niet zelf beslissen.
- **Altijd eerst voorstellen en wachten op een expliciet "ok, bouw maar"** voor code geschreven of gewijzigd wordt.
- Bij YAML-wijzigingen: **altijd de volledige YAML in de chat plakken** (geen stukjes).
- Logica eerst testen in de simulatie (`test/sim.cpp`) voor ze naar de poort gaat.
- Na elke wijziging aan de component of logica: committen en pushen naar `main` van deze repo;
  Steve drukt dan in ESPHome op Install (de YAML haalt de component op met `refresh: 0s`).
- Meldingen/notificaties doet Steve in **Node-RED**, niet in HA-automations.

## Doel
ESPHome-module (ESP32-S3 DevKitC-1 N16R8, node `esp-poort-t4-test`, HA-entiteiten
`garage_esp_poorthuis_t4_*`) voor een dubbele poort met **twee Nice SNA20**-sturingen via **Bus T4**.
Vervangt een oude D1 mini met relais (die gaat eraf, ze draaien nooit samen).

## Hardware (alle signalen op header J1)
| GPIO | Functie |
|---|---|
| 4 / 5 | TX / RX poort 1 → Seeed Grove RS485 (MAX13487E, auto-richting, VCC 3V3) |
| 6 / 7 | TX / RX poort 2 → tweede Grove RS485 |
| 15 / 16 | Maglock poort 1 / 2 via D4184 MOSFET-module (EL817-opto-ingang, geen pull-down nodig) |
| 17 / 18 | P.P. (stap) poort 1 / 2 via losse PC817 + 330 Ω (hoog = puls) |
| 9 | Reed poort 1: **gesloten = poort 1 DICHT** (NO-reed) |
| 10 | Reed poort 2: **gesloten = poort 2 VOLLEDIG OPEN** (NO-reed) |
| 11 / 12 | Drukknop 1 kant (voetganger) / 2 kanten |

- RS485-klem: B → RJ pos 3 (CANH), A → RJ pos 4 (CANL), GND → pos 5. Pos 2 (+28 V) niet aansluiten.
- Ingangen: 3,3 kΩ pull-up naar 3V3, 100 nF (reeds) / 150 nF (knoppen) naar GND, 470 Ω serie.
- Voeding: 12 V PSU → XY-2596 buck 5,2 V → 5V-pin (1N5819). Maglocks 12 V met 1N4007 over elke maglock
  (kathode op +, anode op LOAD van het D4184-bordje).
- PC817: 1 anode ← 330 Ω ← GPIO, 2 kathode → GND, 4 collector → P.P.(+), 3 emitter → COM(−).
- Status bouw: printplaat gebouwd. P.P. poort 1 en 2 aangesloten, polariteit gemeten en getest vanuit HA (werkt).
  Maglocks nog **niet** aangesloten.

## Bus T4 (Nice)
- 19200 8N1 met break. Motoradres 00.03, ons adres 0x0066. Encoder poort 1 0..1288, poort 2 0..1514.
- Werkt met beide bussen (`[UART0]` = poort 1, `[UART1]` = poort 2 in de log).
- SNA20 negeert een commando vlak na stilstand → instelling "Pauze na stilstand motor" (1,5 s).

## Logica (`components/bus_t4/poort_logic.h`, namespace `poort`)
Zuivere C++ met HAL-callbacks (ingevuld in `on_boot` van de YAML), singleton `poort::ctl()`.
- Poort 1 ligt over poort 2. **Harde regel: poort 2 beweegt alleen als poort 1 ≥ p1_min (40 %).**
- Openen: poort 1 eerst, poort 2 vanaf poort 1 ≥ open2_from (50 %).
- Sluiten: poort 2 eerst, poort 1 vanaf poort 2 ≤ close1_below (Steve gebruikt **50 %**).
  Poort 1 zakt niet onder p1_min + 8 % zolang poort 2 nog niet dicht is (fix na test 16-10).
- Uitzondering laatste stuk sluiten (16-10): staat poort 2 bij sluiten op ≤ 10 % en zijn beide bussen in orde,
  dan geldt 30 % i.p.v. p1_min: poort 2 mag sluiten zolang poort 1 ≥ 30 %, poort 1 stopt pas op 35 %.
  Vaste constanten `END2_FROM`, `END2_P1_MIN`, `END2_FLOOR_MARGIN`. Niet in reservemodus (daar blijft p1_min).
- Knop 1 (voetganger): poort 1 naar voet-% (50); nogmaals → poort 1 dicht; poort 2 open & 1 dicht →
  poort 2 dicht en poort 1 op voetganger; beide open → poort 2 dicht.
- Knop 2: niet alles open → alles open; alles open → alles dicht; iets beweegt → stop.
- Maglocks aan vanaf 90 %, los + wachttijd (2 s) voor sluiten, noodlossing als een poort onverwacht sluit.
  Reeds beslissen de maglock niet (behalve in reservemodus).
- Reeds controleren enkel en geven een waarschuwing bij verschil met de encoder.
- Bus weg > 30 s → reservemodus met P.P.-pulsen op tijd vanaf de laatste bekende stand (NVS).
  Stand onzeker → niets bewegen; Steve bevestigt via camera met de knoppen "Stand bevestigen".
- Controle "commando niet uitgevoerd" (herhaling na 3 s, alarm na 8 s) en "geblokkeerd" (6 s).
- Handmatige knoppen "P.P. poort 1/2 (1 puls)" (config): annuleert lopende opdrachten, lost eerst de maglock;
  P.P. 2 geweigerd als poort 1 zeker onder p1_min staat.
- Alle instellingen zijn HA-numbers met `restore_value` (NVS), geen herflash nodig.

## Component (`components/bus_t4`)
Aangepaste makstech/esphome-BusT4 v1: `MULTI_CONF`, uart-nummer in logs, `get_last_rx_ms()` voor busbewaking,
aparte NVS-sleutel per cover (`set_pref_salt`), `#include "poort_logic.h"` onderaan `bus_t4_component.h`.

## Bestanden
- `esphome/poort-t4-dubbel.yaml`: referentiekopie van de ESPHome-config (wachtwoorden vervangen door `!secret`).
  De echte YAML staat in HA; Steve heeft daarin zijn eigen wachtwoorden.
- `test/sim.cpp`: simulatie (`g++ -std=c++17 test/sim.cpp -o sim && ./sim`, moet eindigen op `VIOLATIONS/FAILS: 0`).

## Node-RED / HA
Node-RED-flow geïmporteerd: alarm via `binary_sensor.garage_esp_poorthuis_t4_poort_alarm` en de busverbindingen,
tekst uit `sensor.garage_esp_poorthuis_t4_poort_melding`, melding naar `notify.mobile_app_pixel_10_pro`.

## Open punten
- Maglocks aansluiten en testen.
- Sluitvolgorde testen met de uitzondering laatste stuk (30 % / 10 %). In de simulatie stopt poort 1 nog kort
  op 48 % als poort 2 merkbaar trager is dan poort 1 (test 20.0); bij gelijke snelheid niet meer (test 20.3).
- Kijken of "Trailing size mismatch" vaker voorkomt en op welke UART.
