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
- Poort 1 ligt over poort 2. **Harde regel: poort 2 beweegt alleen als poort 1 ≥ p1_min** (standaard 40 %, Steve: **35 %**).
- Openen: poort 1 eerst, poort 2 vanaf poort 1 ≥ open2_from (50 %).
- Sluiten: poort 2 eerst, poort 1 vanaf poort 2 ≤ close1_below (Steve: **40 %**). Openen: open2_from (Steve: **40 %**).
  Poort 1 zakt niet onder p1_min + 8 % zolang poort 2 nog niet dicht is (fix na test 16-10).
- Uitzondering laatste stuk sluiten (16-10): staat poort 2 bij sluiten op ≤ 10 % en zijn beide bussen in orde,
  dan geldt 30 % i.p.v. p1_min: poort 2 mag sluiten zolang poort 1 ≥ 30 %, poort 1 stopt pas op 35 %.
  Vaste constanten `END2_FROM`, `END2_P1_MIN`, `END2_FLOOR_MARGIN`. Niet in reservemodus (daar blijft p1_min).
- Encoder poort 2 ≤ 1 % telt als dicht (`shut2`), ook al meldt de SNA20 nog ±2 s "closing" (eindvertraging):
  geen grens meer voor poort 1 en geen STOP naar poort 2 (fix na log 16-10 21:04, poort 1 stopte op 34 %).
- Gemeten looptijden (SNA20): poort 1 open 26,0 s / dicht 25,0 s, poort 2 open 22,9 s / dicht ±29,3 s. Bij sluiten is poort 1 sneller
  (±4,9 %/s) dan poort 2 (±4,3 %/s, onder 10 % maar ±1,6 %/s). Zo nagebootst in sim-test 24.
- Knop 1 (voetganger): poort 1 naar voet-% (50); nogmaals → poort 1 dicht; poort 2 open & 1 dicht →
  poort 2 dicht en poort 1 op voetganger; beide open → poort 2 dicht.
- Knop 2: niet alles open → alles open; alles open → alles dicht; iets beweegt → stop.
- Maglocks aan vanaf 90 %, los + wachttijd (2 s) voor sluiten, noodlossing als een poort onverwacht sluit.
  Bij sluiten één voor één: eerst maglock 2; maglock 1 pas als poort 2 ≤ close1_below + 15 % (`ML1_LEAD`),
  dus ±3,5 s voor poort 1 vertrekt (speling voor wind die poort 2 sneller doet gaan).
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
- Statuspakketten (17-10): de SNA20 stuurt tijdens beweging zelf om de ±983 ms een STA-pakket als command `0x40`
  (STA 0xC0 − 0x80), naar 00.FF: `[02] [pos hi] [pos lo] [04 open / 05 dicht]`. Origineel werd dat genegeerd
  en pollde de component elke `position_report_interval` (1 s) → botsingen ("Trailing size mismatch").
  Nu: positie uit die pakketten (`update_position`), en tijdens beweging alleen pollen als er > 1,5 s
  (`STA_LIVE_MS`) geen statuspakket kwam. Getest 17-10: STA elke ±983 ms, geen mismatch meer.
- Schatting op looptijd pas na 5 s zonder encoderwaarde (`ENCODER_FRESH_MS`, was 2 s): tijdens het uitlopen
  stuurt de SNA20 geen STA en sprong de positie terug (100 % -> 97 %). Ook beter voor de detectie "geblokkeerd".
- Bij de start van een beweging ("Gate opening/closing") geldt de STA als live: geen positievraag tot het eerste
  statuspakket (komt na 0,2-0,7 s), anders reservevraag na 1,5 s. De startvraag botste soms (log 17-10 10:48:37).

## Bestanden
- `esphome/poort-t4-dubbel.yaml`: referentiekopie van de ESPHome-config (wachtwoorden vervangen door `!secret`).
  De echte YAML staat in HA; Steve heeft daarin zijn eigen wachtwoorden.
- `docs/bedrading.md`: bedradingsschema v3 als afbeelding (`poortmodule-bedrading-v3.svg`) met onderdelenlijst,
  pinout en bouwnotities. Bron: `docs/poortmodule-bedrading-v3.html`.
- `test/sim.cpp`: simulatie (`g++ -std=c++17 test/sim.cpp -o sim && ./sim`, moet eindigen op `VIOLATIONS/FAILS: 0`).

## Node-RED / HA
Node-RED-flow geïmporteerd: alarm via `binary_sensor.garage_esp_poorthuis_t4_poort_alarm` en de busverbindingen,
tekst uit `sensor.garage_esp_poorthuis_t4_poort_melding`, melding naar `notify.mobile_app_pixel_10_pro`.

## Open punten
- Maglocks aansluiten en testen.
- Sluitvolgorde getest na de `shut2`-fix (log 16-10 21:17): poort 1 stopt niet meer. OK.
- Drukknoppen getest 17-10 (voetganger, alles open, alles dicht): OK.
- Maglocks één voor één (`ML1_LEAD`) nog testen op de poort (maglocks moeten dan aangesloten zijn).
- "Trailing size mismatch": ook gezien op poort 1 (met GND). Oorzaak: onze positievraag (±1005 ms) botste met
  het statuspakket van de SNA20 (±983 ms), ze schuiven ±25 ms/s → ongeveer elke 35–40 s rijden. Fix: zie
  Component/Statuspakketten. Testen: in het log "STA packet (0x40)" elke ±1 s, geen "Sending … 04.11" tijdens
  rijden, en geen mismatch meer. GND naar RJ pos 5 op poort 2 trekt Steve toch (beter).
