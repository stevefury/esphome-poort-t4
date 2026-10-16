// ------------------------------------------------------------------
// poort_logic.h – logica dubbele poort (Nice SNA20 x2)
//
// Zuivere C++ zonder ESPHome-afhankelijkheden: alle hardware loopt via
// de Hal-callbacks die de YAML bij het opstarten invult.
//
// Poort 1 (index 0) ligt over poort 2 (index 1).
// Harde regel: poort 2 beweegt alleen als poort 1 >= p1_min open staat.
// Openen: poort 1 eerst, poort 2 vanaf open2_from.
// Sluiten: poort 2 eerst, poort 1 pas als poort 2 <= close1_below.
// Reed 0: poort 1 dicht.  Reed 1: poort 2 volledig open.
// ------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <functional>
#include <string>

namespace poort {

enum Op : uint8_t { OP_IDLE = 0, OP_OPENING = 1, OP_CLOSING = 2 };
enum Cmd : uint8_t { C_OPEN = 0, C_CLOSE = 1, C_STOP = 2 };
// Stap-voor-stap (P.P.) toestanden van de sturing
enum PP : uint8_t { PP_CLOSED, PP_OPENING, PP_STOP_OPEN, PP_OPEN, PP_CLOSING, PP_STOP_CLOSE };

static const float TOL_END = 0.01f;       // dicht <= 1 %, open >= 99 %
static const float TOL_PART = 0.02f;      // tussenpositie bereikt
static const uint32_t BUS_GRACE_MS = 30000;   // na opstart: zo lang wachten op de bus
static const uint32_t PULSE_ON_MS = 500;
static const uint32_t PULSE_GAP_MS = 1200;
static const uint32_t CMD_RETRY_MS = 3000;
static const uint32_t CMD_FAIL_MS = 8000;
static const uint32_t STOPPED_MS = 2500;
static const uint32_t REED_MISMATCH_MS = 8000;
static const uint32_t BUS_SETTLE_MS = 3000;    // bus pas gebruiken na 3 s stabiel contact
static const uint32_t STOP_COAST_MS = 3000;    // max. uitlooptijd na onze stop
static const float ASSIST_MARGIN = 0.03f;     // poort 1 iets verder dan nodig openen
static const float CLOSE_FLOOR_MARGIN = 0.08f; // sluiten: poort 1 wacht op p1_min + dit tot poort 2 dicht is
static const uint32_t MANUAL_ML_BLOCK_MS = 4000;  // maglock niet terug aan na handmatige P.P.-puls

struct Settings {
  float voet = 0.50f;          // voetganger-opening poort 1
  float open2_from = 0.50f;    // poort 2 start bij openen als poort 1 >= dit
  float close1_below = 0.05f;  // poort 1 start bij sluiten als poort 2 <= dit
  float maglock_from = 0.90f;  // maglock aan vanaf
  float p1_min = 0.40f;        // poort 1 minimaal open voor poort 2 mag bewegen
  uint32_t release_ms = 2000;  // wachttijd na maglock lossen
  uint32_t pause_ms = 1500;    // pauze na stilstand motor voor nieuw commando
  uint32_t run_open_ms = 26000;
  uint32_t run_close_ms = 26000;
};

struct Hal {
  std::function<uint32_t()> now;
  std::function<bool(int)> bus_alive;        // geldig pakket van de motor recent ontvangen
  std::function<float(int)> bus_pos;         // 0..1
  std::function<int(int)> bus_op;            // Op
  std::function<void(int, Cmd)> bus_cmd;
  std::function<void(int, bool)> pp_out;     // P.P.-relais
  std::function<bool(int)> reed;             // 0: poort 1 dicht, 1: poort 2 volledig open
  std::function<void(int, bool)> maglock_out;
  std::function<void(int, float)> save_pos;  // naar NVS
  std::function<void(const char *)> log;
};

struct Gate {
  bool bus = false;       // positie komt van de T4-bus
  bool known = false;     // positie is betrouwbaar
  float pos = 0.0f;
  Op op = OP_IDLE;
  Op last_dir = OP_CLOSING;  // laatste bewegingsrichting (voor P.P.-stand)

  // reservemodus (schatting op tijd)
  PP pp = PP_CLOSED;
  float est = 0.0f;
  int pulses = 0;         // nog te geven pulsen
  bool pulse_on = false;
  uint32_t pulse_t = 0;   // tijdstip volgende actie
  uint32_t pp_at = 0;     // eerste puls niet voor dit tijdstip
  uint32_t ml_block_until = 0;  // maglock geblokkeerd tot (handmatige puls)
  bool ml_block = false;

  // planning
  float target = -1.0f;   // gewenste eindpositie (-1 = geen)
  float te = -1.0f;       // effectief doel deze cyclus
  float te_prev = -2.0f;
  Op part_dir = OP_IDLE;
  bool part_done = false;
  Op cmd = OP_IDLE;          // wat wij het laatst bevolen
  uint32_t cmd_t = 0;
  float cmd_pos = 0.0f;
  bool cmd_checked = true;
  bool cmd_retried = false;
  uint32_t last_moving_t = 0;
  uint32_t stop_t = 0;       // laatste STOP via de bus

  // maglock
  bool ml = false;
  bool releasing = false;
  uint32_t release_until = 0;

  // controle en opslag
  float saved = -1.0f;
  uint32_t mismatch_since = 0;
  bool reed_alarm = false;
  bool init_done = false;     // opstart afgehandeld (bus of NVS)
  bool stop_pending = false;
  uint32_t alive_since = 0;
  float stuck_pos = -1.0f;
  uint32_t stuck_t = 0;
};

class Controller {
 public:
  Hal hal;
  Settings set;
  Gate g[2];
  std::string msg = "Opstarten";
  bool alarm_cmd = false;
  bool booted = false;
  uint32_t boot_t = 0;
  uint32_t last_t = 0;

  // veilige tijd: 0 zolang de Hal nog niet ingevuld is
  uint32_t now_() const { return hal.now ? hal.now() : 0; }

  // ---------------- opstart ----------------
  void begin(float saved1, float saved2) {
    boot_t = now_();
    last_t = boot_t;
    g[0].saved = saved1;
    g[1].saved = saved2;
    booted = true;
    say("Opgestart, wacht op de bus");
  }

  // ---------------- verzoeken ----------------
  void knop1() {
    if (!booted) return;  // nog niet opgestart  // voetganger
    if (moving()) { stop_all("Knop 1 kant: stop"); return; }
    if (!all_known()) { say("Knop 1 kant geweigerd: stand onbekend"); alarm_cmd = true; return; }
    if (set.voet < set.p1_min) say("Let op: voetganger-% ligt onder 'poort 1 minimaal open'");
    const bool c1 = g[0].pos <= TOL_END, c2 = g[1].pos <= TOL_END;
    if (c1 && c2) {
      say("Knop 1 kant: poort 1 naar voetganger");
      target(0, set.voet);
    } else if (!c2) {
      say("Knop 1 kant: poort 2 dicht, poort 1 op voetganger");
      target(1, 0.0f);
      if (g[0].pos < set.voet - TOL_PART) target(0, set.voet);
    } else {
      say("Knop 1 kant: poort 1 dicht");
      target(0, 0.0f);
    }
  }
  void knop2() {
    if (!booted) return;  // nog niet opgestart
    if (moving()) { stop_all("Knop 2 kanten: stop"); return; }
    if (!all_known()) { say("Knop 2 kanten geweigerd: stand onbekend"); alarm_cmd = true; return; }
    if (g[0].pos >= 1.0f - TOL_END && g[1].pos >= 1.0f - TOL_END) close_all();
    else open_all();
  }
  void open_all() {
    if (!booted) return;  // nog niet opgestart
    if (!all_known()) { say("Openen geweigerd: stand onbekend"); alarm_cmd = true; return; }
    say("Alles open");
    target(0, 1.0f);
    target(1, 1.0f);
  }
  void close_all() {
    if (!booted) return;  // nog niet opgestart
    if (!all_known()) { say("Sluiten geweigerd: stand onbekend"); alarm_cmd = true; return; }
    say("Alles dicht");
    target(0, 0.0f);
    target(1, 0.0f);
  }
  void set_target(int i, float t) {
    if (!booted) return;  // nog niet opgestart
    if (!g[i].known || (i == 1 && !g[0].known)) { say("Commando geweigerd: stand onbekend"); alarm_cmd = true; return; }
    char b[64];
    snprintf(b, sizeof(b), "Poort %d naar %.0f%%", i + 1, t * 100.0f);
    say(b);
    target(i, t);
  }
  void stop_all(const char *why = "Alles stop") {
    if (!booted) return;  // nog niet opgestart
    say(why);
    for (int i = 0; i < 2; i++) {
      g[i].target = -1.0f;
      g[i].te = -1.0f;
      g[i].releasing = false;
      if (!g[i].bus && !g[i].pulse_on) g[i].pulses = 0;
      if (g[i].op != OP_IDLE || g[i].cmd != OP_IDLE || !g[i].bus) g[i].stop_pending = true;
      g[i].cmd = OP_IDLE;
    }
  }
  // stand bevestigen (alleen nodig in reservemodus bij onbekende stand)
  void confirm(float p1, float p2) {
    if (!booted) return;  // nog niet opgestart
    const float p[2] = {p1, p2};
    for (int i = 0; i < 2; i++) {
      if (g[i].bus) continue;
      g[i].est = g[i].pos = p[i];
      g[i].pp = p[i] >= 0.5f ? PP_OPEN : PP_CLOSED;
      g[i].op = OP_IDLE;
      g[i].known = true;
      g[i].reed_alarm = false;
      g[i].mismatch_since = 0;
    }
    alarm_cmd = false;
    say("Stand bevestigd");
  }
  void reset_alarm() {
    alarm_cmd = false;
    say("Alarm gereset");
  }
  // handmatige P.P.-puls (test / noodbediening), buiten de volgorde-logica om
  void manual_pp(int i) {
    if (!booted) return;
    Gate &x = g[i];
    char b[80];
    if (x.pulse_on || x.pulses > 0) {
      snprintf(b, sizeof(b), "P.P. poort %d: vorige puls nog bezig", i + 1);
      say(b);
      return;
    }
    if (i == 1 && g[0].known && g[0].pos + 0.005f < set.p1_min) {
      snprintf(b, sizeof(b), "P.P. poort 2 geweigerd: poort 1 staat maar %.0f%% open", g[0].pos * 100.0f);
      say(b);
      return;
    }
    // lopende opdrachten annuleren zodat de logica niet tegenwerkt
    for (auto &y : g) {
      y.target = -1.0f;
      y.te = -1.0f;
      y.cmd = OP_IDLE;
      y.cmd_checked = true;
      y.stop_pending = false;
    }
    const uint32_t now = now_();
    uint32_t delay = 0;
    if (x.ml) {  // eerst maglock lossen
      x.ml = false;
      hal.maglock_out(i, false);
      delay = set.release_ms;
    }
    x.pp_at = now + delay;
    x.pulse_t = now - PULSE_GAP_MS;
    x.pulses = 1;
    x.ml_block = true;
    x.ml_block_until = now + delay + MANUAL_ML_BLOCK_MS;
    snprintf(b, sizeof(b), "P.P. poort %d: handmatige puls%s", i + 1, delay ? " (na maglock lossen)" : "");
    say(b);
  }

  // ---------------- status ----------------
  bool moving() const {
    for (auto &x : g)
      if (x.op != OP_IDLE || x.target >= 0.0f || x.releasing || x.pulses > 0) return true;
    return false;
  }
  bool all_known() const { return g[0].known && g[1].known; }
  bool reserve(int i) const { return booted && now_() - boot_t >= BUS_GRACE_MS && !g[i].bus; }
  bool alarm() const {
    if (!booted) return false;
    return alarm_cmd || g[0].reed_alarm || g[1].reed_alarm ||
           (now_() - boot_t >= BUS_GRACE_MS && (!g[0].known || !g[1].known));
  }
  const char *mode_text() const {
    if (!booted || now_() - boot_t < BUS_GRACE_MS) {
      if (g[0].bus && g[1].bus) return "Bus";
      return "Opstarten";
    }
    if (!all_known()) return "Stand onbekend";
    if (g[0].bus && g[1].bus) return "Bus";
    if (!g[0].bus && !g[1].bus) return "Reserve (beide)";
    return g[0].bus ? "Reserve (poort 2)" : "Reserve (poort 1)";
  }

  // ---------------- hoofdlus (elke ~100 ms) ----------------
  void loop() {
    if (!booted) return;
    const uint32_t now = now_();
    const uint32_t dt = now - last_t;
    last_t = now;
    for (int i = 0; i < 2; i++) update_source(i, now, dt);
    for (int i = 0; i < 2; i++) pulse_engine(i, now);
    for (int i = 0; i < 2; i++) {
      Gate &x = g[i];
      if (!x.stop_pending) continue;
      if (x.bus) { drive(i, OP_IDLE, true); x.cmd = OP_IDLE; x.stop_pending = false; }
      else if (x.pulses == 0 && !x.pulse_on) {
        if (pp_matches(x.pp, OP_IDLE)) x.stop_pending = false;
        else { drive(i, OP_IDLE, true); x.cmd = OP_IDLE; }
      }
    }
    plan(now);
    for (int i = 0; i < 2; i++) {
      maglock(i, now);
      check_cmd(i, now);
      check_reed(i, now);
      persist(i);
    }
  }

 private:
  void say(const std::string &m) {
    msg = m;
    if (hal.log) hal.log(m.c_str());
  }
  void target(int i, float t) {
    g[i].target = t;
    g[i].part_done = false;
    g[i].te_prev = -2.0f;
  }

  // -------- bron van de positie: bus of schatting --------
  void update_source(int i, uint32_t now, uint32_t dt) {
    Gate &x = g[i];
    const bool raw_alive = hal.bus_alive(i);
    if (!raw_alive) x.alive_since = 0;
    else if (x.alive_since == 0) x.alive_since = now ? now : 1;
    const bool alive = raw_alive && (x.bus || now - x.alive_since >= BUS_SETTLE_MS);
    char b[96];
    if (alive) {
      if (!x.bus) {
        x.init_done = true;
        x.bus = true;
        x.known = true;
        x.reed_alarm = false;
        x.pulses = 0;
        if (x.pulse_on) { hal.pp_out(i, false); x.pulse_on = false; }
        snprintf(b, sizeof(b), "Bus poort %d actief", i + 1);
        say(b);
      }
      x.pos = hal.bus_pos(i);
      x.op = static_cast<Op>(hal.bus_op(i));
      if (x.op != OP_IDLE) x.last_dir = x.op;
    } else {
      if (x.bus) {
        // bus net weggevallen: verder vanaf de laatste bekende toestand
        x.bus = false;
        x.est = x.pos;
        if (x.op == OP_OPENING) x.pp = PP_OPENING;
        else if (x.op == OP_CLOSING) x.pp = PP_CLOSING;
        else if (x.pos <= TOL_END) x.pp = PP_CLOSED;
        else if (x.pos >= 1.0f - TOL_END) x.pp = PP_OPEN;
        else x.pp = (x.last_dir == OP_OPENING) ? PP_STOP_OPEN : PP_STOP_CLOSE;
        snprintf(b, sizeof(b), "BUS POORT %d VERLOREN - reservemodus (P.P.)", i + 1);
        say(b);
      } else if (!x.init_done && now - boot_t >= BUS_GRACE_MS) {
        // na opstart zonder bus: laatste stand uit NVS
        x.init_done = true;
        const float s = x.saved;
        if (s > -0.5f && s <= TOL_END) { x.est = 0.0f; x.pp = PP_CLOSED; x.known = true; }
        else if (s >= 1.0f - TOL_END) { x.est = 1.0f; x.pp = PP_OPEN; x.known = true; }
        else { x.est = s > 0.0f ? s : 0.0f; x.pp = PP_STOP_CLOSE; x.known = false; }
        snprintf(b, sizeof(b), "Geen bus poort %d na opstart - %s", i + 1,
                 x.known ? "laatste stand uit geheugen" : "STAND ONBEKEND, bevestig via camera");
        say(b);
      }
      if (!x.init_done) { x.op = OP_IDLE; return; }  // nog in opstart-wachttijd
      // schatting bijwerken
      if (x.pp == PP_OPENING) {
        x.est += static_cast<float>(dt) / static_cast<float>(set.run_open_ms);
        if (x.est >= 1.0f) { x.est = 1.0f; x.pp = PP_OPEN; }
      } else if (x.pp == PP_CLOSING) {
        x.est -= static_cast<float>(dt) / static_cast<float>(set.run_close_ms);
        if (x.est <= 0.0f) { x.est = 0.0f; x.pp = PP_CLOSED; }
      }
      // reeds corrigeren eindstanden
      if (i == 0 && hal.reed(0) && x.pp == PP_CLOSING) { x.est = 0.0f; x.pp = PP_CLOSED; }
      if (i == 1 && hal.reed(1) && x.pp == PP_OPENING) { x.est = 1.0f; x.pp = PP_OPEN; }
      x.pos = x.est;
      x.op = (x.pp == PP_OPENING) ? OP_OPENING : (x.pp == PP_CLOSING) ? OP_CLOSING : OP_IDLE;
      if (x.op != OP_IDLE) x.last_dir = x.op;
    }
    if (x.op != OP_IDLE) x.last_moving_t = now;
  }

  // -------- P.P.-pulsen --------
  static PP pp_next(PP s) {
    switch (s) {
      case PP_CLOSED: return PP_OPENING;
      case PP_OPENING: return PP_STOP_OPEN;
      case PP_STOP_OPEN: return PP_CLOSING;
      case PP_OPEN: return PP_CLOSING;
      case PP_CLOSING: return PP_STOP_CLOSE;
      case PP_STOP_CLOSE: return PP_OPENING;
    }
    return s;
  }
  static bool pp_matches(PP s, Op want) {
    if (want == OP_OPENING) return s == PP_OPENING;
    if (want == OP_CLOSING) return s == PP_CLOSING;
    return s != PP_OPENING && s != PP_CLOSING;
  }
  void pulse_engine(int i, uint32_t now) {
    Gate &x = g[i];
    if (x.bus && !x.pulse_on && x.pulses == 0) return;  // met bus alleen handmatige pulsen
    if (x.pulse_on) {
      if (now - x.pulse_t >= PULSE_ON_MS) {
        hal.pp_out(i, false);
        x.pulse_on = false;
        x.pulse_t = now;
      }
      return;
    }
    if (x.pulses > 0 && now - x.pulse_t >= PULSE_GAP_MS && static_cast<int32_t>(now - x.pp_at) >= 0) {
      hal.pp_out(i, true);
      x.pulse_on = true;
      x.pulse_t = now;
      x.pulses--;
      if (!x.bus) {  // met bus komt de toestand van de motor zelf
        x.pp = pp_next(x.pp);  // de sturing gaat naar de volgende stap
        x.op = (x.pp == PP_OPENING) ? OP_OPENING : (x.pp == PP_CLOSING) ? OP_CLOSING : OP_IDLE;
      }
    }
  }

  // -------- bewegingsopdracht naar één poort --------
  // returns false zolang er nog gewacht wordt (bv. maglock lossen)
  bool drive(int i, Op want, bool force = false) {
    Gate &x = g[i];
    const uint32_t now = now_();
    if (want == OP_CLOSING) {
      if (x.ml || (x.releasing && now < x.release_until)) {
        if (!x.releasing) {
          x.releasing = true;
          x.release_until = now + set.release_ms;
          x.ml = false;
          hal.maglock_out(i, false);
        }
        if (now < x.release_until) return false;
      }
    }
    if (!force && x.cmd == want) return true;
    if (x.bus && want != OP_IDLE) {
      // motor nog aan het uitlopen na onze stop, of net stil: even wachten,
      // anders negeert de sturing het commando
      if (x.op != OP_IDLE) {
        if (x.cmd == OP_IDLE && now - x.stop_t < STOP_COAST_MS) return false;
      } else if (now - x.last_moving_t < set.pause_ms) {
        return false;
      }
    }
    if (x.bus) {
      hal.bus_cmd(i, want == OP_OPENING ? C_OPEN : want == OP_CLOSING ? C_CLOSE : C_STOP);
      if (want == OP_IDLE) x.stop_t = now;
    } else {
      if (x.pulses == 0 && !x.pulse_on) {
        PP s = x.pp;
        int n = 0;
        while (!pp_matches(s, want) && n < 4) { s = pp_next(s); n++; }
        // volledig open + openen of volledig dicht + sluiten: niets te doen
        if ((want == OP_OPENING && x.pp == PP_OPEN) || (want == OP_CLOSING && x.pp == PP_CLOSED)) n = 0;
        x.pulses = n;
        x.pp_at = now;
      } else {
        return false;  // wacht tot vorige pulsen klaar zijn
      }
    }
    x.cmd = want;
    x.cmd_t = now;
    x.cmd_pos = x.pos;
    x.cmd_checked = (want == OP_IDLE) || !x.bus;
    x.cmd_retried = false;
    return true;
  }

  bool reached(int i) const {
    const Gate &x = g[i];
    if (x.target < 0.0f) return true;
    if (x.target <= TOL_END) return x.pos <= TOL_END && x.op == OP_IDLE;
    if (x.target >= 1.0f - TOL_END) return x.pos >= 1.0f - TOL_END && x.op == OP_IDLE;
    return x.part_done && x.te == x.target && x.op == OP_IDLE;
  }

  // naar effectief doel bewegen; hold = stilstaan (niet sluiten)
  void move_toward(int i, float te, bool may_close) {
    Gate &x = g[i];
    x.te = te;
    if (te != x.te_prev) {
      x.te_prev = te;
      x.part_done = false;
      x.part_dir = (te > x.pos) ? OP_OPENING : OP_CLOSING;
    }
    if (te < 0.0f) {  // geen doel meer: niet verder bewegen
      if (x.cmd != OP_IDLE) { drive(i, OP_IDLE, true); x.cmd = OP_IDLE; }
      return;
    }
    if (te <= TOL_END) {
      if (x.pos <= TOL_END && x.op == OP_IDLE) { x.cmd = OP_IDLE; return; }
      if (!may_close) { hold(i); return; }
      drive(i, OP_CLOSING);
      return;
    }
    if (te >= 1.0f - TOL_END) {
      if (x.pos >= 1.0f - TOL_END && x.op == OP_IDLE) { x.cmd = OP_IDLE; return; }
      drive(i, OP_OPENING);
      return;
    }
    // tussenpositie
    if (x.part_done) return;
    if (x.part_dir == OP_CLOSING && !may_close) { hold(i); return; }
    const bool crossed = (x.part_dir == OP_OPENING) ? (x.pos >= te - 0.01f) : (x.pos <= te + 0.01f);
    if (crossed) {
      if (x.op != OP_IDLE) drive(i, OP_IDLE, true);
      x.cmd = OP_IDLE;
      x.part_done = true;
      return;
    }
    if (std::fabs(x.pos - te) <= TOL_PART && x.op == OP_IDLE) { x.part_done = true; x.cmd = OP_IDLE; return; }
    drive(i, x.part_dir);
  }
  void hold(int i) {
    Gate &x = g[i];
    // stop niet elke cyclus opnieuw sturen terwijl de motor uitloopt
    if (x.cmd != OP_IDLE || (x.op == OP_CLOSING && (!x.bus || now_() - x.stop_t > 1000))) { drive(i, OP_IDLE, true); x.cmd = OP_IDLE; }
  }
  void prerelease(int i, uint32_t now) {
    Gate &x = g[i];
    if (x.ml && !x.releasing) {
      x.releasing = true;
      x.release_until = now + set.release_ms;
      x.ml = false;
      hal.maglock_out(i, false);
    }
  }

  // -------- planner: volgorde en vergrendeling --------
  void plan(uint32_t now) {
    // doelen op onbekende poorten annuleren
    for (int i = 0; i < 2; i++) {
      if (g[i].target >= 0.0f && (!g[i].known || (i == 1 && !g[0].known))) {
        char b[64];
        snprintf(b, sizeof(b), "Poort %d: stand onbekend, opdracht geannuleerd", i + 1);
        g[i].target = -1.0f;
        alarm_cmd = true;
        say(b);
      }
    }
    const bool pend2 = !reached(1);
    float need = set.p1_min;
    if (pend2 && g[1].target > g[1].pos) need = std::fmax(set.p1_min, set.open2_from);

    // poort 1
    float te1 = g[0].target;
    bool may_close1 = true;
    if (pend2) {
      const float assist = std::fmin(1.0f, need + ASSIST_MARGIN);
      if (g[0].pos < assist - 0.01f && (te1 < assist || g[0].pos + 0.005f < need)) {
        te1 = (te1 > assist) ? te1 : assist;  // eerst ver genoeg open voor poort 2
      } else {
        // poort 1 mag sluiten zodra poort 2 onder close1_below zit, maar zakt niet
        // onder p1_min + marge zolang poort 2 nog niet helemaal dicht is
        const float floor1 = std::fmin(1.0f, set.p1_min + CLOSE_FLOOR_MARGIN);
        may_close1 = (g[1].target <= TOL_END && g[1].pos <= set.close1_below) && g[0].pos > floor1;
      }
    }
    // sluitreeks: maglock van poort 1 al lossen terwijl poort 2 sluit
    if (pend2 && g[0].target >= 0.0f && g[0].target < g[0].pos - 0.01f) prerelease(0, now);
    if (te1 >= 0.0f || g[0].te >= 0.0f) move_toward(0, te1, may_close1);

    // poort 2
    if (pend2) {
      if (g[0].pos + 0.005f < need || g[0].pos + 0.005f < set.p1_min) {
        hold(1);  // wachten op poort 1
        g[1].te = g[1].target;
      } else {
        move_toward(1, g[1].target, true);
      }
    } else if (g[1].target >= 0.0f) {
      move_toward(1, g[1].target, true);
    }

    // klaar? doelen wissen
    for (int i = 0; i < 2; i++) {
      if (g[i].target >= 0.0f && reached(i) && !(i == 0 && pend2)) {
        g[i].target = -1.0f;
        g[i].te = -1.0f;
        g[i].cmd = OP_IDLE;
      }
    }
    if (g[0].target < 0.0f && g[1].target < 0.0f) { g[0].te = g[1].te = -1.0f; }

    // onverwacht gestopt (obstakel / krachtdetectie) – alleen met bus
    for (int i = 0; i < 2; i++) {
      Gate &x = g[i];
      if (x.bus && x.target >= 0.0f && (x.cmd == OP_OPENING || x.cmd == OP_CLOSING) && x.cmd_checked &&
          x.op == OP_IDLE && !reached(i) && now - x.last_moving_t > STOPPED_MS && now - x.cmd_t > STOPPED_MS) {
        char b[80];
        snprintf(b, sizeof(b), "Poort %d onverwacht gestopt op %.0f%% (obstakel?)", i + 1, x.pos * 100.0f);
        alarm_cmd = true;
        stop_all(b);
      }
    }
  }

  // -------- maglocks --------
  void maglock(int i, uint32_t now) {
    Gate &x = g[i];
    // noodlossing: poort sluit terwijl maglock aan staat
    if (x.op == OP_CLOSING && x.ml) {
      x.ml = false;
      hal.maglock_out(i, false);
      if (!x.releasing) {
        char b[64];
        snprintf(b, sizeof(b), "Maglock %d noodgelost (poort sluit)", i + 1);
        say(b);
      }
    }
    if (x.releasing && (x.op == OP_CLOSING || x.pos < set.maglock_from - 0.05f || x.target < 0.0f))
      if (now >= x.release_until) x.releasing = false;
    if (x.ml_block && static_cast<int32_t>(now - x.ml_block_until) >= 0 && x.pulses == 0 && !x.pulse_on)
      x.ml_block = false;
    bool want;
    if (x.ml_block) {
      want = false;  // net handmatig gepulst: niet terug vergrendelen
    } else if (!x.known) {
      want = (i == 1 && !x.bus && hal.reed(1));
    } else {
      const bool going_down = x.target >= 0.0f && x.target < x.pos - 0.01f;
      want = x.pos >= set.maglock_from && x.op != OP_CLOSING && !x.releasing;
      if (i == 1 && !x.bus && hal.reed(1) && x.op != OP_CLOSING && !x.releasing) want = true;
      if (going_down && want) want = x.ml;  // niet opnieuw inschakelen; lossen gebeurt in drive()
    }
    if (want != x.ml) {
      x.ml = want;
      hal.maglock_out(i, want);
    }
  }

  // -------- commando uitgevoerd? (alleen met bus) --------
  void check_cmd(int i, uint32_t now) {
    Gate &x = g[i];
    // beweegt volgens de motor, maar de positie blijft gelijk
    if (x.bus && x.op != OP_IDLE) {
      if (x.stuck_pos < 0.0f || std::fabs(x.pos - x.stuck_pos) >= 0.01f) { x.stuck_pos = x.pos; x.stuck_t = now; }
      else if (now - x.stuck_t > 6000) {
        x.stuck_pos = -1.0f;
        alarm_cmd = true;
        char b[64];
        snprintf(b, sizeof(b), "Poort %d geblokkeerd (positie verandert niet)", i + 1);
        stop_all(b);
        return;
      }
    } else {
      x.stuck_pos = -1.0f;
    }
    if (x.cmd_checked || !x.bus) return;
    if (x.op == x.cmd || std::fabs(x.pos - x.cmd_pos) >= 0.01f) { x.cmd_checked = true; return; }
    if (!x.cmd_retried && now - x.cmd_t > CMD_RETRY_MS) {
      x.cmd_retried = true;
      hal.bus_cmd(i, x.cmd == OP_OPENING ? C_OPEN : C_CLOSE);
      return;
    }
    if (now - x.cmd_t > CMD_FAIL_MS) {
      x.cmd_checked = true;
      alarm_cmd = true;
      char b[64];
      snprintf(b, sizeof(b), "Poort %d reageert niet op commando", i + 1);
      stop_all(b);
    }
  }

  // -------- reed tegen positie --------
  void check_reed(int i, uint32_t now) {
    Gate &x = g[i];
    if (!x.known || x.op != OP_IDLE || x.pulses > 0) { x.mismatch_since = 0; return; }
    const bool mism = (i == 0) ? ((x.pos <= TOL_END) != hal.reed(0))
                               : ((x.pos >= 1.0f - TOL_END) != hal.reed(1));
    if (!mism) {
      x.mismatch_since = 0;
      if (x.reed_alarm) x.reed_alarm = false;
      return;
    }
    if (x.mismatch_since == 0) { x.mismatch_since = now ? now : 1; return; }
    if (now - x.mismatch_since < REED_MISMATCH_MS || x.reed_alarm) return;
    x.reed_alarm = true;
    char b[96];
    if (x.bus) {
      snprintf(b, sizeof(b), "Poort %d: reed en encoder zijn het oneens", i + 1);
    } else {
      x.known = false;  // in reservemodus: stand niet meer betrouwbaar
      x.target = -1.0f;
      snprintf(b, sizeof(b), "Poort %d: reed klopt niet met schatting - STAND ONBEKEND", i + 1);
    }
    say(b);
  }

  // -------- NVS --------
  void persist(int i) {
    Gate &x = g[i];
    if (!x.known || x.op != OP_IDLE || x.pulses > 0) return;
    if (x.saved < -0.5f || std::fabs(x.pos - x.saved) > 0.01f) {
      x.saved = x.pos;
      hal.save_pos(i, x.pos);
    }
  }
};

inline Controller &ctl() {
  static Controller c;
  return c;
}

}  // namespace poort
