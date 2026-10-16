#include "../components/bus_t4/poort_logic.h"
#include <cstdio>
#include <cassert>
#include <vector>
using namespace poort;

struct Real { float pos=0; int op=0; int pp=0; bool link=true; bool ppin=false; float speed=1.0f/26000; uint32_t stop_t=0; };
int ignored=0, stops_sent=0;
Real R[2]; uint32_t T=0; bool ML[2]={0,0}; uint32_t ml_off_t[2]={0,0}; uint32_t close_t[2]={0,0};
int violations=0; bool verbose=false; float saved[2]={0,0};
std::vector<std::string> logs;

void violate(const char*w){ violations++; printf("  !!! VIOLATION t=%u: %s (p1=%.2f p2=%.2f)\n",T,w,R[0].pos,R[1].pos); }

void real_cmd(int i, int c){ // 0 open 1 close 2 stop
  Real&r=R[i];
  if(c==2){ stops_sent++; if(r.op) r.stop_t=T; r.op=0; return; }
  if(r.op==0 && T-r.stop_t<1000){ ignored++; if(verbose)printf("  t=%u motor %d negeert commando\n",T,i); return; }
  if(c==0){ if(r.pos<1) r.op=1; }
  else if(c==1){ if(r.pos>0) { r.op=2; close_t[i]=T; } }
}
void pp_edge(int i){ // SbS: closed->open, opening->stop, stop->reverse, open->close, closing->stop
  Real&r=R[i];
  if(r.op==1){ r.op=0; r.pp=1; }        // stopped after opening
  else if(r.op==2){ r.op=0; r.pp=2; }   // stopped after closing
  else { if(r.pos>=1) {r.op=2; close_t[i]=T;} else if(r.pos<=0) r.op=1; else if(r.pp==1){ r.op=2; close_t[i]=T;} else r.op=1; }
}
void phys(uint32_t dt){
  for(int i=0;i<2;i++){ Real&r=R[i];
    if(r.op==1){ r.pos+=dt*r.speed; if(r.pos>=1){r.pos=1;r.op=0;r.stop_t=T;} }
    if(r.op==2){ r.pos-=dt*r.speed; if(r.pos<=0){r.pos=0;r.op=0;r.stop_t=T;} }
    if(r.op==2 && ML[i]) violate("poort sluit tegen maglock");
    if(r.op==2 && close_t[i]==T && ML[i]) violate("sluit met maglock aan");
  }
  if(R[1].op!=0 && R[0].pos<0.35f) violate("poort 2 beweegt terwijl poort 1 < 35%");
}

Controller C;
void setup_hal(){
  C.hal.now=[](){return T;};
  C.hal.bus_alive=[](int i){return R[i].link;};
  C.hal.bus_pos=[](int i){ return std::round(R[i].pos*100)/100.0f; };
  C.hal.bus_op=[](int i){return R[i].op;};
  C.hal.bus_cmd=[](int i,Cmd c){ if(verbose)printf("  t=%u bus_cmd %d %d\n",T,i,c); if(R[i].link) real_cmd(i,c); };
  C.hal.pp_out=[](int i,bool on){ if(verbose)printf("  t=%u pp %d %d\n",T,i,on); if(on && !R[i].ppin) pp_edge(i); R[i].ppin=on; };
  C.hal.reed=[](int i){ return i==0 ? R[0].pos<=0.005f : R[1].pos>=0.995f; };
  C.hal.maglock_out=[](int i,bool on){ if(!on && ML[i]) ml_off_t[i]=T; ML[i]=on; if(verbose)printf("  t=%u maglock %d %d\n",T,i,on); };
  C.hal.save_pos=[](int i,float p){ saved[i]=p; };
  C.hal.log=[](const char*m){ logs.push_back(m); printf("  [%6.1fs] %s\n",T/1000.0,m); };
}
void run(uint32_t ms){ for(uint32_t t=0;t<ms;t+=100){ T+=100; phys(100); C.loop(); } }
void reset(float p1,float p2,bool link=true){
  C=Controller(); setup_hal(); T=1000; R[0]=Real(); R[1]=Real(); R[0].pos=p1; R[1].pos=p2; R[0].link=R[1].link=link; ML[0]=ML[1]=false;
  C.begin(saved[0],saved[1]); run(5000);
}
void st(const char*l){ printf("  -> %s: p1=%.2f(%s) p2=%.2f(%s) ml=%d%d alarm=%d mode=%s\n",l,R[0].pos,C.g[0].known?"k":"?",R[1].pos,C.g[1].known?"k":"?",ML[0],ML[1],C.alarm(),C.mode_text()); }
#define CHECK(c) do{ if(!(c)){ printf("  FAIL line %d: %s\n",__LINE__,#c); violations++; } }while(0)

int main(){
  // 1 open all
  puts("== 1 knop2 alles dicht -> open"); reset(0,0); C.knop2();
  float p1_when_p2_start=-1;
  for(int k=0;k<600;k++){ T+=100; phys(100); C.loop(); if(p1_when_p2_start<0 && R[1].op==1) p1_when_p2_start=R[0].pos; }
  st("na 60s"); printf("  p1 bij start p2: %.2f\n",p1_when_p2_start);
  CHECK(R[0].pos==1 && R[1].pos==1 && ML[0] && ML[1] && p1_when_p2_start>=0.49f);
  // 2 close all
  puts("== 2 knop2 alles open -> dicht"); C.knop2();
  float p2_when_p1_start=-1;
  for(int k=0;k<900;k++){ T+=100; phys(100); C.loop(); if(p2_when_p1_start<0 && R[0].op==2) p2_when_p1_start=R[1].pos; }
  st("na 90s"); printf("  p2 bij start p1: %.2f, release p2: %u ms\n",p2_when_p1_start, close_t[1]-ml_off_t[1]);
  CHECK(R[0].pos==0 && R[1].pos==0 && !ML[0] && !ML[1] && p2_when_p1_start<=0.06f && close_t[1]-ml_off_t[1]>=2000);
  // 3 voetganger
  puts("== 3 knop1 dicht -> voetganger"); C.knop1(); run(40000); st("na 40s");
  CHECK(R[0].pos>0.48f && R[0].pos<0.56f && R[1].pos==0 && !ML[0]);
  puts("== 3b knop2 bij voetganger -> alles open"); C.knop2(); run(60000); st("na 60s");
  CHECK(R[0].pos==1 && R[1].pos==1);
  puts("== 3c knop1 bij alles open -> poort 2 dicht, poort 1 blijft open"); C.knop1(); run(60000); st("na 60s");
  CHECK(R[0].pos==1 && R[1].pos==0 && ML[0]);
  puts("== 3d knop1 -> poort 1 dicht"); C.knop1(); run(60000); st("na 60s");
  CHECK(R[0].pos==0 && R[1].pos==0);
  // 4 p1 dicht, p2 open -> close all
  puts("== 4 p1 dicht p2 open -> knop2 (niet alles open) -> alles open"); reset(0,1); C.knop2(); run(60000); st("na 60s");
  CHECK(R[0].pos==1 && R[1].pos==1);
  puts("== 4b p1 dicht p2 open -> dubbel dicht"); reset(0,1); C.close_all(); run(90000); st("na 90s");
  CHECK(R[0].pos==0 && R[1].pos==0);
  puts("== 4c p1 dicht p2 open -> knop1 voetganger"); reset(0,1); C.knop1(); run(90000); st("na 90s");
  CHECK(R[1].pos==0 && R[0].pos>0.48f && R[0].pos<0.56f);
  puts("== 4d alleen poort 2 openen bij alles dicht"); reset(0,0); C.set_target(1,1); run(90000); st("na 90s");
  CHECK(R[1].pos==1 && R[0].pos>=0.5f);
  // 5 stop while moving
  puts("== 5 knop2 tijdens beweging -> stop"); reset(0,0); C.knop2(); run(8000); C.knop2(); run(5000); st("na stop");
  CHECK(R[0].op==0 && R[1].op==0 && !C.moving());
  // 6 bus loss idle -> reserve, open/close via PP
  puts("== 6 bus weg bij alles dicht -> reserve, knop2 open"); reset(0,0); R[0].link=R[1].link=false; run(2000); st("bus weg");
  C.knop2(); run(70000); st("na 70s");
  CHECK(R[0].pos==1 && R[1].pos==1 && ML[0] && ML[1]);
  puts("== 6b reserve knop2 dicht"); C.knop2(); run(100000); st("na 100s");
  CHECK(R[0].pos==0 && R[1].pos==0 && C.all_known());
  puts("== 6c reserve voetganger"); C.knop1(); run(40000); st("na 40s");
  CHECK(R[0].pos>0.4f && R[0].pos<0.65f && R[1].pos==0);
  puts("== 6d bus terug"); R[0].link=R[1].link=true; run(6000); st("bus terug");
  CHECK(C.g[0].bus && C.g[1].bus);
  // 7 bus loss mid movement
  puts("== 7 bus weg tijdens openen"); reset(0,0); C.knop2(); run(10000); R[0].link=R[1].link=false; run(70000); st("na 70s");
  CHECK(R[0].pos==1 && R[1].op==0 && C.all_known());
  // 8 boot zonder bus, saved midden -> onbekend
  puts("== 8 opstart zonder bus, NVS 50% -> onbekend"); saved[0]=0.5f; saved[1]=0; reset(0.5f,0,false); run(30000); st("na 35s");
  CHECK(!C.g[0].known && C.alarm()); C.knop2(); run(3000); CHECK(R[0].op==0 && R[1].op==0);
  puts("== 8b bevestigen p1 dicht? nee: zet echte stand naar dicht en bevestig alles dicht"); R[0].pos=0; C.confirm(0,0); run(1000); C.knop2(); run(70000); st("na 70s");
  CHECK(R[0].pos==1 && R[1].pos==1);
  // 9 boot zonder bus, saved dicht
  puts("== 9 opstart zonder bus, NVS dicht"); saved[0]=0; saved[1]=0; reset(0,0,false); run(30000); st("na 35s");
  CHECK(C.all_known() && !C.alarm());
  // 10 extern sluiten met maglock aan
  puts("== 10 alles open, extern sluitcommando (bus) -> noodlossing"); reset(1,1); run(2000); CHECK(ML[0]&&ML[1]);
  int v0=violations; real_cmd(1,1); run(3000); st("na extern"); 
  printf("  (1 verwachte overtreding bij extern sluiten: %d)\n", violations-v0); violations=v0+ ( (violations-v0)>1 ? (violations-v0-1):0 );
  CHECK(!ML[1]);
  // 11 obstakel: poort 1 stopt onverwacht
  puts("== 11 obstakel"); reset(0,0); C.knop2(); run(5000); R[0].op=0; run(5000); st("na obstakel");
  CHECK(C.alarm_cmd && !C.moving());
  // 12 poort reageert niet
  puts("== 12 poort reageert niet"); reset(0,0); R[0].speed=0; C.knop2(); run(12000); st("na 12s");
  CHECK(C.alarm_cmd);
  puts("== 12b motor negeert commando's"); reset(0,0); C.hal.bus_cmd=[](int,Cmd){}; C.knop2(); run(12000); st("na 12s");
  CHECK(C.alarm_cmd && !C.moving());
  setup_hal();
  puts("== 13 alles open, dubbel dicht: overlap"); reset(1,1); C.close_all();
  float p2s=-1; for(int k=0;k<900;k++){ T+=100; phys(100); C.loop(); if(p2s<0 && R[0].op==2) p2s=R[1].pos; } st("na 90s"); printf("  p2 bij start p1: %.2f\n",p2s);
  CHECK(R[0].pos==0 && R[1].pos==0 && p2s>0.0f && p2s<=0.06f);
  puts("== 14 positie 1 naar 30% bij p2 open (mag)"); reset(1,1); C.set_target(0,0.3f); run(40000); st("na 40s");
  CHECK(R[0].pos<0.33f && R[0].pos>0.27f && R[1].pos==1);
  puts("== 15 dan poort 2 dicht -> poort 1 eerst naar 43%"); C.set_target(1,0); run(60000); st("na 60s");
  CHECK(R[1].pos==0 && R[0].pos>=0.4f);
  puts("== 16 alleen bus poort 2 weg, alles open en dicht"); reset(0,0); R[1].link=false; run(2000); C.knop2(); run(70000); st("open");
  CHECK(R[0].pos==1 && R[1].pos==1); C.knop2(); run(100000); st("dicht"); CHECK(R[0].pos==0 && R[1].pos==0);
  puts("== 17 reserve: p1 dicht p2 open -> knop1"); saved[0]=0; saved[1]=1; reset(0,1,false); run(30000); C.knop1(); run(90000); st("na");
  CHECK(R[1].pos==0 && R[0].pos>0.4f && R[0].pos<0.65f);
  puts("== 18 reed klopt niet (bus): waarschuwing"); reset(0,0); C.hal.reed=[](int i){ return i==0 ? false : false; }; run(10000); st("na"); CHECK(C.alarm() && C.g[0].reed_alarm);
  setup_hal();
  puts("== 19 stop en direct omkeren: pauze na stilstand");
  reset(0,0); C.set_target(0,1); run(8000); C.stop_all(); run(1000); int ig0=ignored;
  uint32_t t0=T; C.set_target(0,0); uint32_t ts=0; for(int k=0;k<200;k++){ T+=100; phys(100); C.loop(); if(!ts && R[0].op==2) ts=T; }
  printf("  sluiten gestart na %u ms, genegeerd: %d\n", ts-t0, ignored-ig0);
  CHECK(ignored==ig0 && ts && ts-t0<=1000);
  puts("== 19b zelfde met pauze 0 (oude gedrag)");
  reset(0,0); C.set.pause_ms=0; C.set_target(0,1); run(8000); C.stop_all(); run(500); ig0=ignored;
  t0=T; C.set_target(0,0); ts=0; for(int k=0;k<200;k++){ T+=100; phys(100); C.loop(); if(!ts && R[0].op==2) ts=T; }
  printf("  sluiten gestart na %u ms, genegeerd: %d (verwacht: herhaling na 3 s)\n", ts-t0, ignored-ig0);
  ignored=ig0;

  // ---------- 20: sluiten met overlap 50 % (zoals bij Steve) ----------
  for (int variant=0; variant<3; variant++){
    float c1 = variant==0?0.50f:(variant==1?0.05f:0.60f);
    float sp2 = variant==2? 1.0f/40000 : 1.0f/34000;   // poort 2 trager dan poort 1
    printf("== 20.%d alles dicht, close1_below=%.0f%%, poort 2 traag\n",variant,c1*100);
    reset(1,1); R[1].speed=sp2; C.set.close1_below=c1; run(2000);
    C.close_all();
    int p2stops=0, p1reopen=0; float p1min_while_p2=2; int prev2=R[1].op, prev1=R[0].op; float p2_at_p1start=-1;
    for(int k=0;k<1500;k++){ T+=100; phys(100); C.loop();
      if(prev2==2 && R[1].op==0 && R[1].pos>0) p2stops++;
      if(R[0].op==1) p1reopen++;
      if(R[1].op!=0 && R[0].pos<p1min_while_p2) p1min_while_p2=R[0].pos;
      if(p2_at_p1start<0 && R[0].op==2) p2_at_p1start=R[1].pos;
      prev2=R[1].op; prev1=R[0].op; }
    st("na 150s"); printf("  p2 bij start p1: %.2f, stops poort 2: %d, poort 1 terug open: %d, min p1 terwijl p2 beweegt: %.2f\n",p2_at_p1start,p2stops,p1reopen,p1min_while_p2);
    CHECK(R[0].pos==0 && R[1].pos==0 && p2stops==0 && p1reopen==0 && p1min_while_p2>=0.40f && p2_at_p1start<=c1+0.02f);
  }
  // ---------- 21: handmatige P.P. ----------
  puts("== 21a P.P. poort 1 met bus, alles dicht -> poort 1 opent");
  reset(0,0); C.manual_pp(0); run(3000); st("na 3s"); CHECK(R[0].op==1);
  C.stop_all(); run(3000);
  puts("== 21b P.P. poort 2 met poort 1 dicht -> geweigerd");
  reset(0,0); C.manual_pp(1); run(3000); st("na 3s"); CHECK(R[1].op==0 && R[1].pos==0);
  puts("== 21c alles open (maglocks aan), P.P. poort 2 -> eerst maglock los, dan sluiten");
  reset(1,1); run(2000); CHECK(ML[1]); { int v0=violations; C.manual_pp(1); run(1000); CHECK(!ML[1] && R[1].op==0); run(2500); st("na 3,5s"); CHECK(R[1].op==2 && !ML[1]); run(30000); st("na 33s"); CHECK(R[1].pos==0 && !ML[1]); CHECK(violations==v0); }
  puts("== 21d reserve: P.P. poort 1 vanuit dicht -> schatting loopt mee");
  saved[0]=0; saved[1]=0; reset(0,0,false); run(30000); C.manual_pp(0); run(5000); st("na 5s");
  printf("  echt %.2f, schatting %.2f\n",R[0].pos,C.g[0].pos); CHECK(R[0].op==1 && std::fabs(R[0].pos-C.g[0].pos)<0.08f);
  setup_hal();
  printf("\nGENEGEERDE COMMANDO'S (pauze 1,5 s): %d, STOPs verstuurd: %d\n",ignored,stops_sent);
  printf("\nVIOLATIONS/FAILS: %d\n",violations);
  return violations?1:0;
}
