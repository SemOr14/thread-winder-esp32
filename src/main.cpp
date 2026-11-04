#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <atomic>
#include <soc/gpio_struct.h>
#include "motion_math.h"
#include "web_assets.h"

constexpr int AS=25,AD=26,AE=27,BS=18,BD=19,BE=23,LIMIT=32,START=33,HOME=21;
constexpr const char *SERVICE="7b4e0001-6b2d-4e53-9c71-9a03f30c8a10", *UUID_RX="7b4e0002-6b2d-4e53-9c71-9a03f30c8a10", *UUID_TX="7b4e0003-6b2d-4e53-9c71-9a03f30c8a10";
struct Config {
 double targetM=5,speedPct=25,maxRpm=120,accelRpmS=8,widthMm=12.8,homeOffsetMm=2,threadMm=.11,coreMm=4,pitchMm=.11,lengthFactor=1,leadMm=4;
 double homeSpeedMmS=2,homeAccelMmS2=2,homeTravelMm=300,homeBackoffMm=2,homeLatchMmS=.5;
 int microsteps=16,stepsPerRev=200;
 bool invertSpindle=false,invertTraverse=false,commissioned=false,limitNormallyOpen=true;
};
portMUX_TYPE pulseMux=portMUX_INITIALIZER_UNLOCKED;
struct Pulse {volatile int32_t a=0,b=0;volatile uint32_t total=0,done=0,na=0,nb=0,ea=0,eb=0,phase=0,rateMilli=0;int sa=1,sb=1;volatile bool active=false;} pulse;
void IRAM_ATTR onTimer(){
 portENTER_CRITICAL_ISR(&pulseMux);
 GPIO.out_w1tc=(1UL<<AS)|(1UL<<BS);
 if(pulse.active && pulse.done<pulse.total){
  pulse.phase+=pulse.rateMilli;
  if(pulse.phase>=25000000U){
   pulse.phase-=25000000U;
   pulse.ea+=pulse.na;pulse.eb+=pulse.nb;
   if(pulse.ea>=pulse.total){pulse.ea-=pulse.total;GPIO.out_w1ts=1UL<<AS;pulse.a+=pulse.sa;}
   if(pulse.eb>=pulse.total){pulse.eb-=pulse.total;GPIO.out_w1ts=1UL<<BS;pulse.b+=pulse.sb;}
   ++pulse.done;
   if(pulse.done==pulse.total){pulse.active=false;pulse.rateMilli=0;}
  }
 }
 portEXIT_CRITICAL_ISR(&pulseMux);
}
enum class State {Commissioning,Idle,Homing,Positioning,Running,Pausing,Paused,Stopping,Returning,Error};
const char* stateName(State s){static const char* names[]={"commissioning","idle","homing","positioning","running","pausing","paused","stopping","returning","error"};return names[(int)s];}
enum class Op {Start,Pause,Resume,Reset,Home,Config};
struct Command{Op op;Config cfg;uint32_t ticket=0;};
std::atomic<uint32_t> completedTicket{0};std::atomic<bool> acceptedCommand{false};
struct Snapshot{Config cfg;State state=State::Commissioning;bool homed=false,limitActive=false;double lengthM=0,positionMm=0;char error[120]={0};};
Snapshot snapshot;
SemaphoreHandle_t snapshotMutex,plannerGate;QueueHandle_t commands;
Config cfg;State state=State::Commissioning,pausedState=State::Running;
bool commandSucceeded=true;
void configJson(JsonObject o,const Config& c);
bool homed=false,returnAfterStop=false,clearAfterStop=false;
char errorText[120]={0};
double velocity=0;int32_t jobStartA=0;int64_t jobTargetSteps=0;bool segmentRight=true;
enum class HomePhase{Release,Seek,Backoff,Latch,Park}; HomePhase homePhase;
uint32_t homeStarted=0;double homeBudgetSeconds=0;
bool stableLimit=false,lastLimitRaw=false;uint32_t limitChanged=0;
bool limitIsActive(){bool high=digitalRead(LIMIT);return cfg.limitNormallyOpen?!high:high;}
bool homeLimit(){bool raw=limitIsActive();if(raw!=lastLimitRaw){lastLimitRaw=raw;limitChanged=millis();}if(millis()-limitChanged>=5)stableLimit=raw;return stableLimit;}
int32_t positionA(){portENTER_CRITICAL(&pulseMux);auto v=pulse.a;portEXIT_CRITICAL(&pulseMux);return v;}
int32_t positionB(){portENTER_CRITICAL(&pulseMux);auto v=pulse.b;portEXIT_CRITICAL(&pulseMux);return v;}
bool active(){portENTER_CRITICAL(&pulseMux);auto v=pulse.active;portEXIT_CRITICAL(&pulseMux);return v;}
uint32_t remaining(){portENTER_CRITICAL(&pulseMux);auto v=pulse.total-pulse.done;portEXIT_CRITICAL(&pulseMux);return v;}
void rate(double v){portENTER_CRITICAL(&pulseMux);pulse.rateMilli=(uint32_t)lround(fmax(0.,fmin(8000.,v))*1000);portEXIT_CRITICAL(&pulseMux);}
void freeze(){portENTER_CRITICAL(&pulseMux);pulse.active=false;pulse.rateMilli=0;pulse.phase=0;portEXIT_CRITICAL(&pulseMux);velocity=0;}
double spr(){return cfg.stepsPerRev*cfg.microsteps;}
double spmm(){return spr()/cfg.leadMm;}
void segment(int32_t a,int32_t b){
 freeze();
 digitalWrite(AD,(a<0)^cfg.invertSpindle);digitalWrite(BD,(b<0)^cfg.invertTraverse);
 // Direction setup exceeds A4988 200 ns; no pulse is scheduled before next planner tick.
 portENTER_CRITICAL(&pulseMux);
 pulse.na=abs(a);pulse.nb=abs(b);pulse.total=max(pulse.na,pulse.nb);pulse.done=0;pulse.ea=0;pulse.eb=0;
 pulse.sa=a<0?-1:1;pulse.sb=b<0?-1:1;pulse.active=pulse.total>0;
 portEXIT_CRITICAL(&pulseMux);
}
void fail(const char* msg){freeze();homed=false;state=State::Error;strlcpy(errorText,msg,sizeof(errorText));}
void beginHome(){
 homed=false;homeStarted=millis();homeBudgetSeconds=cfg.homeTravelMm/cfg.homeLatchMmS+60;
 homePhase=limitIsActive()?HomePhase::Release:HomePhase::Seek;
 state=State::Homing;
 segment(0,lround((homePhase==HomePhase::Release?cfg.homeBackoffMm:-cfg.homeTravelMm)*spmm()));
}
void nextWindingSegment(){
 int64_t left=jobTargetSteps-(int64_t(positionA())-jobStartA);
 if(left<=0){beginHome();return;}
 int32_t fullA=lround(cfg.widthMm/cfg.pitchMm*spr());
 int32_t da=(int32_t)min(left,(int64_t)fullA);
 int32_t fullB=lround(cfg.widthMm*spmm());
 int32_t db=da==fullA?fullB:lround(double(fullB)*da/fullA);
 segment(da,segmentRight?db:-db);state=State::Running;
}
void publish(){
 Snapshot next;next.cfg=cfg;next.state=state;next.homed=homed;next.limitActive=limitIsActive();next.positionMm=positionB()/spmm();
 next.lengthM=windingLength(fmax(0.,double(positionA()-jobStartA)/spr()),cfg.coreMm,cfg.threadMm,cfg.pitchMm,cfg.widthMm,cfg.lengthFactor);
 strlcpy(next.error,errorText,sizeof(next.error));
 xSemaphoreTake(snapshotMutex,portMAX_DELAY);snapshot=next;xSemaphoreGive(snapshotMutex);
}
void process(const Command& c){
 if(c.op==Op::Config){
  if(state!=State::Idle && state!=State::Commissioning && state!=State::Error){return;}
  freeze();JsonDocument stored;stored["schema"]=2;configJson(stored["config"].to<JsonObject>(),c.cfg);String data;serializeJson(stored,data);
  Preferences p;bool ok=p.begin("winder",false);if(ok){ok=p.putString("config",data)==data.length();ok=ok&&p.getString("config","")==data;p.end();}
  if(!ok){commandSucceeded=false;strlcpy(errorText,"Не удалось сохранить настройки в памяти ESP32",sizeof(errorText));return;}
  bool referenceValid=homed&&cfg.commissioned&&c.cfg.commissioned&&cfg.microsteps==c.cfg.microsteps&&cfg.stepsPerRev==c.cfg.stepsPerRev&&cfg.leadMm==c.cfg.leadMm&&cfg.invertTraverse==c.cfg.invertTraverse&&cfg.limitNormallyOpen==c.cfg.limitNormallyOpen;
  cfg=c.cfg;homed=referenceValid;state=cfg.commissioned?State::Idle:State::Commissioning;errorText[0]=0;
  digitalWrite(AE,!cfg.commissioned);digitalWrite(BE,!cfg.commissioned);return;
 }
 if(!cfg.commissioned){strlcpy(errorText,"Сначала завершите проверку подключения",sizeof(errorText));return;}
 if(c.op==Op::Reset || c.op==Op::Home){
  if(state==State::Running || state==State::Positioning)pausedState=state;
  returnAfterStop=c.op==Op::Home;clearAfterStop=c.op==Op::Reset;
  if(state==State::Homing){freeze();homed=false;}
  state=State::Stopping;return;
 }
 if(c.op==Op::Pause && (state==State::Running || state==State::Positioning)){pausedState=state;state=State::Pausing;return;}
 if(c.op==Op::Resume || (c.op==Op::Start && state==State::Paused)){
  if(state==State::Paused){state=pausedState;}return;
 }
 if(c.op==Op::Start && state==State::Idle){
  if(!homed){strlcpy(errorText,"Сначала выполните возврат к концевику",sizeof(errorText));return;}
  freeze();portENTER_CRITICAL(&pulseMux);pulse.a=0;portEXIT_CRITICAL(&pulseMux);
  jobStartA=0;jobTargetSteps=llround(windingTurns(cfg.targetM,cfg.coreMm,cfg.threadMm,cfg.pitchMm,cfg.widthMm,cfg.lengthFactor)*spr());segmentRight=true;
  errorText[0]=0;segment(0,lround(cfg.homeOffsetMm*spmm())-positionB());state=State::Positioning;
 }
}
const char* rejectReason(const Command& c){
 if(c.op==Op::Config)return state==State::Idle||state==State::Commissioning||state==State::Error?nullptr:"Настройки доступны после остановки";
 if(!cfg.commissioned)return "Сначала завершите проверку подключения";
 if(c.op==Op::Home || c.op==Op::Reset)return nullptr;
 if(c.op==Op::Pause)return state==State::Running||state==State::Positioning?nullptr:"Сейчас нельзя поставить на паузу";
 if(c.op==Op::Resume)return state==State::Paused?nullptr:"Станок не на паузе";
 if(c.op==Op::Start && state==State::Paused)return nullptr;
 if(c.op==Op::Start && state==State::Idle)return homed?nullptr:"Сначала выполните возврат к концевику";
 return "Команда недоступна в текущем состоянии";
}
void planner(void*){
 if(cfg.commissioned){digitalWrite(AE,LOW);digitalWrite(BE,LOW);beginHome();}
 TickType_t last=xTaskGetTickCount();
 for(;;){
  xSemaphoreTake(plannerGate,portMAX_DELAY);
  Command cmd;while(xQueueReceive(commands,&cmd,0)==pdTRUE){commandSucceeded=true;const char* why=rejectReason(cmd);if(why)strlcpy(errorText,why,sizeof(errorText));else process(cmd);if(cmd.ticket){publish();acceptedCommand.store(!why&&commandSucceeded);completedTicket.store(cmd.ticket);}}
  if(state==State::Homing){
   if((millis()-homeStarted)>homeBudgetSeconds*1000){fail("Истекло время поиска концевика");}
   else {
    bool hit=homeLimit();
    if(homePhase==HomePhase::Release && !active()){
     if(hit)fail("Концевик не отпущен: проверьте проводку");else {homePhase=HomePhase::Seek;segment(0,-lround(cfg.homeTravelMm*spmm()));}
    }else if(homePhase==HomePhase::Seek && hit){freeze();homePhase=HomePhase::Backoff;segment(0,lround(cfg.homeBackoffMm*spmm()));}
    else if(homePhase==HomePhase::Backoff && !active()){
     if(hit)fail("Концевик не отпущен после отхода");else{homePhase=HomePhase::Latch;segment(0,-lround(2*cfg.homeBackoffMm*spmm()));}
    }else if(homePhase==HomePhase::Latch && hit){freeze();portENTER_CRITICAL(&pulseMux);pulse.b=0;portEXIT_CRITICAL(&pulseMux);homePhase=HomePhase::Park;segment(0,lround(cfg.homeBackoffMm*spmm()));}
    else if(homePhase==HomePhase::Park && !active()){if(hit)fail("Концевик не отпущен после поиска нуля");else{homed=true;state=State::Idle;errorText[0]=0;}}
    else if(!active() && (homePhase==HomePhase::Seek || homePhase==HomePhase::Latch))fail("Концевик не найден в пределах хода");
    if(state==State::Homing){double vmax=(homePhase==HomePhase::Latch?cfg.homeLatchMmS:cfg.homeSpeedMmS)*spmm();velocity=nextRate(velocity,vmax,cfg.homeAccelMmS2*spmm(),remaining(),.001);if(remaining()<=2)velocity=fmin(vmax,fmax(velocity,5.));rate(velocity);}
   }
  }else if(state==State::Pausing || state==State::Stopping){
   if(active()&&limitIsActive()){fail("Концевик сработал при остановке");publish();xSemaphoreGive(plannerGate);vTaskDelay(pdMS_TO_TICKS(1));continue;}
   double accel=(pausedState==State::Running?cfg.accelRpmS*spr()/60:cfg.homeAccelMmS2*spmm());
   velocity=fmax(0.,velocity-accel*.001);rate(velocity);
   if(velocity<=.01 || !active()){
    rate(0);velocity=0;
    if(state==State::Pausing)state=State::Paused;
    else {freeze();if(clearAfterStop){jobStartA=positionA();jobTargetSteps=0;clearAfterStop=false;}if(returnAfterStop){returnAfterStop=false;beginHome();}else state=State::Idle;}
   }
  }else if(state==State::Running || state==State::Positioning){
   if(limitIsActive()){fail("Концевик сработал вне поиска исходной позиции");}
   else if(!active()){
    velocity=0;
    if(state==State::Positioning)nextWindingSegment();else {segmentRight=!segmentRight;nextWindingSegment();}
   }else {
    bool wind=state==State::Running;double vmax=wind?cfg.maxRpm*cfg.speedPct/100*spr()/60:cfg.homeSpeedMmS*spmm();
    double acc=wind?cfg.accelRpmS*spr()/60:cfg.homeAccelMmS2*spmm();
    velocity=nextRate(velocity,vmax,acc,remaining(),.001);
    // Two-step exact-end crawl avoids floating-point stopping deadlock.
    if(remaining()<=2)velocity=fmin(vmax,fmax(velocity,5.));
    rate(velocity);
   }
  }
  publish();xSemaphoreGive(plannerGate);vTaskDelay(pdMS_TO_TICKS(1));
 }
}
Snapshot readSnapshot(){xSemaphoreTake(snapshotMutex,portMAX_DELAY);auto s=snapshot;xSemaphoreGive(snapshotMutex);return s;}
void configJson(JsonObject o,const Config& c){
#define OUT(k) o[#k]=c.k
 OUT(targetM);OUT(speedPct);OUT(maxRpm);OUT(accelRpmS);OUT(widthMm);OUT(homeOffsetMm);OUT(threadMm);OUT(coreMm);OUT(pitchMm);OUT(lengthFactor);OUT(leadMm);OUT(homeSpeedMmS);OUT(homeAccelMmS2);OUT(homeTravelMm);OUT(homeBackoffMm);OUT(homeLatchMmS);OUT(microsteps);OUT(stepsPerRev);OUT(invertSpindle);OUT(invertTraverse);OUT(commissioned);OUT(limitNormallyOpen);
#undef OUT
}
bool parseConfig(JsonObjectConst o,Config& c){
 JsonDocument known;configJson(known.to<JsonObject>(),Config{});for(JsonPairConst kv:o)if(!known.containsKey(kv.key()))return false;
#define NUM(k,lo,hi) if(o.containsKey(#k)){if(!o[#k].is<double>())return false;double v=o[#k].as<double>();if(!isfinite(v)||v<lo||v>hi)return false;c.k=v;}
 NUM(targetM,1,40);NUM(speedPct,1,100);NUM(maxRpm,1,150);NUM(accelRpmS,.1,30);NUM(widthMm,1,50);NUM(homeOffsetMm,.5,100);NUM(threadMm,.02,1);NUM(coreMm,1,30);NUM(pitchMm,.02,1);NUM(lengthFactor,.5,2);NUM(leadMm,1,20);NUM(homeSpeedMmS,.1,5);NUM(homeAccelMmS2,.1,10);NUM(homeTravelMm,5,500);NUM(homeBackoffMm,.5,10);NUM(homeLatchMmS,.1,1);
#undef NUM
 for(auto key:{"invertSpindle","invertTraverse","commissioned","limitNormallyOpen"})if(o.containsKey(key)&&!o[key].is<bool>())return false;
 if(o.containsKey("invertSpindle"))c.invertSpindle=o["invertSpindle"];
 if(o.containsKey("invertTraverse"))c.invertTraverse=o["invertTraverse"];
 if(o.containsKey("commissioned"))c.commissioned=o["commissioned"];
 if(o.containsKey("limitNormallyOpen"))c.limitNormallyOpen=o["limitNormallyOpen"];
 if(o.containsKey("microsteps")){int n=o["microsteps"]|0;if(n!=1&&n!=2&&n!=4&&n!=8&&n!=16)return false;c.microsteps=n;}
 if(o.containsKey("stepsPerRev")){int n=o["stepsPerRev"]|0;if(n!=200&&n!=400)return false;c.stepsPerRev=n;}
 if(c.maxRpm*c.microsteps*c.stepsPerRev/60>8000 || c.homeSpeedMmS*c.microsteps*c.stepsPerRev/c.leadMm>8000)return false;
 if(c.homeOffsetMm+c.widthMm+c.homeBackoffMm>=c.homeTravelMm || c.homeLatchMmS>c.homeSpeedMmS || c.homeBackoffMm*2>=c.homeTravelMm)return false;
 return true;
}
bool bleConnected=false;WebServer server(80);NimBLECharacteristic* tx=nullptr;
String statusJson(){
 auto s=readSnapshot();JsonDocument doc;doc["state"]=stateName(s.state);doc["homed"]=s.homed;doc["lengthM"]=s.lengthM;doc["targetM"]=s.cfg.targetM;doc["speedPct"]=s.cfg.speedPct;doc["positionMm"]=s.positionMm;doc["limitActive"]=s.limitActive;doc["limit"]["rawHigh"]=bool(digitalRead(LIMIT));doc["limit"]["active"]=s.limitActive;doc["limit"]["normallyOpen"]=s.cfg.limitNormallyOpen;doc["error"]=s.error;doc["bleConnected"]=bleConnected;doc["version"]="0.3.0";configJson(doc["config"].to<JsonObject>(),s.cfg);
 String out;serializeJson(doc,out);return out;
}
String commandJson(const String& text){
 JsonDocument doc;JsonDocument reply;
 auto respond=[&](bool ok,const char* msg=nullptr){JsonDocument out;if(ok)deserializeJson(out,statusJson());out["ok"]=ok;if(!doc["id"].isNull())out["id"]=doc["id"];if(msg)out["error"]=msg;String result;serializeJson(out,result);return result;};
 if(text.length()>2048 || deserializeJson(doc,text)){return respond(false,"Некорректный JSON");}
 String name=doc["cmd"]|"";Command c;auto s=readSnapshot();
 if(name=="get")return respond(true);
 if(name=="start")c.op=Op::Start;else if(name=="pause")c.op=Op::Pause;else if(name=="resume")c.op=Op::Resume;else if(name=="reset")c.op=Op::Reset;else if(name=="home")c.op=Op::Home;
 else if(name=="config"){
  if(s.state!=State::Idle && s.state!=State::Commissioning && s.state!=State::Error)return respond(false,"Настройки доступны после остановки");
  c.cfg=s.cfg;if(!doc["config"].is<JsonObject>() || !parseConfig(doc["config"],c.cfg))return respond(false,"Недопустимые настройки");c.op=Op::Config;
 }else return respond(false,"Неизвестная команда");
 static uint32_t seq=0;c.ticket=++seq;
 if(xQueueSend(commands,&c,0)!=pdTRUE)return respond(false,"Очередь команд заполнена");
 uint32_t start=millis();while(completedTicket.load()!=c.ticket && millis()-start<500)delay(1);
 if(completedTicket.load()!=c.ticket)return respond(false,"Нет ответа планировщика движения");
 if(!acceptedCommand.load()){auto result=readSnapshot();return respond(false,result.error);}
 return respond(true);
}
QueueHandle_t incoming;struct Line{char data[2049];};
class Callbacks:public NimBLECharacteristicCallbacks {
 String buffer;bool discard=false;
 void onWrite(NimBLECharacteristic* ch) override {
  auto value=ch->getValue();for(char c:value){if(c=='\n'){if(!discard && buffer.length()){Line line;strlcpy(line.data,buffer.c_str(),sizeof(line.data));xQueueSend(incoming,&line,0);}buffer="";discard=false;}else if(c!='\r'&&!discard){if(buffer.length()>=2048){buffer="";discard=true;}else buffer+=c;}}
 }
};
class ServerCallbacks:public NimBLEServerCallbacks{
 void onConnect(NimBLEServer*) override {bleConnected=true;}
 void onDisconnect(NimBLEServer*) override {bleConnected=false;NimBLEDevice::startAdvertising();}
};
void sendBle(const String& msg){if(!bleConnected || !tx)return;String line=msg+"\n";for(size_t i=0;i<line.length();i+=20){tx->setValue((uint8_t*)line.c_str()+i,min((size_t)20,line.length()-i));tx->notify();delay(3);}}
struct Button {int pin;bool raw=true,stable=true,longDone=false;uint32_t changed=0,down=0;Button(int p):pin(p){}};Button startButton{START},homeButton{HOME};
void enqueueButton(Command c){if(c.op==Op::Home||c.op==Op::Reset){while(xQueueSendToFront(commands,&c,pdMS_TO_TICKS(50))!=pdTRUE)vTaskDelay(1);}else xQueueSend(commands,&c,0);}
void buttonTick(Button& b){
 bool raw=digitalRead(b.pin);uint32_t now=millis();
 if(raw!=b.raw){b.raw=raw;b.changed=now;}
 if(now-b.changed>=30 && b.stable!=raw){
  b.stable=raw;
  if(!raw){b.down=now;b.longDone=false;}
  else {
   if(!b.longDone||b.pin==HOME){Command c;auto s=readSnapshot();c.op=b.pin==HOME?Op::Home:(s.state==State::Running||s.state==State::Positioning)?Op::Pause:Op::Start;enqueueButton(c);}
  }
 }
 if(b.pin==START&&!b.stable&&!b.longDone&&now-b.down>=1800){b.longDone=true;Command c;c.op=Op::Reset;enqueueButton(c);}
}
void buttonsTask(void*){for(;;){buttonTick(startButton);buttonTick(homeButton);vTaskDelay(pdMS_TO_TICKS(5));}}
void serialInputs(){
 static String line;static bool discard=false;
 while(Serial.available()){char c=Serial.read();if(c=='\n'){
   if(!discard&&line.length()){JsonDocument request;if(deserializeJson(request,line))Serial.println("{\"ok\":false,\"error\":\"Invalid JSON\"}");else{String cmd=request["cmd"]|"";if(cmd=="get")Serial.println(commandJson(line));else Serial.println("{\"ok\":false,\"error\":\"USB allows get only\"}");}}
   line="";discard=false;
  }else if(c!='\r'&&!discard){if(line.length()>=2048){line="";discard=true;}else line+=c;}}
}
void setup(){
 pinMode(AS,OUTPUT);pinMode(AD,OUTPUT);pinMode(AE,OUTPUT);pinMode(BS,OUTPUT);pinMode(BD,OUTPUT);pinMode(BE,OUTPUT);digitalWrite(AE,HIGH);digitalWrite(BE,HIGH);
 pinMode(LIMIT,INPUT_PULLUP);pinMode(START,INPUT_PULLUP);pinMode(HOME,INPUT_PULLUP);Serial.begin(115200);
 Preferences prefs;prefs.begin("winder",true);String saved=prefs.getString("config","");prefs.end();if(saved.length()){JsonDocument doc;if(!deserializeJson(doc,saved)&&doc["config"].is<JsonObject>()){bool legacy=doc["schema"]==1&&doc["config"].size()==21;bool current=doc["schema"]==2&&doc["config"].size()==22;if(legacy||current){Config loaded;if(legacy)loaded.limitNormallyOpen=false;if(parseConfig(doc["config"],loaded))cfg=loaded;}}}
 state=cfg.commissioned?State::Idle:State::Commissioning;snapshotMutex=xSemaphoreCreateMutex();plannerGate=xSemaphoreCreateMutex();commands=xQueueCreate(6,sizeof(Command));incoming=xQueueCreate(4,sizeof(Line));publish();
 Serial.println(statusJson());
 auto timer=timerBegin(0,80,true);timerAttachInterrupt(timer,&onTimer,true);timerAlarmWrite(timer,40,true);timerAlarmEnable(timer);
 xTaskCreatePinnedToCore(planner,"motion",6144,nullptr,4,nullptr,1);
 xTaskCreatePinnedToCore(buttonsTask,"buttons",3072,nullptr,2,nullptr,0);
 WiFi.mode(WIFI_AP);WiFi.softAP("ThreadWinder","winding12345");
 server.on("/",HTTP_GET,[]{server.send_P(200,webAssets[0].mime,webAssets[0].data,webAssets[0].size);});
 for(const auto& asset:webAssets){const auto* a=&asset;server.on(a->path,HTTP_GET,[a]{server.send_P(200,a->mime,a->data,a->size);});}
 server.on("/api/status",HTTP_GET,[]{server.send(200,"application/json; charset=utf-8",statusJson());});
 server.on("/api/command",HTTP_POST,[]{server.send(200,"application/json; charset=utf-8",commandJson(server.arg("plain")));});server.begin();
 NimBLEDevice::init("ThreadWinder");auto srv=NimBLEDevice::createServer();srv->setCallbacks(new ServerCallbacks());auto svc=srv->createService(SERVICE);
 auto rx=svc->createCharacteristic(UUID_RX,NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::WRITE_NR);rx->setCallbacks(new Callbacks());tx=svc->createCharacteristic(UUID_TX,NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);svc->start();auto adv=NimBLEDevice::getAdvertising();adv->addServiceUUID(SERVICE);adv->start();Serial.println("ThreadWinder AP: http://192.168.4.1");
}
void loop(){
 serialInputs();
 server.handleClient();Line line;while(xQueueReceive(incoming,&line,0)==pdTRUE)sendBle(commandJson(line.data));
 static uint32_t sent=0;
 if(millis()-sent>=1000){sent=millis();sendBle(statusJson());}
 delay(2);
}
