// Carrot B1, ESP32-S3. 500 kbit/s Classic CAN; commands never contain raw CAN.
#include <Arduino.h>
#include <BLEDevice.h>
#include <Preferences.h>
#include <driver/twai.h>
#include <mbedtls/md.h>
#include <atomic>
#include "B1Tap.h"
#include "B1Ota.h"

constexpr char SERVICE[] = "db340a90-804c-4ead-af37-8a0e6af754a1";
constexpr char STATUS_UUID[] = "db340a91-804c-4ead-af37-8a0e6af754a1";
constexpr char META[] = "db340a92-804c-4ead-af37-8a0e6af754a1";
constexpr char CONTROL[] = "db340a93-804c-4ead-af37-8a0e6af754a1";
enum State : uint8_t { Missing, Ready, Sending, Waiting, Override, Fault };
enum Op : uint8_t { Lease, Left, Right, Stop, HoldLeft, HoldRight };
struct __attribute__((packed)) Request {
  uint32_t session, stamp;
  uint16_t seq;
  uint8_t op, ttl5ms, tag[8];
};
struct __attribute__((packed)) Report {
  uint32_t session, stamp;
  uint16_t ack;
  uint8_t state, lamps, version;
  uint16_t lastSeq;
};
static_assert(sizeof(Request) == 20 && sizeof(Report) == 15);
struct Sample {
  uint8_t data[8] = {};
  uint32_t stamp = 0, first = 0;
  uint8_t count = 0;
  bool fresh(uint32_t now) const { return count && now - stamp < 400; }
  void set(const uint8_t *src, uint32_t now) {
    if (!fresh(now)) first=now;
    count = fresh(now) ? min(10, int(count) + 1) : 1;
    stamp = now;
    memcpy(data, src, 8);
  }
};
Sample stalk, feedback;
Preferences prefs;
struct Counter { uint8_t original[8], last; };
Counter saved;
uint8_t key[32];
BLECharacteristic *statusValue, *metaValue;
QueueHandle_t inbox;
std::atomic<uint32_t> session{0};
uint32_t currentSession, deadline, startUs, errorAt, overrideAt;
uint16_t lastSeq, ack, activeSeq;
bool leased, active, installed, online, inFlight, haveSaved, owned;
uint32_t ownedAt, darkAt;
bool sawLamp;
Op ownedDirection;
uint8_t original[8];
b1::TapFrame frames[7];
int frameCount, nextFrame;

void led(uint8_t r,uint8_t g,uint8_t b) {
  static uint32_t last = UINT32_MAX;
  uint32_t rgb=(uint32_t(r)<<16)|(uint32_t(g)<<8)|b;
  if (rgb==last) return;
  rgbLedWrite(48,r,g,b); rgbLedWrite(38,r,g,b); last=rgb;
}

bool mode(bool tx) {
  if (installed) {
    twai_stop();
    if (twai_driver_uninstall()!=ESP_OK) return online=false;
    installed=false;
  }
  twai_general_config_t cfg=TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4,GPIO_NUM_5,tx?TWAI_MODE_NORMAL:TWAI_MODE_LISTEN_ONLY);
  cfg.tx_queue_len=0; cfg.rx_queue_len=128;
  cfg.alerts_enabled=TWAI_ALERT_TX_SUCCESS|TWAI_ALERT_TX_FAILED|TWAI_ALERT_BUS_ERROR|TWAI_ALERT_BUS_OFF|TWAI_ALERT_RX_QUEUE_FULL|TWAI_ALERT_ARB_LOST;
  twai_timing_config_t timing=TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter=TWAI_FILTER_CONFIG_ACCEPT_ALL();
  installed=twai_driver_install(&cfg,&timing,&filter)==ESP_OK;
  return online=installed && twai_start()==ESP_OK;
}

uint8_t lamps() {
  const auto *d=feedback.data;
  return (d[7]&64) ? (!!(d[7]&8) | (!!(d[7]&32)<<1)) : (!!(d[2]&16) | (!!(d[2]&64)<<1));
}
bool matched(uint32_t now) {
  return online && stalk.fresh(now) && feedback.fresh(now) && stalk.count>=3 && feedback.count>=3 && now-stalk.first>=200 && now-feedback.first>=100;
}
bool neutral(uint32_t now) {
  return matched(now) && !(stalk.data[3]&0x40) && !(stalk.data[4]&0x51) && !(feedback.data[1]&5) && !lamps();
}
bool guard(uint32_t now) {
  return leased && currentSession && session.load()==currentSession && int32_t(deadline-now)>0;
}
void finish(bool success) {
  if (active) {
    active=inFlight=false;
    if (!mode(false)) success=false;
  }
  if (success) ack=activeSeq;
  else { ack=0; owned=false; }
}
void startTap(const Request &req) {
  uint32_t now=millis();
  if (active || !neutral(now) || !guard(now)) return;
  memcpy(original,stalk.data,8);
  unsigned counter=haveSaved && !memcmp(saved.original,original,8) ? saved.last : original[1]>>4;
  frameCount=b1::encode(original,req.op==Left?b1::Direction::Left:b1::Direction::Right,counter,frames);
  if (!frameCount) return;
  memcpy(saved.original,original,8); saved.last=frames[frameCount-1].data[1]>>4;
  if (prefs.putBytes("counter",&saved,sizeof(saved))!=sizeof(saved)) { errorAt=now; return; }
  haveSaved=true;
  active=true; owned=true; ownedAt=now; activeSeq=req.seq; ack=0;
  ownedDirection=Op(req.op); sawLamp=false; darkAt=0;
  nextFrame=0; inFlight=false;
  if (!mode(true) || !guard(millis())) { finish(false); errorAt=now; return; }
  startUs=micros();
}
void receiveCan() {
  twai_message_t f;
  for (unsigned i=0; online && i<128 && twai_receive(&f,0)==ESP_OK; ++i) {
    ota::lastBus=millis();
    if (f.extd || (f.identifier!=0x3c1 && f.identifier!=0x413)) continue;
    uint32_t now=millis();
    Sample &s=f.identifier==0x3c1 ? stalk : feedback;
    if (f.rtr || f.data_length_code!=8 || (&s==&stalk && (b1::crc(f.data)!=f.data[0] || f.data[1]>>4==15))) {
      s.count=0;
    } else s.set(f.data,now);
    if (owned && &s==&feedback && s.count) {
      uint8_t expected=ownedDirection==Left ? 1 : 2;
      if ((lamps() & ~expected) || (s.data[1]&5)) { finish(false); overrideAt=now; }
      else if (lamps()==expected) { sawLamp=true; darkAt=0; }
      else if (sawLamp && !darkAt) darkAt=now;
    }
    if (owned && (!s.count || (&s==&stalk && memcmp(original,s.data,8)))) {
      finish(false); overrideAt=now;
    }
  }
}
class Link : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { session.store(esp_random()|1U); }
  void onDisconnect(BLEServer *s) override { session.store(0); s->startAdvertising(); }
};
class Writer : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String bytes=c->getValue();
    if (bytes.length()!=sizeof(Request)) return;
    Request r; memcpy(&r,bytes.c_str(),sizeof(r));
    uint8_t digest[32], diff=0;
    if (mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),key,sizeof(key),
                        (uint8_t*)&r,offsetof(Request,tag),digest)) return;
    for (unsigned i=0;i<sizeof(r.tag);++i) diff |= r.tag[i]^digest[i];
    if (!diff && r.session && r.session==session.load()) xQueueSend(inbox,&r,0);
  }
};
void setup() {
  Serial.begin(115200); led(22,9,0);
  inbox=xQueueCreate(8,sizeof(Request));
  if (!inbox || !prefs.begin("carrot-b1",false)) { led(40,0,0); while(true) delay(1000); }
  BLEDevice::init("Carrot-B1");
  BLEDevice::setMTU(517);
  if (prefs.getBytesLength("auth")!=sizeof(key)) {
    esp_fill_random(key,sizeof(key));
    if (prefs.putBytes("auth",key,sizeof(key))!=sizeof(key)) { led(40,0,0); while(true) delay(1000); }
  }
  if (prefs.getBytes("auth",key,sizeof(key))!=sizeof(key)) { led(40,0,0); while(true) delay(1000); }
  haveSaved=prefs.getBytesLength("counter")==sizeof(saved) && prefs.getBytes("counter",&saved,sizeof(saved))==sizeof(saved);
  mode(false);
  auto *server=BLEDevice::createServer(); server->setCallbacks(new Link());
  auto *svc=server->createService(SERVICE);
  statusValue=svc->createCharacteristic(STATUS_UUID,BLECharacteristic::PROPERTY_READ);
  metaValue=svc->createCharacteristic(META,BLECharacteristic::PROPERTY_READ);
  svc->createCharacteristic(CONTROL,BLECharacteristic::PROPERTY_WRITE)->setCallbacks(new Writer());
  ota::setup(svc);
  svc->start(); BLEDevice::getAdvertising()->addServiceUUID(SERVICE);
  BLEDevice::getAdvertising()->setScanResponse(true); BLEDevice::startAdvertising();
  Serial.printf("Carrot-B1 control v2 %s 500k\n",BLEDevice::getAddress().toString().c_str());
}
void loop() {
  uint32_t now=millis();
  uint32_t sid=session.load();
  if (currentSession!=sid) {
    finish(false); leased=false; lastSeq=0; currentSession=sid;
  }
  receiveCan();
  uint32_t alerts=0;
  if (online && twai_read_alerts(&alerts,0)==ESP_OK) {
    if (alerts & ~TWAI_ALERT_TX_SUCCESS) {
      finish(false); stalk.count=feedback.count=0; errorAt=now;
    } else if (active && inFlight && (alerts&TWAI_ALERT_TX_SUCCESS)) {
      inFlight=false;
      if (nextFrame==frameCount) finish(true);
    }
  }
  ota::tick(now,sid,active,key);
  Request r;
  while (xQueueReceive(inbox,&r,0)==pdTRUE) {
    if (ota::busy || ota::reboot || !sid || r.session!=sid || int16_t(r.seq-lastSeq)<=0 || !r.ttl5ms || r.ttl5ms>70 ||
        now-r.stamp>=uint32_t(r.ttl5ms)*5 || r.op>HoldRight) continue;
    lastSeq=r.seq;
    if (r.op==Stop) { leased=false; finish(false); continue; }
    leased=true; deadline=r.stamp+r.ttl5ms*5;
    if (r.op==Left || r.op==Right) startTap(r);
    bool maintain=(r.op==HoldLeft && ownedDirection==Left) || (r.op==HoldRight && ownedDirection==Right);
    if (maintain && owned && !active && ack && darkAt && now-darkAt>=1200 && now-ownedAt<300000 && neutral(now)) {
      // Repeat only after confirmed physical activity and a full dark interval.
      uint32_t first=ownedAt;
      Request repeat=r; repeat.op=ownedDirection; repeat.seq=ack;
      startTap(repeat); ownedAt=first;
    }
  }
  if (owned && (!guard(now) || !matched(now) || now-ownedAt>300000)) finish(false);
  if (active) {
    uint32_t elapsed=micros()-startUs;
    if (!guard(millis()) || (inFlight && elapsed-frames[nextFrame-1].offsetUs>40000)) { finish(false); errorAt=now; }
    else if (!inFlight && nextFrame<frameCount && elapsed>=frames[nextFrame].offsetUs) {
      twai_message_t f={}; f.identifier=0x3c1; f.data_length_code=8; f.ss=1;
      memcpy(f.data,frames[nextFrame].data,8);
      if (elapsed-frames[nextFrame].offsetUs>=50000 || twai_transmit(&f,0)!=ESP_OK) { finish(false); errorAt=now; }
      else { ++nextFrame; inFlight=true; }
    }
  }
  static uint32_t published=0;
  if (now-published>=20) {
    published=now;
    State state=!online || (errorAt && now-errorAt<500) ? Fault : overrideAt && now-overrideAt<400 ? Override :
      !matched(now) ? Missing : active ? Sending : neutral(now) ? Ready : Waiting;
    Report report={sid,now,ack,state,lamps(),2,lastSeq};
    metaValue->setValue((uint8_t*)&report,sizeof(report));
    char info[24]; snprintf(info,sizeof(info),"3C1=%c 413=%c L=%c R=%c",stalk.fresh(now)?'1':'-',feedback.fresh(now)?'1':'-',
      feedback.fresh(now)?'0'+(lamps()&1):'-',feedback.fresh(now)?'0'+(lamps()>>1):'-');
    statusValue->setValue(info);
    if (ota::busy || ota::reboot) led(0,24,32);
    else if (state==Fault) led(40,0,0);
    else if (state==Sending) led(0,32,24);
    else if (matched(now) && guard(now)) led(0,30,0);
    else if (matched(now)) { unsigned phase=now%2400; led(0,0,2+min(phase,2400-phase)*34/1200); }
    else if (sid) led(17,0,20); else led(22,9,0);
  }
  // Enrollment key is available only by explicit request over the physical UART.
  static char cmd[12]; static unsigned pos=0;
  while (Serial.available()) {
    char ch=Serial.read();
    if (ch=='\n') {
      cmd[pos]=0; pos=0;
      if (!strcmp(cmd,"PAIR")) { Serial.print("PAIR "); for(uint8_t b:key) Serial.printf("%02x",b); Serial.println(); }
    } else if (ch!='\r' && pos<sizeof(cmd)-1) cmd[pos++]=ch;
  }
  delay(1);
}
