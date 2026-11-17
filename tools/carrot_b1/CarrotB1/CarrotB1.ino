// Carrot B1, ESP32-S3. 500 kbit/s Classic CAN; commands never contain raw CAN.
#include <Arduino.h>
#include <esp_app_desc.h>
#include <BLEDevice.h>
#include <Preferences.h>
#include <BLESecurity.h>
#include <mbedtls/sha256.h>
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
BLECharacteristic *statusValue, *metaValue, *traceValue;
uint16_t traceSeq;
unsigned confirmedFrames;
const char *traceReason="idle";
uint32_t traceAlerts;

QueueHandle_t inbox, claims;
bool claimed;
struct Claim { uint32_t session; uint8_t seed[16]; };
class ClaimWriter : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String value=c->getValue();
    if (value.length()!=sizeof(Claim)) return;
    Claim candidate;
    memcpy(&candidate,value.c_str(),sizeof(candidate));
    xQueueSend(claims,&candidate,0);
  }
};
std::atomic<uint32_t> session{0};
uint32_t currentSession, deadline, startUs, errorAt, overrideAt;
uint16_t lastSeq, ack, activeSeq;
bool leased, active, installed, online, inFlight, haveSaved, owned;
uint32_t ownedAt, darkAt;
bool sawLamp, awaitingLamp;
uint32_t awaitSince;
Op ownedDirection;
uint8_t original[8];
b1::TapFrame frames[7];
b1::TapFeedback tapFeedback;
int frameCount, nextFrame;
int retriedFrame=-1;
unsigned arbitrationRetries, frameRetries;
uint32_t retryAtUs;
// Initial arbitration has no preceding delivered payload to duplicate. Keep
// later retries within their existing slot and leave margin to the next frame.
uint32_t arbitrationWindow(int index) {
  if (!index) return 40000;
  if (index+1==frameCount) return 18000;
  uint32_t slot=frames[index+1].offsetUs-frames[index].offsetUs;
  return min(uint32_t(18000),slot-5000);
}

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
void finish(bool success, const char *reason="cancelled") {
  awaitingLamp=false;
  if (active) {
    active=inFlight=false;
    if (!mode(false)) { success=false; reason="listen_mode_failed"; }
  }
  traceReason=success ? "completed" : reason;
  if (success) ack=activeSeq;
  else { ack=0; owned=false; }
}
// A lost single shot did not deliver that frame. Earlier confirmed press
// frames can already have triggered the OEM three-flash sequence. Do not
// retransmit the tap from the beginning or claim all frames were sent.
void endArbitration(uint32_t now) {
  bool observe=owned && confirmedFrames>0 && guard(now) && matched(now);
  finish(false,"arbitration_budget");
  if (observe && online) {
    owned=true;
    awaitingLamp=true;
    awaitSince=now;
  }
}
void startTap(const Request &req) {
  uint32_t now=millis();
  if (active || awaitingLamp) return;
  traceSeq=req.seq;
  confirmedFrames=0;
  arbitrationRetries=0;
  retriedFrame=-1;
  frameRetries=0; retryAtUs=0;
  traceAlerts=0;
  if (!neutral(now)) { traceReason="not_neutral"; return; }
  if (!guard(now)) { traceReason="expired_at_start"; return; }
  traceReason="admitted";
  memcpy(original,stalk.data,8);
  unsigned counter=haveSaved && !memcmp(saved.original,original,8) ? saved.last : original[1]>>4;
  frameCount=b1::encode(original,req.op==Left?b1::Direction::Left:b1::Direction::Right,counter,frames);
  if (!frameCount) { traceReason="invalid_template"; return; }
  memcpy(saved.original,original,8); saved.last=frames[frameCount-1].data[1]>>4;
  if (prefs.putBytes("counter",&saved,sizeof(saved))!=sizeof(saved)) { traceReason="counter_store_failed"; errorAt=now; return; }
  haveSaved=true;
  active=true; owned=true; ownedAt=now; activeSeq=req.seq; ack=0;
  tapFeedback={};
  ownedDirection=Op(req.op); sawLamp=false; darkAt=0;
  nextFrame=0; inFlight=false;
  if (!mode(true) || !guard(millis())) { finish(false,"mode_or_lease"); errorAt=now; return; }
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
      uint8_t stalkReport=s.data[1]&5;
      if (!tapFeedback.accepts(stalkReport, lamps(),
          ownedDirection==Left ? b1::Direction::Left : b1::Direction::Right,
          confirmedFrames>0, micros()-startUs, frames[frameCount-1].offsetUs+50000)) {
        finish(false,"feedback_override"); overrideAt=now;
      }
      else if (lamps()==expected) { if (confirmedFrames) sawLamp=true; darkAt=0; }
      else if (sawLamp && !darkAt) darkAt=now;
    }
    if (owned && (!s.count || (&s==&stalk && memcmp(original,s.data,8)))) {
      finish(false,"driver_override"); overrideAt=now;
    }
  }
}
class Link : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { session.store(esp_random()|1U); }
  void onConnect(BLEServer *s, ble_gap_conn_desc *event) override {
    // 10-15 ms ATT connection interval. Preserve command and lease deadlines.
    s->updateConnParams(event->conn_handle,8,12,0,400);
  }
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
constexpr char RELEASE_TAG[]="carrot-b1-release=5";
char buildId[65];

void setup() {
  const auto *app=esp_app_get_description();
  for (size_t i=0; i<sizeof(app->app_elf_sha256); ++i)
    snprintf(buildId+2*i, 3, "%02x", app->app_elf_sha256[i]);
  Serial.begin(115200); led(22,9,0);
  claims=xQueueCreate(1,sizeof(Claim));
  inbox=xQueueCreate(8,sizeof(Request));
  if (!inbox || !claims || !prefs.begin("carrot-b1",false)) { led(40,0,0); while(true) delay(1000); }
  BLEDevice::init("Carrot-B1");
  BLEDevice::setMTU(517);
  if (prefs.getBytesLength("auth")!=sizeof(key)) {
    esp_fill_random(key,sizeof(key));
    if (prefs.putBytes("auth",key,sizeof(key))!=sizeof(key)) { led(40,0,0); while(true) delay(1000); }
  }
  if (prefs.getBytes("auth",key,sizeof(key))!=sizeof(key)) { led(40,0,0); while(true) delay(1000); }
  haveSaved=prefs.getBytesLength("counter")==sizeof(saved) && prefs.getBytes("counter",&saved,sizeof(saved))==sizeof(saved);
  claimed=prefs.getBytesLength("owner-key")==sizeof(key);
  if (claimed && prefs.getBytes("owner-key",key,sizeof(key))!=sizeof(key)) { led(40,0,0); while(true) delay(1000); }
  BLESecurity::setCapability(ESP_IO_CAP_NONE);
  BLESecurity::setAuthenticationMode(true,false,true);
  mode(false);
  auto *server=BLEDevice::createServer(); server->setCallbacks(new Link());
  auto *svc=server->createService(SERVICE);
  statusValue=svc->createCharacteristic(STATUS_UUID,BLECharacteristic::PROPERTY_READ);
  metaValue=svc->createCharacteristic(META,BLECharacteristic::PROPERTY_READ);
  traceValue=svc->createCharacteristic("db340a96-804c-4ead-af37-8a0e6af754a1",BLECharacteristic::PROPERTY_READ);
  svc->createCharacteristic(CONTROL,BLECharacteristic::PROPERTY_WRITE)->setCallbacks(new Writer());
  auto *claim=svc->createCharacteristic("db340a95-804c-4ead-af37-8a0e6af754a1",BLECharacteristic::PROPERTY_WRITE);
  claim->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);
  claim->setCallbacks(new ClaimWriter());
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
  now=millis();  // receiveCan may have published newer sample timestamps.
  uint32_t alerts=0;
  if (online && twai_read_alerts(&alerts,0)==ESP_OK) {
    constexpr uint32_t lostShot=TWAI_ALERT_ARB_LOST | TWAI_ALERT_TX_FAILED;
    if (active && inFlight && alerts==lostShot) {
      int frame=nextFrame-1;
      uint32_t elapsed=micros()-startUs;
      uint32_t due=elapsed+600+(esp_random()%901);
      // One shot lost arbitration before delivering payload. Retry only this
      // frame, inside its original slot, and never retry an ambiguous TX error.
      if (frameRetries<(frame==0 ? 16U : 8U) && due-frames[frame].offsetUs<arbitrationWindow(frame)) {
        ++frameRetries; ++arbitrationRetries;
        retriedFrame=frame; nextFrame=frame; retryAtUs=due; inFlight=false;
      } else {
        endArbitration(millis());
      }
      traceAlerts=alerts;
    } else if (alerts & ~TWAI_ALERT_TX_SUCCESS) {
      if (active) traceAlerts=alerts;
      finish(false,"twai_alert"); stalk.count=feedback.count=0; errorAt=now;
    } else if (active && inFlight && (alerts&TWAI_ALERT_TX_SUCCESS)) {
      if (!confirmedFrames && frameRetries) startUs=micros();
      ++confirmedFrames; frameRetries=0; retryAtUs=0;
      inFlight=false;
      if (nextFrame==frameCount) finish(true);
    }
  }
  ota::tick(now,sid,active,key);
  Claim candidate;
  if (xQueueReceive(claims,&candidate,0)==pdTRUE && !claimed && sid && candidate.session==sid &&
      now<180000 && !active && !ota::busy) {
    uint8_t nextKey[32];
    if (!mbedtls_sha256(candidate.seed,sizeof(candidate.seed),nextKey,0) &&
        prefs.putBytes("owner-key",nextKey,sizeof(nextKey))==sizeof(nextKey)) {
      memcpy(key,nextKey,sizeof(key)); claimed=true;
    }
  }
  Request r;
  while (xQueueReceive(inbox,&r,0)==pdTRUE) {
    if (ota::busy || ota::reboot || !sid || r.session!=sid || int16_t(r.seq-lastSeq)<=0 || !r.ttl5ms || r.ttl5ms>70 ||
        r.op>HoldRight) continue;
    if (now-r.stamp>=uint32_t(r.ttl5ms)*5) {
      if (!active && (r.op==Left || r.op==Right)) {
        traceSeq=r.seq; confirmedFrames=0; arbitrationRetries=0; retriedFrame=-1;
        traceAlerts=0; traceReason="expired_on_arrival";
      }
      continue;
    }
    if (!claimed) {
      if (prefs.putBytes("owner-key",key,sizeof(key))!=sizeof(key)) continue;
      claimed=true;
    }
    lastSeq=r.seq;
    if (r.op==Stop) { leased=false; finish(false,"stop"); continue; }
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
  // Admission and CAN driver reinstallation can cross millisecond boundaries.
  // Compare against a clock sampled after both operations, including on rollover.
  now=millis();
  if (owned && !guard(now)) finish(false,"lease_expired");
  if (owned && !matched(now)) finish(false,"stale_can");
  if (owned && now-ownedAt>300000) finish(false,"ownership_expired");
  if (awaitingLamp) {
    // receiveCan already cancelled ownership on opposite lamps, an invalid
    // frame, changed original stalk, or independent lever operation.
    if (now-awaitSince>=350) {
      finish(false);
    } else if (sawLamp && owned && guard(now) && matched(now)) {
      ack=activeSeq;
      traceReason="lamp_confirmed_partial";
      awaitingLamp=false;
    }
  }
  if (active) {
    uint32_t elapsed=micros()-startUs;
    if (!guard(millis()) || (inFlight && elapsed-frames[nextFrame-1].offsetUs>40000)) { finish(false,"tx_timeout_or_lease"); errorAt=now; }
    else if (!inFlight && nextFrame<frameCount && elapsed>=frames[nextFrame].offsetUs && elapsed>=retryAtUs) {
      twai_message_t f={}; f.identifier=0x3c1; f.data_length_code=8; f.ss=1;
      memcpy(f.data,frames[nextFrame].data,8);
      if (frameRetries && elapsed-frames[nextFrame].offsetUs>=arbitrationWindow(nextFrame)) {
        endArbitration(millis());
      } else if (elapsed-frames[nextFrame].offsetUs>=50000 || twai_transmit(&f,0)!=ESP_OK) { finish(false,"late_or_submit_failed"); errorAt=now; }
      else { ++nextFrame; inFlight=true; }
    }
  }
  static uint32_t published=0;
  if (now-published>=20) {
    published=now;
    State state=!online || (errorAt && now-errorAt<500) ? Fault : overrideAt && now-overrideAt<400 ? Override :
      !matched(now) ? Missing : active ? Sending : (!awaitingLamp && neutral(now)) ? Ready : Waiting;
    Report report={sid,now,ack,state,lamps(),2,lastSeq};
    metaValue->setValue((uint8_t*)&report,sizeof(report));
    char trace[192];
    snprintf(trace,sizeof(trace),"seq=%u confirmed=%u reason=%s alerts=%lx retries=%u frame=%d elf=%s %s",traceSeq,confirmedFrames,traceReason,(unsigned long)traceAlerts,arbitrationRetries,retriedFrame,buildId,RELEASE_TAG);
    traceValue->setValue(trace);
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
