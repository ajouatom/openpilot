// Maintenance transport. Authenticate the size and SHA-256 of the whole image
// before opening the unused OTA slot. Only the main loop mutates flash.
#pragma once
#include <Update.h>
#include <mbedtls/sha256.h>

namespace ota {
constexpr char UUID[]="db340a94-804c-4ead-af37-8a0e6af754a1";
struct Block { uint16_t size; uint8_t data[512]; };
QueueHandle_t queue;
BLECharacteristic *attr;
mbedtls_sha256_context sha;
bool busy=false, reboot=false, parkedManifest=false;
uint32_t owner, size, offset, last, lastBus;
uint8_t digest[32];
enum Result : uint8_t { Idle, Active, Done, Rejected };
void report(Result state) {
  uint8_t bytes[5]={state};
  memcpy(bytes+1,&offset,4); attr->setValue(bytes,5);
}
class Reader : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v=c->getValue();
    if (!v.length() || v.length()>512) return;
    Block b; b.size=v.length(); memcpy(b.data,v.c_str(),b.size);
    xQueueSend(queue,&b,0);
  }
};
void setup(BLEService *svc) {
  queue=xQueueCreate(2,sizeof(Block));
  if (!queue) return;
  attr=svc->createCharacteristic(UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);
  attr->setCallbacks(new Reader()); report(Idle);
}
void stop() {
  if (busy) { Update.abort(); mbedtls_sha256_free(&sha); }
  busy=false; report(Rejected);
}
uint32_t word(const uint8_t *p) { uint32_t n; memcpy(&n,p,4); return n; }
void tick(uint32_t now,uint32_t sid,bool transmitting,const uint8_t *key) {
  if (!queue) return;
  if (reboot) { if (now-last>700) ESP.restart(); return; }
  if (busy && (!sid || sid!=owner || now-last>20000 || (!parkedManifest && now-lastBus<5000))) stop();
  Block b;
  if (xQueueReceive(queue,&b,0)!=pdTRUE) return;
  // Type 4 is signed only by the comma updater after checking live parked state.
  bool parkedRequest=b.data[0]==4;
  if ((b.data[0]==1 || parkedRequest) && b.size==73 && !busy && !transmitting && sid &&
      (parkedRequest || now-lastBus>=5000) && word(b.data+1)==sid) {
    uint8_t mac[32], diff=0;
    if (mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),key,32,b.data,41,mac)) { stop(); return; }
    for (int i=0;i<32;i++) diff|=mac[i]^b.data[41+i];
    size=word(b.data+5); offset=0;
    if (diff || size<256 || size>0x300000 || !Update.begin(size,U_FLASH)) { stop(); return; }
    owner=sid; busy=true; parkedManifest=parkedRequest; memcpy(digest,b.data+9,32);
    mbedtls_sha256_init(&sha); mbedtls_sha256_starts(&sha,0);
  } else if (busy && b.size>5 && b.data[0]==2 && word(b.data+1)==offset && uint32_t(b.size-5)<=size-offset) {
    size_t n=b.size-5;
    if (mbedtls_sha256_update(&sha,b.data+5,n) || Update.write(b.data+5,n)!=n) { stop(); return; }
    offset+=n;
  } else if (busy && b.size==1 && b.data[0]==3 && offset==size) {
    uint8_t actual[32];
    if (mbedtls_sha256_finish(&sha,actual) || memcmp(actual,digest,32) || !Update.end()) { stop(); return; }
    mbedtls_sha256_free(&sha); busy=false; reboot=true; last=now; report(Done); return;
  } else { stop(); return; }
  last=now; report(Active);
}
}
