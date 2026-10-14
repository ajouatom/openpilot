// Minimal setup text only. All driving UI is rendered on C4.
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include "display_status.h"
#include "esp_heap_caps.h"

#include "esp_timer.h"
#include "mbedtls/des.h"

static void be32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }

void display_command(uint8_t out[512], uint8_t command, uint32_t value) {
  uint8_t plain[504]={0}, iv[8]="slv3tuzx";
  plain[0]=command; plain[2]=0x1a; plain[3]=0x6d;
  uint32_t stamp=(uint32_t)(esp_timer_get_time()/1000);
  memcpy(plain+4, &stamp, 4);
  if (command==14) plain[8]=value; else be32(plain+8, value);
  memset(out, 0, 512);
  mbedtls_des_context ctx; mbedtls_des_init(&ctx);
  assert(mbedtls_des_setkey_enc(&ctx, (const uint8_t *)"slv3tuzx")==0);
  assert(mbedtls_des_crypt_cbc(&ctx, MBEDTLS_DES_ENCRYPT, 504, iv, plain, out)==0);
  mbedtls_des_free(&ctx); out[510]=0xa1; out[511]=0x1a;
}

void display_decode(const uint8_t in[512], uint8_t out[504]) {
  uint8_t iv[8]="slv3tuzx";
  mbedtls_des_context ctx; mbedtls_des_init(&ctx);
  assert(mbedtls_des_setkey_dec(&ctx, (const uint8_t *)"slv3tuzx")==0);
  assert(mbedtls_des_crypt_cbc(&ctx, MBEDTLS_DES_DECRYPT, 504, iv, in, out)==0);
  mbedtls_des_free(&ctx);
}

// Five columns, seven rows, bit 0 at the top; digits, letters and punctuation.
static const uint8_t glyphs[][5]={
  {62,81,73,69,62},{0,66,127,64,0},{98,81,73,73,70},{34,65,73,73,54},{24,20,18,127,16},
  {39,69,69,69,57},{60,74,73,73,48},{1,113,9,5,3},{54,73,73,73,54},{6,73,73,41,30},
  {126,17,17,17,126},{127,73,73,73,54},{62,65,65,65,34},{127,65,65,34,28},{127,73,73,73,65},
  {127,9,9,9,1},{62,65,73,73,122},{127,8,8,8,127},{0,65,127,65,0},{32,64,65,63,1},
  {127,8,20,34,65},{127,64,64,64,64},{127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},
  {127,9,9,9,6},{62,65,81,33,94},{127,9,25,41,70},{38,73,73,73,50},{1,1,127,1,1},
  {63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},{99,20,8,20,99},{7,8,112,8,7},
  {97,81,73,69,67},{8,8,8,8,8},{0,96,96,0,0},{0,54,54,0,0}
};
static void text(uint8_t *blocks, int x, int y, const char *line) {
  for (; *line; line++, x+=6) {
    char c=*line;
    if (c>='a' && c<='z') c-=32;
    int g=c>='0'&&c<='9'?c-'0':c>='A'&&c<='Z'?c-'A'+10:c=='-'?36:c=='.'?37:c==':'?38:-1;
    if (g<0) continue;
    for (int col=0; col<5; col++) for (int row=0; row<7; row++)
      if ((glyphs[g][col]&(1<<row)) && x+col<240 && y+row<58)
        blocks[(x+col)*58+57-y-row]=255;
  }
}

// A small baseline RGB-compatible JPEG: each 8x8 MCU is a constant YCbCr block.
// This is sufficient for setup text and needs neither a UI library nor a JPEG dependency.
typedef struct { uint8_t *out; size_t used; unsigned bits, count; } writer_t;
static void byte(writer_t *w, uint8_t v) { w->out[w->used++]=v; }
static void word(writer_t *w, uint16_t v) { byte(w,v>>8); byte(w,v); }
static void bits(writer_t *w, unsigned value, unsigned count) {
  for (int i=count-1; i>=0; i--) {
    w->bits=(w->bits<<1)|((value>>i)&1);
    if (++w->count==8) {
      byte(w,w->bits); if ((w->bits&255)==255) byte(w,0);
      w->count=0; w->bits=0;
    }
  }
}
static void block(writer_t *w, int delta) {
  unsigned magnitude=delta<0?-delta:delta, category=0;
  for (unsigned n=magnitude; n; n>>=1) category++;
  bits(w,category,4);
  if (category) bits(w,delta<0?delta-1:delta,category);
  bits(w,0,1); // AC end-of-block.
}
uint8_t *display_setup_image(const char *ssid, const char *password, size_t *length) {
  uint8_t *blocks=heap_caps_calloc(58*240,1,MALLOC_CAP_SPIRAM);
  uint8_t *out=heap_caps_malloc(65536,MALLOC_CAP_SPIRAM);
  if (!blocks || !out) { free(blocks); free(out); return NULL; }
  text(blocks,10,2,"CARROT S3 - PHONE SETUP"); text(blocks,10,12,ssid);
  text(blocks,10,22,"PASSWORD:"); text(blocks,70,22,password);
  text(blocks,10,32,"OPEN 192.168.4.1"); text(blocks,10,42,"SETUP CLOSES AFTER 10 MINUTES");
  writer_t w={.out=out,.used=512};
  word(&w,0xffd8);
  word(&w,0xffdb); word(&w,67); byte(&w,0);
  for (int i=0;i<64;i++) byte(&w,8);
  word(&w,0xffc0); word(&w,17); byte(&w,8); word(&w,1920); word(&w,462); byte(&w,3);
  for (int i=1;i<=3;i++) { byte(&w,i); byte(&w,0x11); byte(&w,0); }
  word(&w,0xffc4); word(&w,49); byte(&w,0);
  for (int i=1;i<=16;i++) byte(&w,i==4?12:0);
  for (int i=0;i<12;i++) byte(&w,i);
  byte(&w,0x10);
  for (int i=1;i<=16;i++) byte(&w,i==1?1:0);
  byte(&w,0);
  word(&w,0xffda); word(&w,12); byte(&w,3);
  for (int i=1;i<=3;i++) { byte(&w,i); byte(&w,0); }
  byte(&w,0); byte(&w,63); byte(&w,0);
  int previous=0;
  for (int i=0;i<58*240;i++) {
    int dc=(int)blocks[i]-128;
    block(&w,dc-previous); previous=dc;
    block(&w,0); block(&w,0);
  }
  if (w.count) bits(&w,255,8-w.count);
  word(&w,0xffd9);
  display_command(out,101,w.used-512); *length=w.used;
  free(blocks); return out;
}
