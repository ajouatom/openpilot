// Same-frame, bounded FunctionFS writes. No slot is reused before completion.
#include <errno.h>
#include <linux/aio_abi.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#define DEPTH 16
#define CHUNK 32768
#define MAX_FRAMES 34
struct slot { struct iocb cb; unsigned char data[CHUNK]; uint64_t sequence; int pending; };
struct writer { aio_context_t context; struct slot slots[DEPTH]; int fd, failed; uint64_t file_offset; int regular; };
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
void *jetlink_writer_create(int fd, int regular) {
  struct writer *w = calloc(1, sizeof(*w));
  if (!w) return NULL;
  w->fd = fd; w->regular = regular;
  if (syscall(__NR_io_setup, DEPTH, &w->context) < 0) { int e=errno; free(w); errno=e; return NULL; }
  return w;
}
int jetlink_writer_send(void *pointer, const unsigned char *data, size_t size,
                        const uint32_t *lengths, unsigned count, double budget) {
  struct writer *w=pointer;
  if (!w || !data || !lengths || !count || count>MAX_FRAMES || !(budget>0 && budget<=.5)) return -EINVAL;
  if (w->failed) return w->failed;
  size_t total=0;
  for (unsigned i=0;i<count;i++) { if (!lengths[i] || lengths[i]>CHUNK) return -EINVAL; total+=lengths[i]; }
  if (total!=size) return -EINVAL;
  for (int i=0;i<DEPTH;i++) if (w->slots[i].pending) return w->failed=-EBUSY;
  double deadline=now()+budget;
  unsigned issued=0, completed=0; size_t offset=0;
  while (completed<count) {
    if (now()>=deadline) return w->failed=-ETIMEDOUT;
    struct iocb *batch[DEPTH]; unsigned n=0;
    for (int i=0;i<DEPTH && issued+n<count;i++) {
      struct slot *s=&w->slots[i]; if (s->pending) continue;
      unsigned seq=issued+n; size_t len=lengths[seq];
      memcpy(s->data,data+offset,len); offset+=len;
      memset(&s->cb,0,sizeof(s->cb)); s->sequence=seq;
      s->cb.aio_data=seq; s->cb.aio_lio_opcode=IOCB_CMD_PWRITE; s->cb.aio_fildes=w->fd;
      s->cb.aio_buf=(uintptr_t)s->data; s->cb.aio_nbytes=len;
      s->cb.aio_offset=w->regular ? w->file_offset+offset-len : 0;
      batch[n++]=&s->cb;
    }
    if (n) {
      long submitted=syscall(__NR_io_submit,w->context,(long)n,batch);
      if (submitted>0) {
        for (long j=0;j<submitted;j++) for (int k=0;k<DEPTH;k++)
          if (batch[j]==&w->slots[k].cb) w->slots[k].pending=1;
        issued+=(unsigned)submitted;
      }
      // Partial submission ends this stream; queued buffers stay owned until close.
      if (submitted!=(long)n) return w->failed=submitted<0 ? -errno : -EIO;
    }
    double left=deadline-now(); if (left<=0) return w->failed=-ETIMEDOUT;
    struct timespec timeout={.tv_sec=(time_t)left,.tv_nsec=(long)((left-(time_t)left)*1e9)};
    struct io_event events[DEPTH];
    long received=syscall(__NR_io_getevents,w->context,1L,(long)DEPTH,events,&timeout);
    if (received<0) { if (errno==EINTR) continue; return w->failed=-errno; }
    if (!received) return w->failed=-ETIMEDOUT;
    for (long j=0;j<received;j++) {
      struct io_event *e=&events[j]; struct slot *s=NULL;
      for (int i=0;i<DEPTH;i++) if (e->obj==(uintptr_t)&w->slots[i].cb) s=&w->slots[i];
      if (!s || !s->pending || e->data!=s->sequence) return w->failed=-EPROTO;
      s->pending=0;
      // FunctionFS 4.9 completes with (ret, ret); regular files use (ret, 0).
      if (e->res!=(int64_t)s->cb.aio_nbytes || (e->res2 && e->res2!=e->res)) return w->failed=e->res<0 ? (int)e->res : -EIO;
      completed++;
    }
  }
  w->file_offset+=size;
  return 0;
}
int jetlink_writer_close(void *pointer) {
  struct writer *w=pointer; if (!w) return 0;
  // Caller unbinds FunctionFS first on failures; never free kernel-owned slots.
  if (syscall(__NR_io_destroy,w->context)<0) return -errno;
  free(w); return 0;
}
