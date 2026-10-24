// Bounded FunctionFS writes. No slot is reused before its exact completion.
#include <errno.h>
#include <linux/aio_abi.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#define DEPTH 128
#define CHUNK 32768
#define MAX_FRAMES 136
struct slot { struct iocb cb; unsigned char *data; uint64_t sequence; int pending; };
struct writer {
  aio_context_t context;
  int fd, failed, regular, deferred, depth, pending, quantum;
  uint64_t file_offset, sequence;
  struct slot slots[];
};
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
void *jetlink_writer_create(int fd, int regular, int deferred, int quantum) {
  if ((quantum!=8192 && quantum!=CHUNK) || (!deferred && quantum!=CHUNK)) { errno=EINVAL; return NULL; }
  int depth = deferred ? (1<<20)/quantum : 16;
  struct writer *w = calloc(1, sizeof(*w) + (size_t)depth * (sizeof(struct slot)+quantum));
  if (!w) return NULL;
  w->fd = fd; w->regular = regular; w->deferred = !!deferred;
  w->depth = depth; w->quantum = quantum;
  unsigned char *buffers=(unsigned char *)(w->slots+depth);
  for (int i=0;i<depth;i++) w->slots[i].data=buffers+(size_t)i*quantum;
  if (syscall(__NR_io_setup, w->depth, &w->context) < 0) { int e=errno; free(w); errno=e; return NULL; }
  return w;
}
static int fail(struct writer *w, int error) {
  if (!w->failed) w->failed = error;
  return w->failed;
}
static int collect(struct writer *w, int wait, double seconds) {
  if (!w->pending) return 0;
  struct timespec timeout={.tv_sec=(time_t)seconds,.tv_nsec=(long)((seconds-(time_t)seconds)*1e9)};
  struct io_event events[DEPTH];
  long received=syscall(__NR_io_getevents,w->context,(long)wait,(long)w->depth,events,&timeout);
  if (received<0) return errno==EINTR ? 0 : fail(w,-errno);
  if (received>w->depth) return fail(w,-EPROTO);
  for (long j=0;j<received;j++) {
    struct io_event *e=&events[j]; struct slot *s=NULL;
    for (int i=0;i<w->depth;i++) if (e->obj==(uintptr_t)&w->slots[i].cb) s=&w->slots[i];
    if (!s || !s->pending || e->data!=s->sequence) { fail(w,-EPROTO); continue; }
    s->pending=0; w->pending--;
    // FunctionFS 4.9 completes with (ret, ret); regular files use (ret, 0).
    if (e->res!=(int64_t)s->cb.aio_nbytes || (e->res2 && e->res2!=e->res))
      fail(w,e->res<0 ? (int)e->res : -EIO);
  }
  return w->failed ? w->failed : (int)received;
}
int jetlink_writer_poll(void *pointer) {
  struct writer *w=pointer;
  if (!w) return -EINVAL;
  collect(w,0,0);
  return w->failed;
}
int jetlink_writer_send(void *pointer, const unsigned char *data, size_t size,
                        const uint32_t *lengths, unsigned count, double budget) {
  struct writer *w=pointer;
  if (!w || !data || !lengths || !count || count>MAX_FRAMES || !(budget>0 && budget<=.5)) return -EINVAL;
  if (w->failed) return w->failed;
  size_t total=0;
  for (unsigned i=0;i<count;i++) { if (!lengths[i] || lengths[i]>(unsigned)w->quantum) return -EINVAL; total+=lengths[i]; }
  if (total!=size) return -EINVAL;
  if (jetlink_writer_poll(w)<0) return w->failed;
  if (!w->deferred && w->pending) return fail(w,-EBUSY);
  double deadline=now()+budget;
  unsigned issued=0; size_t offset=0;
  while (issued<count || (!w->deferred && w->pending)) {
    if (now()>=deadline) return fail(w,-ETIMEDOUT);
    struct iocb *batch[DEPTH]; unsigned n=0;
    for (int i=0;i<w->depth && issued+n<count;i++) {
      struct slot *s=&w->slots[i]; if (s->pending) continue;
      size_t len=lengths[issued+n];
      memcpy(s->data,data+offset,len); offset+=len;
      memset(&s->cb,0,sizeof(s->cb)); s->sequence=w->sequence++;
      s->cb.aio_data=s->sequence; s->cb.aio_lio_opcode=IOCB_CMD_PWRITE; s->cb.aio_fildes=w->fd;
      s->cb.aio_buf=(uintptr_t)s->data; s->cb.aio_nbytes=len;
      s->cb.aio_offset=w->regular ? w->file_offset+offset-len : 0;
      batch[n++]=&s->cb;
    }
    if (n) {
      long submitted=syscall(__NR_io_submit,w->context,(long)n,batch);
      if (submitted>0) {
        if (submitted>(long)n) return fail(w,-EPROTO);
        for (long j=0;j<submitted;j++) for (int k=0;k<w->depth;k++)
          if (batch[j]==&w->slots[k].cb) w->slots[k].pending=1;
        w->pending+=(int)submitted;
        issued+=(unsigned)submitted;
      }
      // Partial submission ends this stream; queued buffers stay owned until close.
      if (submitted!=(long)n) return fail(w,submitted<0 ? -errno : -EIO);
    }
    if (w->deferred && issued==count) break;
    double left=deadline-now(); if (left<=0) return fail(w,-ETIMEDOUT);
    if (collect(w,1,left)<0) return w->failed;
  }
  w->file_offset+=size;
  return 0;
}
int jetlink_writer_close(void *pointer) {
  struct writer *w=pointer; if (!w) return 0;
  if (w->deferred) {
    // io_destroy may itself hang for a suspended USB bus. Never call it while
    // queued requests still own memory. Caller has already unbound the link.
    double deadline=now()+.5;
    while (w->pending && now()<deadline) {
      double left=deadline-now(); if (left<=0) break;
      collect(w,1,left);
    }
    if (w->pending) return -ETIMEDOUT;
  }
  // Caller unbinds FunctionFS first on failures; never free kernel-owned slots.
  if (syscall(__NR_io_destroy,w->context)<0) return -errno;
  free(w); return 0;
}
