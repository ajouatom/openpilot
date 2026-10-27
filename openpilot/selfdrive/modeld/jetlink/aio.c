// Bounded Linux AIO reads for FunctionFS. Buffers live until io_destroy completes.
#include <errno.h>
#include <linux/aio_abi.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>

#define DEPTH 4
#define CHUNK 16384
struct slot {
  struct iocb cb;
  unsigned char data[CHUNK];
  uint64_t sequence;
  int pending, ready;
  int64_t result;
};
struct reader {
  aio_context_t context;
  struct slot slots[DEPTH];
  uint64_t issued, consumed;
  int fd, started, failed;
  uint64_t offset_step; // zero for endpoints; nonzero only for regular-file tests
};

void *jetlink_aio_create(int fd, uint64_t offset_step) {
  struct reader *r = calloc(1, sizeof(*r));
  if (!r) return NULL;
  r->fd = fd;
  r->offset_step = offset_step;
  if (syscall(__NR_io_setup, DEPTH, &r->context) < 0) {
    int saved = errno;
    free(r);
    errno = saved;
    return NULL;
  }
  return r;
}

static int submit(struct reader *r, struct slot *s) {
  memset(&s->cb, 0, sizeof(s->cb));
  s->sequence = r->issued;
  s->cb.aio_data = s->sequence;
  s->cb.aio_lio_opcode = IOCB_CMD_PREAD;
  s->cb.aio_fildes = r->fd;
  s->cb.aio_buf = (uintptr_t)s->data;
  s->cb.aio_nbytes = CHUNK;
  s->cb.aio_offset = r->offset_step * s->sequence;
  struct iocb *cb = &s->cb;
  long count = syscall(__NR_io_submit, r->context, 1L, &cb);
  if (count != 1) return r->failed = count < 0 ? -errno : -EIO;
  s->pending = 1;
  s->ready = 0;
  r->issued++;
  return 0;
}

int jetlink_aio_read(void *reader, void *destination, size_t capacity) {
  struct reader *r = reader;
  if (!r || !destination || capacity < CHUNK) return -EINVAL;
  if (r->failed) return r->failed;
  if (!r->started) {
    r->started = 1;
    for (int i = 0; i < DEPTH; i++) {
      int result = submit(r, &r->slots[i]);
      if (result) return result;
    }
  }
  struct slot *next = &r->slots[r->consumed % DEPTH];
  while (!next->ready) {
    struct io_event events[DEPTH];
    long count = syscall(__NR_io_getevents, r->context, 1L, (long)DEPTH, events, NULL);
    if (count < 0) {
      if (errno == EINTR) continue;
      return r->failed = -errno;
    }
    for (long i = 0; i < count; i++) {
      struct io_event *e = &events[i];
      struct slot *s = &r->slots[e->data % DEPTH];
      if (!s->pending || s->sequence != e->data || e->obj != (uintptr_t)&s->cb)
        return r->failed = -EPROTO;
      s->pending = 0;
      s->ready = 1;
      s->result = e->res;
    }
  }
  if (next->sequence != r->consumed || next->result > CHUNK) return r->failed = -EPROTO;
  if (next->result <= 0) return r->failed = next->result < 0 ? (int)next->result : -ENODATA;
  int count = (int)next->result;
  memcpy(destination, next->data, count);
  r->consumed++;
  // Reuse only after that exact read completed and its bytes were copied.
  int result = submit(r, next);
  return result ? result : count;
}

int jetlink_aio_close(void *reader) {
  struct reader *r = reader;
  if (!r) return 0;
  // The transport unbinds before joining the reader. io_destroy also cancels
  // pending reads on early protocol failure. Never free submitted buffers first.
  if (syscall(__NR_io_destroy, r->context) < 0) return -errno;
  free(r);
  return 0;
}
