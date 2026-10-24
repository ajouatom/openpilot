"""Native bounded request writer completion, ownership, and exact-byte checks."""
import ctypes
import errno
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink.transport.request_writer import Writer, library

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Linux native AIO')


@pytest.mark.parametrize('deferred,quantum', [(False,32768), (True,32768), (True,8192)])
def test_native_regular_file_exact_order_and_repeated_requests(tmp_path, deferred, quantum):
  lib = library()
  assert lib is not None
  path = tmp_path / 'output'
  with path.open('w+b') as f:
    writer = Writer(f.fileno(), lib, regular=True, deferred=deferred, quantum=quantum)
    expected = bytearray()
    try:
      for count in (1, 2, 4, 15, 16, 17, 33, 15):
        packets = [bytes([i % 251]) * (32768 if i % 2 else 16384) for i in range(count)]
        writer.send(packets, .5)
        expected.extend(b''.join(packets))
    finally:
      writer.close()
    writer.close()
    f.seek(0)
    assert f.read() == expected
    with pytest.raises(RuntimeError): writer.send([b'x'], .5)


def test_native_invalid_descriptor_fails_then_closes():
  writer = Writer(-1, library())
  try:
    with pytest.raises(OSError): writer.send([b'x' * 32768] * 4, .5)
    with pytest.raises(OSError): writer.send([b'x'], .5)
  finally:
    writer.close()


@pytest.fixture
def fault_library(tmp_path):
  source = Path(__file__).resolve().parents[2] / 'third_party/jetlink/jetlink/transport/request_writer.c'
  harness = r'''
#include <stdarg.h>
#include <errno.h>
#include <linux/aio_abi.h>
#include <stdint.h>
#include <sys/syscall.h>
#include <unistd.h>
long fake_syscall(long number, ...);
#define syscall fake_syscall
#include "request_writer.c"
#undef syscall
static struct iocb *pending[DEPTH];
static int pending_count, mode, destroyed;
void configure(int m) { mode=m; pending_count=0; destroyed=0; }
void set_mode(int m) { mode=m; }
int queued_count(void) { return pending_count; }
int queued_byte(int i, int offset) {
 if (i<0 || i>=pending_count || offset<0 || (uint64_t)offset>=pending[i]->aio_nbytes) return -1;
 return ((unsigned char *)(uintptr_t)pending[i]->aio_buf)[offset];
}
int destroy_count(void) { return destroyed; }
long fake_syscall(long number, ...) {
 va_list args; va_start(args,number); long result=0;
 if (number==__NR_io_setup) { (void)va_arg(args,int); *va_arg(args,aio_context_t *)=1; }
 else if (number==__NR_io_submit) {
  (void)va_arg(args,aio_context_t); long n=va_arg(args,long); struct iocb **items=va_arg(args,struct iocb **);
  if (mode==1) { errno=ENOMEM; result=-1; }
  else { if (mode==2) n--; for (long i=0;i<n;i++) pending[pending_count++]=items[i]; result=n; }
 } else if (number==__NR_io_getevents) {
  (void)va_arg(args,aio_context_t); (void)va_arg(args,long); (void)va_arg(args,long);
  struct io_event *events=va_arg(args,struct io_event *);
  if (mode==3) result=0;
  else {
   result=pending_count;
   for (int i=0;i<pending_count;i++) {
    struct iocb *cb=pending[pending_count-i-1];
    events[i]=(struct io_event){.obj=(uintptr_t)cb,.data=cb->aio_data,.res=cb->aio_nbytes,.res2=0};
    if (mode==4) events[i].res--;
    if (mode==5) events[i].data+=100;
    if (mode==6) events[i].res=-ESHUTDOWN;
    if (mode==8) events[i].res2=events[i].res;
    if (mode==9) events[i].res2=events[i].res+1;
   }
   pending_count=0;
  }
 } else if (number==__NR_io_destroy) {
  destroyed++;
  if (mode==7) { errno=EIO; result=-1; } else pending_count=0;
 } else { errno=ENOSYS; result=-1; }
 va_end(args); return result;
}
'''
  (tmp_path / 'request_writer.c').write_bytes(source.read_bytes())
  (tmp_path / 'harness.c').write_text(harness)
  path = tmp_path / 'test.so'
  subprocess.run(['cc','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(tmp_path/'harness.c'),'-o',str(path)],check=True)
  lib=ctypes.CDLL(str(path),use_errno=True)
  real=library()
  for name in ('jetlink_writer_create','jetlink_writer_send','jetlink_writer_poll','jetlink_writer_close'):
    getattr(lib,name).argtypes=getattr(real,name).argtypes
    getattr(lib,name).restype=getattr(real,name).restype
  return lib


@pytest.mark.parametrize('mode', [0,1,2,3,4,5,6,8,9])
def test_completion_reordering_and_faults_keep_context_until_close(fault_library, mode):
  lib=fault_library; lib.configure(mode)
  writer=Writer(1,lib)
  if mode in (0,8):
    writer.send([b'x'*32768]*33,.5)  # completions deliberately reversed
  else:
    with pytest.raises(OSError): writer.send([b'x'*32768]*33,.5)
    with pytest.raises(OSError): writer.send([b'x'],.5)
  assert writer.context is not None and lib.destroy_count()==0
  writer.close()
  assert writer.context is None and lib.destroy_count()==1


def test_failed_destroy_never_releases_owned_context(fault_library):
  lib=fault_library;lib.configure(7);writer=Writer(1,lib)
  original=writer.context
  with pytest.raises(OSError):writer.close()
  assert writer.context==original
  lib.configure(0);writer.close()
  assert writer.context is None


def test_deferred_send_returns_before_notifications_and_owns_all_bytes(fault_library):
  lib=fault_library; lib.configure(3)
  writer=Writer(1,lib,deferred=True)
  source=bytearray(b'a'*32768)
  writer.send([source]*15,.1)
  source[:]=b'z'*len(source)
  writer.send([b'b'*32768]*15,.1)
  assert lib.queued_count()==30
  for i in range(30):
    assert lib.queued_byte(i,0)==lib.queued_byte(i,32767)==ord('a' if i<15 else 'b')
  # The1MiB budget is real: a stalled consumer cannot grow the queue or
  # overwrite the oldest request, even when a new batch only partially fits.
  with pytest.raises(OSError) as caught: writer.send([b'c'*32768]*3,.01)
  assert caught.value.errno==errno.ETIMEDOUT
  assert lib.queued_count()==32 and lib.queued_byte(0,0)==ord('a')
  assert writer.context is not None and lib.destroy_count()==0
  lib.set_mode(0); writer.close()
  assert writer.context is None and lib.destroy_count()==1


def test_deferred_late_error_latches_before_next_send(fault_library):
  lib=fault_library; lib.configure(3); writer=Writer(1,lib,deferred=True)
  writer.send([b'a'*32768]*15,.1)
  lib.set_mode(6)
  with pytest.raises(OSError) as caught: writer.poll()
  assert caught.value.errno==errno.ESHUTDOWN
  assert lib.queued_count()==0
  with pytest.raises(OSError): writer.send([b'b'],.1)
  assert lib.queued_count()==0
  writer.close()


def test_deferred_close_does_not_destroy_pending_kernel_buffers(fault_library):
  lib=fault_library; lib.configure(3); writer=Writer(1,lib,deferred=True)
  writer.send([b'a'*32768],.1)
  original=writer.context
  with pytest.raises(OSError) as caught: writer.close()
  assert caught.value.errno==errno.ETIMEDOUT
  assert writer.context==original and lib.destroy_count()==0
  assert lib.queued_byte(0,32767)==ord('a')
  lib.set_mode(0); writer.close()
  assert writer.context is None and lib.destroy_count()==1


def test_small_physical_writes_preserve_framed_bytes_and_bounded_ownership(fault_library):
  from jetlink import request_chunks as chunks, protocol
  lib=fault_library; lib.configure(3)
  writer=Writer(1,lib,deferred=True,quantum=8192)
  raw=bytes(range(251))*1828
  packets=chunks.packets(protocol.Msg.INFER_REQ,19,(raw,))
  expected=b''.join(packets)
  writer.send(packets,.1)
  assert lib.queued_count()==len(expected)//8192
  for i in range(lib.queued_count()):
    for offset in (0,31,32,4095,8191):
      assert lib.queued_byte(i,offset)==expected[i*8192+offset]
    assert lib.queued_byte(i,8192)==-1
  lib.set_mode(0); writer.close()
