"""Next-frame maps: actual codec/session plus cancellation and lifetime gates."""
from dataclasses import replace
from types import SimpleNamespace as NS
import json
import threading
import numpy as np
import pytest

from test_nav_rl import SPEC
from test_output_projection import pair
from jetlink import deferred_map as D, packed_codec as C, protocol as P
from jetlink.transport.base import Message, LinkError
from tools.jetlink.server import CarrotSession


def packed(value):
  result = np.arange(SPEC.packed_nelem, dtype=np.float32)
  off = C._layout(SPEC) // 4
  result[off:off+65536] = value
  return result


def setup():
  now = [10_000_000_000]
  sender = D.Sender(SPEC, clock=lambda: now[0])
  receiver = D.Receiver(clock=lambda: now[0])
  sequence = iter(range(100, 1000))
  messages = []
  def send(kind, seq, parts, timeout):
    assert kind == D.MESSAGE and timeout == .15
    body = b''.join(parts)
    messages.append(body)
    receiver.accept(seq, body)
  def flush(): return sender.flush(lambda: next(sequence), send, .15)
  def meta(route=77): return (route, now[0], now[0]+500_000_000)
  return now, sender, receiver, messages, flush, meta


def test_current_camera_context_previous_map_and_deferred_conversion(monkeypatch):
  now, sender, receiver, messages, flush, meta = setup()
  original = C.encode
  conversions = []
  monkeypatch.setattr(C, 'encode', lambda *a: (conversions.append(True), original(*a))[1])
  first, flags = sender.prepare(packed(11), 1, True, meta())
  assert flags == C.FLAG and not sender.used and len(conversions) == 1
  assert flush() == 65560 and len(conversions) == 1
  now[0] += 50_000_000
  current = packed(22); current[0] = 321
  reference, flags = sender.prepare(current, 2, False, meta())
  assert flags == C.FLAG | D.FLAG and sender.used and len(conversions) == 1
  expanded = np.frombuffer(C.decode(receiver.expand(reference, SPEC), SPEC), np.float32)
  expected = current.copy(); off = C._layout(SPEC)//4; expected[off:off+65536] = 11
  assert expanded.tobytes() == expected.tobytes()
  assert flush() == 65560 and len(conversions) == 2
  now[0] += 50_000_000
  third, flags = sender.prepare(packed(33), 3, False, meta())
  expanded = np.frombuffer(C.decode(receiver.expand(third, SPEC), SPEC), np.float32)
  assert np.all(expanded[off:off+65536] == 22)
  assert len(first)-len(reference) == 65520


@pytest.mark.parametrize('change', ['cancel', 'route', 'reset', 'gap', 'backward', 'expired', 'short_instruction'])
def test_inline_current_input_immediately_supersedes_invalid_cache(change):
  now, sender, receiver, messages, flush, meta = setup()
  source = meta(); sender.prepare(packed(11), 1, False, source); flush()
  now[0] += 50_000_000
  metadata = meta(); frame=2; reset=False
  if change == 'cancel': metadata=(0,0,0)
  if change == 'route': metadata=meta(78)
  if change == 'reset': reset=True
  if change == 'gap': frame=3
  if change == 'backward': metadata=(77, source[1]-1, source[2]-1)
  if change == 'expired': now[0]+=400_000_000; metadata=source
  if change == 'short_instruction': metadata=(77, now[0], now[0]+150_000_000)
  wire, flags = sender.prepare(packed(0), frame, reset, metadata)
  assert not flags & D.FLAG and not sender.used
  assert C.decode(wire,SPEC) == packed(0).tobytes()


def test_same_image_does_not_refresh_ttl_or_resend_and_reconnect_has_no_cache():
  now, sender, receiver, messages, flush, meta = setup()
  source=meta(); sender.prepare(packed(11),1,False,source); flush()
  until=sender.cached['until']
  now[0]+=50_000_000
  sender.prepare(packed(11),2,False,source)
  assert sender.used and flush()==0 and sender.cached['until']==until and len(messages)==1
  sender.clear()
  _,flags=sender.prepare(packed(11),3,False,source)
  assert not flags & D.FLAG


@pytest.mark.parametrize('fault', ['missing','expired','identity','route','short','long','model'])
def test_reference_rejected_instead_of_using_wrong_pixels(fault):
  now,sender,receiver,messages,flush,meta=setup()
  sender.prepare(packed(11),1,False,meta());flush();now[0]+=50_000_000
  ref,_=sender.prepare(packed(22),2,False,meta()); spec=SPEC
  if fault=='missing': receiver.clear()
  if fault=='expired': now[0]+=400_000_000
  if fault in ('identity','route'):
    ref=bytearray(ref);off=C._layout(SPEC)+(8 if fault=='route' else 0);ref[off]^=1
  if fault=='short':ref=ref[:-1]
  if fault=='long':ref+=b'x'
  if fault=='model':spec=replace(SPEC,sha256='0'*64)
  with pytest.raises(ValueError):receiver.expand(ref,spec)


@pytest.mark.parametrize('fault', ['crc','short','long','ttl0','ttl351','sequence','route'])
def test_update_validation_and_immutable_owned_pixels(fault):
  now,sender,receiver,messages,flush,meta=setup()
  sender.prepare(packed(11),1,False,meta());flush()
  body=bytearray(messages[0]); fields=list(D.UPDATE.unpack_from(body))
  if fault=='crc': body[-1]^=1
  if fault=='short':body=body[:-1]
  if fault=='long':body+=b'x'
  if fault=='ttl0':fields[2]=0
  if fault=='ttl351':fields[2]=351
  if fault=='sequence':fields[0]+=1
  if fault=='route':fields[1]=0
  body[:D.UPDATE.size]=D.UPDATE.pack(*fields)
  with pytest.raises(ValueError):receiver.accept(100,body)
  original=bytearray(messages[0]);receiver.accept(100,original);original[-1]=99
  assert receiver.cached[3][-1]==11


@pytest.mark.parametrize('metadata', [(True,1,2),(1,1.,2),(1,1,600000002),(0,1,2),(1,11_000_000_000,11_000_000_100)])
def test_bad_metadata(metadata):
  with pytest.raises(ValueError):D.metadata(metadata,10_000_000_000)


def test_noncanonical_new_map_cannot_be_published_as_success():
  now,sender,receiver,messages,flush,meta=setup()
  sender.prepare(packed(11),1,False,meta());flush();now[0]+=50_000_000
  sender.prepare(packed(.5),2,False,meta())
  assert sender.used
  with pytest.raises(ValueError,match='noncanonical'):flush()
  assert len(messages)==1


def test_real_session_uses_old_map_current_camera_and_cancels(monkeypatch):
  client,session,loaded,outputs=pair()
  session.__class__=CarrotSession
  session.deferred_maps=D.Receiver()
  session.deferred_negotiated=True
  session.telemetry.health=None
  client._deferred_map=True; client._nav_u8=True
  client.t.handler=lambda message: session._infer(loaded,message)
  original_send=client.t.send
  def send(kind,seq,parts=(),flags=0,**kw):
    if kind==D.MESSAGE:session.deferred_maps.accept(seq,b''.join(parts))
    else:original_send(kind,seq,parts,flags=flags,**kw)
  monkeypatch.setattr(client.t,'send',send)
  for frame,value in enumerate([11,22,0],1):
    now=D.time.monotonic_ns()
    metadata=(77,now,now+500_000_000) if value else (0,0,0)
    seq=client.infer_begin(np.full(SPEC.warped_shape,frame,np.uint8),packed(value),frame,nav_metadata=metadata)
    assert client.last_map_used==(frame==2)
    client.flush_map(); assert np.isfinite(client.infer_end(seq)).all()
    assert np.all(loaded.host_inputs['nav_map']==(11 if frame==2 else value))
    assert np.any(loaded.host_inputs['new_img'])
  assert session.deferred_maps.cached is None


@pytest.mark.parametrize('bad', [None,{},True,{**D.descriptor(),'version':True},{**D.descriptor(),'pixels':65536.0}])
def test_strict_negotiation(bad):assert not D.matches(bad)

@pytest.mark.parametrize('enabled', [None,'0','1'])
@pytest.mark.parametrize('capable', [False,True])
def test_local_metadata_requires_peer_and_retains_explicit_off(monkeypatch,enabled,capable):
  from openpilot.selfdrive.modeld.jetlink import link
  if enabled is None:monkeypatch.delenv('JETLINK_DEFER_NAV',raising=False)
  else:monkeypatch.setenv('JETLINK_DEFER_NAV',enabled)
  hello={'spec':SPEC.to_dict()}
  if capable:hello[D.IPC_CAPABILITY]=D.descriptor()
  monkeypatch.setattr(link.socket,'AF_UNIX',1,raising=False)
  monkeypatch.setattr(link.socket,'socket',lambda *a:NS(settimeout=lambda *a:None,connect=lambda *a:None,close=lambda:None))
  monkeypatch.setattr(link,'receive',lambda *a:json.dumps(hello).encode())
  client=link.Client();assert client.deferred_nav==(enabled!='0' and capable)
  requests=[]
  monkeypatch.setattr(link,'send_parts',lambda sock,*parts,**kw:requests.append(b''.join(memoryview(x).cast('B') for x in parts)))
  client.reader=NS(receive=lambda *a:memoryview(link.REPLY.pack(7,24,1,25)+bytes(SPEC.output_nelem*4)))
  now=D.time.monotonic_ns();metadata=(77,now,now+500_000_000)
  client.infer(np.zeros(SPEC.warped_shape,np.uint8),packed(11),7,True,nav_metadata=metadata)
  request=requests[0]
  assert link.REQUEST.unpack_from(request)==(7,5 if client.deferred_nav else 1,0)
  assert len(request)==link.REQUEST.size+SPEC.warped_nbytes+SPEC.packed_nbytes+(D.META.size if client.deferred_nav else 0)
  if client.deferred_nav:assert D.META.unpack(request[-D.META.size:])==metadata


def test_hello_and_model_request_retire_cache(monkeypatch):
  from jetlink.server.session import Session
  from jetlink import request_chunks
  client,session,loaded,_=pair();session.__class__=CarrotSession
  session.deferred_maps=D.Receiver();session.request_chunks=request_chunks.Receiver()
  session.deferred_negotiated=True;session.power_lease=None
  monkeypatch.setattr(Session,'handle',lambda *a:None)
  for kind,body in [(P.Msg.HELLO_REQ,{'client':{D.CAPABILITY:D.descriptor()}}),(P.Msg.ENGINE_REQ,{}),(P.Msg.HELLO_REQ,{'client':{}})]:
    session.deferred_maps.cached=(1,77,999,b'old')
    session.handle(Message(kind,10,0,memoryview(json.dumps(body).encode())))
    assert session.deferred_maps.cached is None
  assert not session.deferred_negotiated


def test_map_input_metadata_preserves_integer_clock_and_clears_cancel(monkeypatch):
  from openpilot.selfdrive.navigation import map_client
  monkeypatch.setattr(map_client.time,'monotonic',lambda:10.)
  source=map_client.MapInput();source.client=NS(is_connected=lambda:True,recv=lambda _:None)
  source.image=np.ones((256,256),np.uint8);source.image_time_ns=9_800_000_001
  source.image_time=source.image_time_ns*1e-9;source.route_time=77
  class SM(dict):pass
  sm=SM(navInstructionCarrot=NS(allManeuvers=[]),navRoute=NS(carrotRouteId=77))
  sm.valid={'navInstructionCarrot':True,'navRoute':True};sm.recv_time={'navInstructionCarrot':9.9}
  seen=[];model=NS(views={'nav_map':None},set_nav_metadata=seen.append)
  assert 'nav_map' in source.update(model,sm,0)
  assert seen==[None,(77,9_800_000_001,10_300_000_001)]
  sm['navRoute'].carrotRouteId=78
  assert 'nav_map' not in source.update(model,sm,0) and seen[-1] is None
