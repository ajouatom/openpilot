"""HTTP 자식 코드 자체를 실행해 인터페이스 고정과 TLS 정책을 검사한다."""
import contextlib
import http.client
import importlib.util
import io
from pathlib import Path
import socket
import ssl
import sys
import os
import unittest
from unittest.mock import patch


if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('https_client_fixture',HERE.parent/'host_client.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class HTTPS(unittest.TestCase):
    def execute(self, *, bind_failure=False, certificate_failure=False, remote='140.82.112.6'):
        events=[]
        class Raw:
            def __enter__(self):return self
            def __exit__(self,*_):events.append('raw_closed')
            def settimeout(self,seconds):events.append(('raw_timeout',seconds))
            def setsockopt(self,level,option,value):
                events.append(('device',level,option,value))
                if bind_failure:raise PermissionError('fixture bind')
            def bind(self,address):events.append(('source',address))
            def connect(self,address):events.append(('connect',address))
        class Secure:
            def __enter__(self):return self
            def __exit__(self,*_):events.append('tls_closed')
            def settimeout(self,seconds):events.append(('tls_timeout',seconds))
            def getsockname(self):return ('192.168.60.11',52341)
            def getpeername(self):return ('140.82.112.6',443)
            def sendall(self,data):events.append(('request',data))
        class Context:
            check_hostname=True
            verify_mode=ssl.CERT_REQUIRED
            minimum_version=None
            def wrap_socket(self,raw,*,server_hostname):
                events.append(('tls',server_hostname,self.minimum_version,self.verify_mode,self.check_hostname))
                if certificate_failure:raise ssl.SSLCertVerificationError('fixture verify')
                return Secure()
        class Response:
            status=200
            def __init__(self,sock):events.append('response_created')
            def begin(self):events.append('headers_only')
            def close(self):events.append('response_closed')
        output=io.StringIO();exit_code=0
        with patch.object(socket,'socket',return_value=Raw()), \
             patch.object(ssl,'create_default_context',return_value=Context()), \
             patch.object(http.client,'HTTPResponse',Response), \
             patch.object(sys,'argv',['-c','usb0','192.168.60.11',remote,m.PROBE]), \
             contextlib.redirect_stdout(output):
            try:exec(compile(m.BOUND_HTTPS_CODE,'bound_https','exec'),{'__name__':'__main__'})
            except SystemExit as error:exit_code=error.code
        return exit_code,output.getvalue(),events

    def test_numeric_peer_device_and_source_binding_precede_verified_tls(self):
        code,out,events=self.execute()
        self.assertEqual(code,0)
        self.assertEqual(out.strip(),'200 192.168.60.11 140.82.112.6')
        device=next(e for e in events if isinstance(e,tuple) and e[0]=='device')
        self.assertEqual(device,('device',socket.SOL_SOCKET,socket.SO_BINDTODEVICE,b'usb0\0'))
        self.assertLess(events.index(device),events.index(('source',('192.168.60.11',0))))
        self.assertLess(events.index(('source',('192.168.60.11',0))),events.index(('connect',('140.82.112.6',443))))
        self.assertIn(('tls',m.PROBE,ssl.TLSVersion.TLSv1_2,ssl.CERT_REQUIRED,True),events)
        request=next(e[1] for e in events if isinstance(e,tuple) and e[0]=='request')
        self.assertTrue(request.startswith(b'HEAD / HTTP/1.1\r\n'))
        self.assertIn(b'Host: api.github.com\r\n',request)
        self.assertIn('headers_only',events)

    def test_device_binding_failure_never_falls_back_to_other_network(self):
        code,out,events=self.execute(bind_failure=True)
        self.assertNotEqual(code,0)
        self.assertEqual(out.strip(),'BOUND_HTTPS_SOCKET_FAILED')
        self.assertFalse(any(isinstance(e,tuple) and e[0] in ('connect','request','tls') for e in events))

    def test_certificate_error_never_sends_http_request(self):
        code,out,events=self.execute(certificate_failure=True)
        self.assertNotEqual(code,0)
        self.assertEqual(out.strip(),'BOUND_HTTPS_CERTIFICATE_FAILED')
        self.assertFalse(any(isinstance(e,tuple) and e[0]=='request' for e in events))

    def test_private_remote_is_rejected_before_network_use(self):
        code,out,events=self.execute(remote='192.0.2.1')
        self.assertNotEqual(code,0)
        self.assertEqual(out.strip(),'BOUND_HTTPS_INPUT_OR_FEATURE_INVALID')
        self.assertEqual(events,[])


if __name__=='__main__':unittest.main()
