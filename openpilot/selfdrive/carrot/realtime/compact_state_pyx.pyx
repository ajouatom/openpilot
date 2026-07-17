# distutils: language = c++
# cython: language_level = 3

from cpython.bytes cimport PyBytes_FromStringAndSize
from libc.stddef cimport size_t
from libc.stdint cimport uint16_t
from libcpp.string cimport string


cdef extern from "selfdrive/carrot/realtime/compact_state_native.h":
  string encode_carrot_state_compact_frame(const char *service, size_t service_size,
                                           const char *data, size_t size, uint16_t sequence) except +


def encode_frame(str service, bytes payload, int sequence):
  cdef bytes encoded_service = service.encode("utf-8")
  cdef const char *service_data = encoded_service
  cdef const char *data = payload
  cdef string encoded = encode_carrot_state_compact_frame(service_data, len(encoded_service), data, len(payload),
                                                          <uint16_t>(sequence & 0xffff))
  return PyBytes_FromStringAndSize(encoded.data(), encoded.size())


def encode_model_v2(bytes payload, int sequence):
  return encode_frame("modelV2", payload, sequence)
