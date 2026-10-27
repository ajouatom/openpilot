#!/usr/bin/env python3
"""Xiaoge data and vision service entry point."""

import json
import socket
import struct
import threading
import time
import traceback

import openpilot.cereal.messaging as messaging
from opendbc.can import CANParser
from opendbc.car.carlog import carlog

from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper


TESLA_DAS_ROAD_ADDRESS = 605
TESLA_AUTOPILOT_PARTY_BUS = 2
TESLA_DAS_ROAD_TIMEOUT_NS = 1_000_000_000


class XiaogeDataBroadcaster:
  def __init__(self):
    self.tcp_port = 7711
    self.sequence = 0
    self.device_ip = self.get_ip_address()
    self.clients: dict[tuple[str, int], socket.socket] = {}
    self.clients_lock = threading.Lock()
    self.server_socket: socket.socket | None = None
    self.server_running = False
    self.vision_service = None
    self.vision_server = None

    self.params = Params()
    self.car_brand_checked = False
    self.is_tesla = False
    self.tesla_can_sock = None
    self.tesla_can_parser = None
    self.tesla_das_road_updated_at = 0
    self.tesla_traffic_status: str | None = None

  @staticmethod
  def get_ip_address() -> str:
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
      return "127.0.0.1"

  @staticmethod
  def recvall(sock: socket.socket, length: int) -> bytes | None:
    data = bytearray()
    while len(data) < length:
      packet = sock.recv(length - len(data))
      if not packet:
        return None
      data.extend(packet)
    return bytes(data)

  def send_packet_to_client(self, conn: socket.socket, packet: bytes) -> bool:
    try:
      conn.sendall(struct.pack("!I", len(packet)))
      conn.sendall(packet)
      return True
    except OSError:
      return False

  def handle_client(self, conn: socket.socket, addr: tuple[str, int]) -> None:
    print(f"Client connected from {addr}")
    with self.clients_lock:
      self.clients[addr] = conn
    try:
      while self.server_running:
        command = self.recvall(conn, 4)
        if not command:
          break
        if struct.unpack("!I", command)[0] == 2:
          conn.sendall(struct.pack("!I", 0))
    except OSError:
      pass
    finally:
      with self.clients_lock:
        self.clients.pop(addr, None)
      conn.close()
      print(f"Client {addr} disconnected")

  def start_tcp_server(self) -> None:
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        self.server_socket = server_socket
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(("0.0.0.0", self.tcp_port))
        server_socket.listen(5)
        print(f"Xiaoge data TCP server listening on port {self.tcp_port}")
        while self.server_running:
          try:
            conn, addr = server_socket.accept()
          except OSError:
            if self.server_running:
              raise
            break
          threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
    except OSError as error:
      if self.server_running:
        print(f"Xiaoge data TCP server error: {error}")
    finally:
      self.server_socket = None

  def start_vision_server(self) -> None:
    from openpilot.selfdrive.carrot.xiaoge.v_asm_server import create_server

    self.vision_service, self.vision_server = create_server()
    print("Xiaoge vision server listening on port 8082")
    self.vision_server.serve_forever()

  def broadcast_to_clients(self, packet: bytes) -> None:
    with self.clients_lock:
      clients = dict(self.clients)
    for addr, conn in clients.items():
      if not self.send_packet_to_client(conn, packet):
        with self.clients_lock:
          self.clients.pop(addr, None)
        conn.close()

  def shutdown(self) -> None:
    self.server_running = False
    if self.vision_service is not None:
      self.vision_service.running = False
    if self.vision_server is not None:
      self.vision_server.shutdown()
      self.vision_server.server_close()
    if self.server_socket is not None:
      self.server_socket.close()
    with self.clients_lock:
      for conn in self.clients.values():
        conn.close()
      self.clients.clear()

  def initialize_tesla_can(self) -> None:
    if self.car_brand_checked:
      return
    car_params = self.params.get("CarParams")
    if car_params is None:
      return
    CP = messaging.log_from_bytes(car_params, car.CarParams)
    self.car_brand_checked = True
    self.is_tesla = CP.brand == "tesla"
    if not self.is_tesla:
      return
    self.tesla_can_parser = CANParser("tesla_modely_hw4_perception", [("APP_trafficControl", float("nan"))], TESLA_AUTOPILOT_PARTY_BUS)
    self.tesla_can_sock = messaging.sub_sock("can")

  def _set_tesla_traffic_status(self, status: str) -> None:
    if status != self.tesla_traffic_status:
      if status == "valid":
        carlog.info("Tesla traffic data available")
      else:
        carlog.warning(f"Tesla traffic data unavailable: {status}")
      self.tesla_traffic_status = status

  def collect_tesla_das_road(self) -> dict[str, float | int] | None:
    self.initialize_tesla_can()
    if self.tesla_can_parser is None or self.tesla_can_sock is None:
      return None
    now = time.clock_gettime_ns(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))
    for can_event in messaging.drain_sock(self.tesla_can_sock):
      timestamp = can_event.logMonoTime
      if timestamp <= 0 or not 0 <= now - timestamp <= TESLA_DAS_ROAD_TIMEOUT_NS:
        continue
      if timestamp < self.tesla_das_road_updated_at:
        continue
      for frame in can_event.can:
        if frame.address != TESLA_DAS_ROAD_ADDRESS or frame.src != TESLA_AUTOPILOT_PARTY_BUS:
          continue
        self.tesla_das_road_updated_at = timestamp
        data = bytes(frame.dat)
        if not 6 <= len(data) <= 8:
          self._set_tesla_traffic_status("invalid_dlc")
          continue
        if TESLA_DAS_ROAD_ADDRESS not in self.tesla_can_parser.update([(timestamp, [(frame.address, data, frame.src)])]):
          self._set_tesla_traffic_status("decode_failed")
          continue
        values = self.tesla_can_parser.vl["APP_trafficControl"]
        valid = (values["APP_tcControlType"] == 3 and values["APP_tcControlLightState"] in (0, 1, 2, 3) and
                 0 <= values["APP_tcControlDistance"] <= 200)
        self._set_tesla_traffic_status("valid" if valid else "invalid_traffic_observation")
    if not self.tesla_das_road_updated_at or not 0 <= now - self.tesla_das_road_updated_at <= TESLA_DAS_ROAD_TIMEOUT_NS:
      self._set_tesla_traffic_status("stale")
      return None
    if self.tesla_traffic_status != "valid":
      return None
    traffic = self.tesla_can_parser.vl["APP_trafficControl"]
    return {
      "stopLineDist": float(traffic["APP_tcControlDistance"]),
      "trafficLightColor": int(traffic["APP_tcControlLightState"]),
    }

  def collect_broadcast_data(self) -> dict[str, dict[str, float | int]] | None:
    tesla_das_road = self.collect_tesla_das_road()
    if tesla_das_road is None:
      return None
    return {"carState": tesla_das_road}

  def create_packet(self, data: dict[str, dict[str, float | int]]) -> bytes:
    return json.dumps({
      "version": 1,
      "sequence": self.sequence,
      "timestamp": time.monotonic(),
      "ip": self.device_ip,
      "data": data,
    }).encode()

  def broadcast_data(self) -> None:
    self.server_running = True
    threading.Thread(target=self.start_tcp_server, daemon=True).start()
    threading.Thread(target=self.start_vision_server, daemon=True).start()
    rk = Ratekeeper(20, print_delay_threshold=None)
    try:
      while self.server_running:
        data = self.collect_broadcast_data()
        if data is not None:
          self.broadcast_to_clients(self.create_packet(data))
          self.sequence += 1
        rk.keep_time()
    except KeyboardInterrupt:
      pass
    except Exception as error:
      print(f"XiaogeDataBroadcaster error: {error}")
      traceback.print_exc()
    finally:
      self.shutdown()


def main() -> None:
  XiaogeDataBroadcaster().broadcast_data()


if __name__ == "__main__":
  main()
