#!/usr/bin/env python3
"""Xiaoge data and vision service entry point."""

import json
import socket
import struct
import threading
import time
import traceback
from typing import Any

import openpilot.cereal.messaging as messaging
from opendbc.can import CANParser

from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper


TESLA_DAS_ROAD_ADDRESS = 605
TESLA_AUTOPILOT_PARTY_BUS = 2
TESLA_DAS_ROAD_TIMEOUT_S = 1.0


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
    self.tesla_das_road_updated_at = 0.0
    self.sm = messaging.SubMaster(["carState", "modelV2", "selfdriveState"])

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

  def collect_car_state(self, car_state) -> dict[str, Any]:
    return {
      "vEgo": max(float(car_state.vEgo), 0.0),
      "steeringAngleDeg": float(car_state.steeringAngleDeg),
      "leftLatDist": float(car_state.leftLatDist),
      "leftBlindspot": bool(car_state.leftBlindspot),
      "rightBlindspot": bool(car_state.rightBlindspot),
    }

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
    self.tesla_can_parser = CANParser("tesla_model3_party", [("DAS_road", float("nan"))], TESLA_AUTOPILOT_PARTY_BUS)
    self.tesla_can_sock = messaging.sub_sock("can")

  def collect_tesla_das_road(self) -> dict[str, Any] | None:
    self.initialize_tesla_can()
    if self.tesla_can_parser is None or self.tesla_can_sock is None:
      return None
    for can_event in messaging.drain_sock(self.tesla_can_sock):
      frames = [(frame.address, bytes(frame.dat), frame.src) for frame in can_event.can
                if frame.address == TESLA_DAS_ROAD_ADDRESS and frame.src == TESLA_AUTOPILOT_PARTY_BUS]
      if frames and TESLA_DAS_ROAD_ADDRESS in self.tesla_can_parser.update([can_event.logMonoTime, frames]):
        self.tesla_das_road_updated_at = time.monotonic()
    if time.monotonic() - self.tesla_das_road_updated_at > TESLA_DAS_ROAD_TIMEOUT_S:
      return None
    das_road = self.tesla_can_parser.vl["DAS_road"]
    return {
      "stopLineDist": float(das_road["DAS_stopLineDist"]),
      "trafficLightColor": int(das_road["DAS_trafficLightColor"]),
    }

  @staticmethod
  def collect_model_data(model_v2) -> dict[str, Any]:
    data: dict[str, Any] = {}
    if model_v2.leadsV3:
      lead = model_v2.leadsV3[0]
      data["lead0"] = {
        "x": float(lead.x[0]) if lead.x else 0.0,
        "y": float(lead.y[0]) if lead.y else 0.0,
        "v": float(lead.v[0]) if lead.v else 0.0,
        "prob": float(lead.prob),
      }
    else:
      data["lead0"] = {"x": 0.0, "y": 0.0, "v": 0.0, "prob": 0.0}
    data["laneLineProbs"] = [
      float(model_v2.laneLineProbs[1]) if len(model_v2.laneLineProbs) >= 3 else 0.0,
      float(model_v2.laneLineProbs[2]) if len(model_v2.laneLineProbs) >= 3 else 0.0,
    ]
    meta = model_v2.meta
    data["meta"] = {
      "distanceToRoadEdgeLeft": float(meta.distanceToRoadEdgeLeft),
      "distanceToRoadEdgeRight": float(meta.distanceToRoadEdgeRight),
    }
    if model_v2.orientationRate.z:
      data["curvature"] = {"maxOrientationRate": max((float(x) for x in model_v2.orientationRate.z), key=abs)}
    else:
      data["curvature"] = {"maxOrientationRate": 0.0}
    return data

  @staticmethod
  def collect_system_state(selfdrive_state) -> dict[str, bool]:
    return {"enabled": bool(selfdrive_state.enabled), "active": bool(selfdrive_state.active)}

  def create_packet(self, data: dict[str, Any]) -> bytes:
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
        self.sm.update(0)
        data: dict[str, Any] = {}
        if self.sm.alive["carState"]:
          data["carState"] = self.collect_car_state(self.sm["carState"])
          tesla_das_road = self.collect_tesla_das_road()
          if tesla_das_road is not None:
            data["carState"].update(tesla_das_road)
        if self.sm.alive["modelV2"]:
          data["modelV2"] = self.collect_model_data(self.sm["modelV2"])
        if self.sm.alive["selfdriveState"]:
          data["systemState"] = self.collect_system_state(self.sm["selfdriveState"])
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
