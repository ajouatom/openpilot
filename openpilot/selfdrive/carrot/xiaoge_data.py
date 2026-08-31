#!/usr/bin/env python3
"""
小鸽数据服务 / Xiaoge data service / 샤오거 데이터 서비스

- 中文：TCP 7711 广播实时车辆 JSON；HTTP 8888 提供 416x416 道路相机灰度 JPEG；
  UDP 4213 接收手机 App 的左右车道线结果；仅特斯拉读取 DBC 停止线和交通灯数据。
- English: TCP 7711 broadcasts live vehicle JSON; HTTP 8888 serves a 416x416 grayscale
  road-camera JPEG; UDP 4213 receives lane results from the mobile app; only Tesla reads
  stop-line and traffic-light data from the DBC.
- 한국어: TCP 7711은 실시간 차량 JSON을 전송하고, HTTP 8888은 416x416 도로 카메라
  흑백 JPEG를 제공하며, UDP 4213은 모바일 앱의 차선 결과를 수신합니다. DBC 정지선 및
  신호등 데이터는 Tesla 차량에서만 읽습니다.
"""
import json
import socket
import struct
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from typing import Dict, Any
from urllib.parse import urlsplit

import numpy as np
import openpilot.cereal.messaging as messaging
from opendbc.can import CANParser
from PIL import Image
from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.carrot.xiaoge_lane import parse_xiaoge_udp_payload


HTTP_PORT = 8888
UDP_HOST = "0.0.0.0"
UDP_PORT = 4213
TARGET_WIDTH = 416
TARGET_HEIGHT = 416
JPEG_QUALITY = 50
CLIENT_ACTIVE_TIMEOUT = 2.0
LANE_RESULT_TIMEOUT = 3.0
CAMERA_RETRY_DELAY = 1.0
IDLE_SLEEP = 0.1
ERROR_SLEEP = 0.05

TESLA_DAS_ROAD_ADDRESS = 605
TESLA_AUTOPILOT_PARTY_BUS = 2
TESLA_DAS_ROAD_TIMEOUT_S = 1.0


class XiaogeDataBroadcaster:

    def get_ip_address(self):
        """获取本机局域网 IP / Get the local LAN IP / 로컬 LAN IP 주소 가져오기."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def __init__(self):
        self.tcp_port = 7711
        self.sequence = 0
        self.device_ip = self.get_ip_address()

        # 管理 TCP 客户端连接 / Manage TCP client connections / TCP 클라이언트 연결 관리
        self.clients = {}
        self.clients_lock = threading.Lock()
        self.server_socket = None
        self.server_running = False

        self.http_server = None
        self.udp_lane_socket = None
        self.frame_lock = threading.Lock()
        self.latest_gray_jpeg = self.make_placeholder_jpeg()
        self.last_snapshot_time = 0.0
        self.lane_lock = threading.Lock()
        self.latest_lane_result = {
            "left_lane": -1,
            "right_lane": -1,
            "valid": False,
            "source_ip": "",
            "source_port": 0,
            "updated_at": 0.0,
        }

        self.params = Params()
        self.car_brand_checked = False
        self.is_tesla = False
        self.tesla_can_sock = None
        self.tesla_can_parser = None
        self.tesla_das_road_updated_at = 0.0

        # 订阅车辆消息 / Subscribe to vehicle messages / 차량 메시지 구독
        self.sm = messaging.SubMaster([
            'carState',
            'modelV2',
            'selfdriveState',
        ])
        self.pm = messaging.PubMaster(['customReservedRawData0'])

    def recvall(self, sock, n):
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def send_packet_to_client(self, conn, packet):
        try:
            size = len(packet)
            conn.sendall(struct.pack('!I', size))
            conn.sendall(packet)
            return True
        except (socket.error, OSError):
            return False

    def handle_client(self, conn, addr):
        print(f"Client connected from {addr}")
        with self.clients_lock:
            self.clients[addr] = conn
        try:
            while self.server_running:
                cmd_data = self.recvall(conn, 4)
                if not cmd_data:
                    break
                cmd = struct.unpack('!I', cmd_data)[0]
                if cmd == 2:  # 心跳 / Heartbeat / 하트비트
                    try:
                        conn.sendall(struct.pack('!I', 0))
                    except:
                        break
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
        finally:
            with self.clients_lock:
                self.clients.pop(addr, None)
            try:
                conn.close()
            except:
                pass
            print(f"Client {addr} disconnected")

    def start_tcp_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(5)
            self.server_running = True
            print(f"TCP server started, listening on port {self.tcp_port}")
            while self.server_running:
                try:
                    conn, addr = self.server_socket.accept()
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(conn, addr),
                        daemon=True
                    )
                    client_thread.start()
                except socket.error as e:
                    if self.server_running:
                        print(f"Error accepting connection: {e}")
                    break
        except Exception as e:
            print(f"TCP server error: {e}")
            traceback.print_exc()
        finally:
            self.server_running = False
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
            print("TCP server stopped")

    def broadcast_to_clients(self, packet):
        if not packet:
            return
        with self.clients_lock:
            clients_copy = dict(self.clients)
        dead_clients = []
        for addr, conn in clients_copy.items():
            if not self.send_packet_to_client(conn, packet):
                dead_clients.append(addr)
        if dead_clients:
            with self.clients_lock:
                for addr in dead_clients:
                    self.clients.pop(addr, None)
                    try:
                        if addr in clients_copy:
                            clients_copy[addr].close()
                    except:
                        pass

    def shutdown(self):
        print("Shutting down TCP server...")
        self.server_running = False
        http_server = self.http_server
        if http_server:
            http_server.shutdown()
            http_server.server_close()
        udp_lane_socket = self.udp_lane_socket
        if udp_lane_socket:
            udp_lane_socket.close()
        with self.clients_lock:
            for addr, conn in self.clients.items():
                try:
                    conn.close()
                except:
                    pass
            self.clients.clear()
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        print("TCP server shutdown complete")

    @staticmethod
    def lane_name(value):
        if value == 1:
            return "solid"
        if value == 0:
            return "dashed"
        return "unknown"

    @staticmethod
    def y_to_jpeg(buf, width, height, stride):
        """
        从 NV12 的 Y 平面生成固定尺寸灰度 JPEG。
        Generate a fixed-size grayscale JPEG directly from the NV12 Y plane.
        NV12 Y 평면에서 고정 크기 흑백 JPEG를 직접 생성합니다.
        """
        data = np.frombuffer(buf, dtype=np.uint8)
        if data.size < height * stride:
            raise ValueError(f"short Y plane: {data.size} < {height * stride}")

        y_plane = data[:height * stride].reshape(height, stride)[:, :width]
        crop_size = min(width, height)
        start_x = (width - crop_size) // 2
        start_y = (height - crop_size) // 2
        y_crop = np.ascontiguousarray(y_plane[start_y:start_y + crop_size, start_x:start_x + crop_size])

        output = BytesIO()
        image = Image.fromarray(y_crop)
        if image.size != (TARGET_WIDTH, TARGET_HEIGHT):
            image = image.resize((TARGET_WIDTH, TARGET_HEIGHT), Image.Resampling.BILINEAR)
        image.save(output, "JPEG", quality=JPEG_QUALITY)
        return output.getvalue()

    @staticmethod
    def make_placeholder_jpeg():
        output = BytesIO()
        Image.new("L", (TARGET_WIDTH, TARGET_HEIGHT)).save(output, "JPEG", quality=JPEG_QUALITY)
        return output.getvalue()

    def camera_thread(self):
        from msgq.visionipc import VisionIpcClient, VisionStreamType

        while self.server_running:
            vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
            while self.server_running and not vipc_client.connect(False):
                time.sleep(CAMERA_RETRY_DELAY)
            if not self.server_running:
                return

            print(f"[lane-image] connected: {vipc_client.width}x{vipc_client.height}, stride={vipc_client.stride}")
            while self.server_running:
                if time.monotonic() - self.last_snapshot_time > CLIENT_ACTIVE_TIMEOUT:
                    time.sleep(IDLE_SLEEP)
                    continue

                yuv_buf = vipc_client.recv()
                if yuv_buf is None:
                    time.sleep(ERROR_SLEEP)
                    break

                try:
                    jpeg = self.y_to_jpeg(yuv_buf.data, yuv_buf.width, yuv_buf.height, yuv_buf.stride)
                except (ValueError, OSError) as error:
                    print(f"[lane-image] Y->JPEG error: {error}")
                    time.sleep(ERROR_SLEEP)
                    continue

                with self.frame_lock:
                    self.latest_gray_jpeg = jpeg

    def road_gray_jpeg(self):
        self.last_snapshot_time = time.monotonic()
        with self.frame_lock:
            return self.latest_gray_jpeg

    def lane_result_snapshot(self):
        with self.lane_lock:
            result = dict(self.latest_lane_result)
        result["age_sec"] = time.monotonic() - result["updated_at"] if result["updated_at"] > 0 else None
        result["active"] = bool(
            result["valid"] and result["age_sec"] is not None and result["age_sec"] <= LANE_RESULT_TIMEOUT
        )
        result["left_name"] = self.lane_name(result["left_lane"])
        result["right_name"] = self.lane_name(result["right_lane"])
        return result

    def publish_lane_result(self, result):
        lane_payload = {
            "type": "lane",
            "version": 1,
            "leftLine": result["left_lane"],
            "rightLine": result["right_lane"],
            "lineValid": result["active"],
            "leftBlind": 8 if result["active"] and result["left_lane"] >= 1 else 0,
            "rightBlind": 8 if result["active"] and result["right_lane"] >= 1 else 0,
            "receivedMonoTimeNanos": int(result["updated_at"] * 1e9),
        }
        payload = json.dumps(lane_payload, separators=(",", ":")).encode("utf-8")
        msg = messaging.new_message('customReservedRawData0', size=len(payload), valid=True)
        msg.customReservedRawData0 = payload
        self.pm.send('customReservedRawData0', msg)

    def start_http_server(self):
        broadcaster = self

        class LaneHttpServer(ThreadingHTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        class LaneHttpHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlsplit(self.path).path
                if path == "/roadgray.jpg":
                    self.send_payload(broadcaster.road_gray_jpeg(), "image/jpeg", no_cache=True)
                elif path == "/lane.json":
                    payload = json.dumps(broadcaster.lane_result_snapshot(), ensure_ascii=False).encode("utf-8")
                    self.send_payload(payload, "application/json; charset=utf-8", no_cache=True)
                else:
                    self.send_error(404)

            def send_payload(self, payload, content_type, no_cache=False):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Access-Control-Allow-Origin", "*")
                if no_cache:
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                    self.send_header("Pragma", "no-cache")
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, fmt, *args):
                return

        server = None
        try:
            server = LaneHttpServer((self.device_ip, HTTP_PORT), LaneHttpHandler)
            self.http_server = server
            print(f"[lane-image] HTTP listening on {self.device_ip}:{HTTP_PORT}")
            server.serve_forever(poll_interval=0.2)
        except OSError as error:
            print(f"[lane-image] HTTP server error: {error}")
        finally:
            if server:
                server.server_close()
            if self.http_server is server:
                self.http_server = None

    def udp_lane_receiver_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_lane_socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((UDP_HOST, UDP_PORT))
            sock.settimeout(1.0)
            print(f"[lane-udp] listening on {UDP_HOST}:{UDP_PORT}")

            while self.server_running:
                try:
                    data, addr = sock.recvfrom(4096)
                except TimeoutError:
                    continue

                try:
                    left_lane, right_lane = parse_xiaoge_udp_payload(data)
                except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as error:
                    print(f"[lane-udp] invalid result from {addr[0]}:{addr[1]}: {error}")
                    continue

                result = {
                    "left_lane": left_lane,
                    "right_lane": right_lane,
                    "valid": True,
                    "source_ip": addr[0],
                    "source_port": addr[1],
                    "updated_at": time.monotonic(),
                }
                with self.lane_lock:
                    self.latest_lane_result.update(result)

                message = f"[lane-udp] left={left_lane}({self.lane_name(left_lane)}), "
                message += f"right={right_lane}({self.lane_name(right_lane)}), from={addr[0]}:{addr[1]}"
                print(message)
        except OSError as error:
            if self.server_running:
                print(f"[lane-udp] receiver error: {error}")
        finally:
            sock.close()
            if self.udp_lane_socket is sock:
                self.udp_lane_socket = None

    def collect_car_state(self, carState) -> Dict[str, Any]:
        vEgo = float(carState.vEgo)
        if vEgo < 0:
            vEgo = 0.0
        return {
            'vEgo': vEgo,
            'steeringAngleDeg': float(carState.steeringAngleDeg),
            'leftLatDist': float(carState.leftLatDist),
            'leftBlindspot': bool(carState.leftBlindspot) if hasattr(carState, 'leftBlindspot') else False,
            'rightBlindspot': bool(carState.rightBlindspot) if hasattr(carState, 'rightBlindspot') else False,
        }

    def initialize_tesla_can(self):
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

        self.tesla_can_parser = CANParser(
            "tesla_model3_party",
            [("DAS_road", float("nan"))],
            TESLA_AUTOPILOT_PARTY_BUS,
        )
        self.tesla_can_sock = messaging.sub_sock("can")

    def collect_tesla_das_road(self) -> dict[str, Any] | None:
        self.initialize_tesla_can()
        if self.tesla_can_parser is None or self.tesla_can_sock is None:
            return None

        for can_event in messaging.drain_sock(self.tesla_can_sock):
            frames = [
                (frame.address, bytes(frame.dat), frame.src)
                for frame in can_event.can
                if frame.address == TESLA_DAS_ROAD_ADDRESS and frame.src == TESLA_AUTOPILOT_PARTY_BUS
            ]
            if frames and TESLA_DAS_ROAD_ADDRESS in self.tesla_can_parser.update([can_event.logMonoTime, frames]):
                self.tesla_das_road_updated_at = time.monotonic()

        if time.monotonic() - self.tesla_das_road_updated_at > TESLA_DAS_ROAD_TIMEOUT_S:
            return None

        das_road = self.tesla_can_parser.vl["DAS_road"]
        return {
            "stopLineDist": float(das_road["DAS_stopLineDist"]),
            "trafficLightColor": int(das_road["DAS_trafficLightColor"]),
        }

    def collect_model_data(self, modelV2) -> Dict[str, Any]:
        """
        收集精简模型数据，只输出必要字段。
        Collect a compact subset of required model fields.
        필요한 모델 필드만 간결하게 수집합니다.
        """
        data = {}

        # 前车 / Lead vehicle / 선행 차량
        if len(modelV2.leadsV3) > 0:
            lead = modelV2.leadsV3[0]
            x = float(lead.x[0]) if len(lead.x) > 0 else 0.0
            y = float(lead.y[0]) if len(lead.y) > 0 else 0.0
            v = float(lead.v[0]) if len(lead.v) > 0 else 0.0
            data['lead0'] = {'x': x, 'y': y, 'v': v, 'prob': float(lead.prob)}
        else:
            data['lead0'] = {'x': 0.0, 'y': 0.0, 'v': 0.0, 'prob': 0.0}

        # 车道线置信度 / Lane-line probabilities / 차선 신뢰도
        data['laneLineProbs'] = [
            float(modelV2.laneLineProbs[1]) if len(modelV2.laneLineProbs) >= 3 else 0.0,
            float(modelV2.laneLineProbs[2]) if len(modelV2.laneLineProbs) >= 3 else 0.0,
        ]

        # 道路边缘距离 / Distance to road edges / 도로 가장자리까지의 거리
        meta = modelV2.meta
        data['meta'] = {
            'distanceToRoadEdgeLeft': float(meta.distanceToRoadEdgeLeft),
            'distanceToRoadEdgeRight': float(meta.distanceToRoadEdgeRight),
        }

        # 曲率 / Curvature / 곡률
        if hasattr(modelV2, 'orientationRate') and len(modelV2.orientationRate.z) > 0:
            orz = [float(x) for x in modelV2.orientationRate.z]
            data['curvature'] = {'maxOrientationRate': max(orz, key=abs)}
        else:
            data['curvature'] = {'maxOrientationRate': 0.0}

        return data

    def collect_system_state(self, selfdriveState) -> Dict[str, Any]:
        return {
            'enabled': bool(selfdriveState.enabled) if selfdriveState else False,
            'active': bool(selfdriveState.active) if selfdriveState else False,
        }

    def create_packet(self, data: Dict[str, Any]) -> bytes:
        packet_data = {
            'version': 1,
            'sequence': self.sequence,
            'timestamp': time.time(),
            'ip': self.device_ip,
            'data': data
        }
        json_str = json.dumps(packet_data)
        packet_bytes = json_str.encode('utf-8')
        if len(packet_bytes) > 1024 * 1024:
            print(f"Warning: Large packet size {len(packet_bytes)} bytes")
        return packet_bytes

    def broadcast_data(self):
        """
        主循环：收集数据并通过 TCP 推送给所有客户端。
        Main loop: collect data and send it to every TCP client.
        메인 루프: 데이터를 수집하여 모든 TCP 클라이언트에 전송합니다.
        """
        rk = Ratekeeper(20, print_delay_threshold=None)  # 20 Hz 更新率 / update rate / 갱신 주기

        self.server_running = True
        for target in (
            self.start_tcp_server,
            self.start_http_server,
            self.udp_lane_receiver_thread,
            self.camera_thread,
        ):
            threading.Thread(target=target, daemon=True).start()
        time.sleep(0.5)
        print(f"XiaogeDataBroadcaster started, TCP server listening on port {self.tcp_port}")

        try:
            while True:
                try:
                    self.sm.update(0)
                    data = {}

                    if self.sm.alive['carState']:
                        data['carState'] = self.collect_car_state(self.sm['carState'])
                        tesla_das_road = self.collect_tesla_das_road()
                        if tesla_das_road is not None:
                            data['carState'].update(tesla_das_road)

                    if self.sm.alive['modelV2']:
                        data['modelV2'] = self.collect_model_data(self.sm['modelV2'])

                    if self.sm.alive['selfdriveState']:
                        data['systemState'] = self.collect_system_state(self.sm['selfdriveState'])

                    lane_result = self.lane_result_snapshot()
                    self.publish_lane_result(lane_result)
                    if lane_result["active"]:
                        data['laneResult'] = lane_result

                    if data:
                        packet = self.create_packet(data)
                        try:
                            self.broadcast_to_clients(packet)
                            self.sequence += 1
                            if self.sequence % 100 == 0:
                                with self.clients_lock:
                                    client_count = len(self.clients)
                                print(f"Sent {self.sequence} packets to {client_count} clients, last size: {len(packet)} bytes")
                        except Exception as e:
                            print(f"Failed to send packet to clients: {e}")
                    else:
                        # 心跳包 / Heartbeat packet / 하트비트 패킷
                        try:
                            heartbeat = {
                                'version': 1, 'sequence': self.sequence,
                                'timestamp': time.time(), 'ip': self.device_ip,
                                'data': {}
                            }
                            self.broadcast_to_clients(json.dumps(heartbeat).encode('utf-8'))
                            self.sequence += 1
                        except Exception:
                            pass

                    rk.keep_time()

                except KeyboardInterrupt:
                    print("\nReceived shutdown signal, closing gracefully...")
                    break
                except Exception as e:
                    print(f"XiaogeDataBroadcaster error: {e}")
                    traceback.print_exc()
                    time.sleep(1)
        finally:
            self.shutdown()


def main():
    broadcaster = XiaogeDataBroadcaster()
    broadcaster.broadcast_data()


if __name__ == "__main__":
    main()
