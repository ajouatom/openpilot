#!/usr/bin/env python3
"""
小鸽数据广播模块
从系统获取实时数据，通过TCP连接传输到7711端口
"""

import json
import socket
import struct
import threading
import time
import traceback
from typing import Dict, Any, List, Tuple

import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import PC


class XiaogeDataBroadcaster:
    # 常量定义（参考 radard.py:28）
    RADAR_TO_CAMERA = 1.52  # 雷达相对于相机中心的偏移（米）
    RADAR_LAT_FACTOR = 0.5  # 未来位置预测时间因子（秒），参考 radard.py 的 radar_lat_factor
    FILTER_INIT_FRAMES = 3  # 滤波器初始化所需的最小帧数（参考 radard.py:520-546 的 cnt > 3）

    # 优化：车道分类和检测阈值（参考 radard.py:520-546）
    LANE_PROB_THRESHOLD = 0.1  # 车道内概率阈值，用于区分当前车道和侧方车道（参考 radard.py:520）
    CUTIN_PROB_THRESHOLD = 0.1  # Cut-in 检测的车道内概率阈值（参考 radard.py:520）

    # 优化：历史数据配置
    HISTORY_SIZE = 10  # 历史数据保留帧数，用于计算横向速度

    # 优化：动态置信度阈值参数（参考 radard.py:126-157 的匹配逻辑）
    CONFIDENCE_BASE_THRESHOLD = 0.5  # 基础置信度阈值
    CONFIDENCE_DISTANCE_THRESHOLD = 50.0  # 距离阈值（米），超过此距离要求更高置信度
    CONFIDENCE_DISTANCE_BOOST = 0.7  # 距离超过阈值时的置信度提升
    CONFIDENCE_VELOCITY_DIFF_THRESHOLD = 10.0  # 速度差异阈值（m/s）
    CONFIDENCE_VELOCITY_BOOST = 0.6  # 速度差异超过阈值时的置信度提升

    # 优化：侧方车辆筛选参数（参考 radard.py:560-569）
    SIDE_VEHICLE_MIN_DISTANCE = 5.0  # 侧方车辆最小距离（米）
    SIDE_VEHICLE_MAX_DPATH = 3.5  # 侧方车辆最大路径偏移（米）

    # 优化：车道宽度计算参数
    DEFAULT_LANE_HALF_WIDTH = 1.75  # 默认车道半宽 3.5m / 2
    MIN_LANE_HALF_WIDTH = 0.1  # 最小车道半宽阈值（避免除零）
    TARGET_LANE_WIDTH_DISTANCE = 20.0  # 车道宽度计算的目标距离（米）

    def get_ip_address(self):
        """获取本机局域网IP地址"""
        try:
            # 创建一个UDP socket连接到外部地址（不需要实际连接成功）
            # 这样可以自动选择正确的网络接口IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def __init__(self):
        self.tcp_port = 7711  # TCP 端口号
        self.sequence = 0
        self.device_ip = self.get_ip_address()  # 获取本机IP

        # TCP 客户端连接管理（线程安全）
        self.clients = {}  # {addr: conn}  存储活跃的客户端连接
        self.clients_lock = threading.Lock()  # 保护客户端列表的锁
        self.server_socket = None  # TCP 服务器 socket
        self.server_running = False  # 服务器运行状态标志

        # 订阅消息（纯视觉数据，不使用雷达）
        self.sm = messaging.SubMaster([
            'carState',
            'modelV2',
            'selfdriveState',
            # 移除 'controlsState' - 不再需要 longControlState
            # 移除 'can' - 盲区数据直接从carState获取
            # 移除 'radarState' - 纯视觉方案，不使用雷达融合数据
        ])

        # 时间滤波：用于平滑侧方车辆数据（指数移动平均）
        # alpha 值：0.3 表示新数据权重30%，历史数据权重70%
        self.filter_alpha = 0.3
        self.lead_left_filtered = {'x': 0.0, 'v': 0.0, 'y': 0.0, 'vRel': 0.0, 'dPath': 0.0, 'yRel': 0.0}
        self.lead_right_filtered = {'x': 0.0, 'v': 0.0, 'y': 0.0, 'vRel': 0.0, 'dPath': 0.0, 'yRel': 0.0}
        self.lead_left_count = 0  # 连续检测计数（用于滤波器初始化）
        self.lead_right_count = 0

        # 历史数据缓存：用于计算横向速度（yvRel）和滤波器初始化
        # 存储最近几帧的 yRel 和 dRel，用于计算横向速度
        self.lead_left_history: List[Dict[str, float]] = []  # 存储 {'yRel': float, 'dRel': float, 'timestamp': float}
        self.lead_right_history: List[Dict[str, float]] = []

        # 车道线数据缓存：避免重复计算
        # 修复：添加 position_valid 字段，缓存规划路径单调性验证结果
        self._lane_cache = {
            'lane_xs': None,
            'left_ys': None,
            'right_ys': None,
            'position_x': None,
            'position_y': None,
            'position_valid': False,  # 新增：缓存规划路径单调性验证结果
            'cache_valid': False
        }

    def recvall(self, sock, n):
        """
        接收指定字节数的数据（TCP 需要确保接收完整数据）
        参考 carrot_man.py:765-773 的实现
        参数:
        - sock: socket 对象
        - n: 需要接收的字节数
        返回: 接收到的数据（bytearray），如果连接关闭则返回 None
        """
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:  # 杩炴帴宸插叧闂�
                return None
            data.extend(packet)
        return data

    def send_packet_to_client(self, conn, packet):
        """
        向单个客户端发送数据包（TCP 需要确保数据完整发送）
        参数:
        - conn: 客户端连接对象
        - packet: 要发送的数据包（bytes）
        返回: 是否发送成功（bool）
        """
        try:
            # TCP 发送数据包格式: [数据长度(4字节)][数据]
            # 先发送数据长度（网络字节序，big-endian）
            size = len(packet)
            conn.sendall(struct.pack('!I', size))
            # 再发送实际数据
            conn.sendall(packet)
            return True
        except (socket.error, OSError):
            # 杩炴帴宸叉柇寮€鎴栧彂閫佸け璐�
            return False

    def handle_client(self, conn, addr):
        """
        处理单个客户端连接
        支持客户端发送命令：
        - CMD 2: 心跳包，回复 0 表示存活
        """
        print(f"Client connected from {addr}")

        # 将客户端添加到连接列表（线程安全）
        with self.clients_lock:
            self.clients[addr] = conn

        try:
            while self.server_running:
                # 接收客户端请求(4字节命令)
                # 如果客户端只是接收数据不发送命令，这里会阻塞，这是正常的
                # 只要不抛出异常，连接就保持着，主线程可以继续通过 broadcast_to_clients 发送数据
                cmd_data = self.recvall(conn, 4)

                if not cmd_data:
                    break

                cmd = struct.unpack('!I', cmd_data)[0]

                if cmd == 2:  # 心跳请求
                    # 响应心跳：发送大小为0的数据包
                    try:
                        conn.sendall(struct.pack('!I', 0))
                    except (socket.error, OSError):
                        break
                # 可以扩展其他命令，例如请求特定数据
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
        finally:
            # 清理客户端连接
            with self.clients_lock:
                self.clients.pop(addr, None)
            try:
                conn.close()
            except:
                pass
            print(f"Client {addr} disconnected")

    def start_tcp_server(self):
        """
        启动 TCP 服务器（在独立线程中运行）
        参考 carrot_man.py:809-878 的 carrot_route() 实现
        """
        try:
            # 创建 TCP socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 设置 SO_REUSEADDR 选项，允许端口重用
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 绑定到所有网络接口的指定端口
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            # 开始监听连接（最大 5 个待处理连接）
            self.server_socket.listen(5)

            self.server_running = True
            print(f"TCP server started, listening on port {self.tcp_port}")

            while self.server_running:
                try:
                    # 等待客户端连接（阻塞调用）
                    conn, addr = self.server_socket.accept()
                    # 为每个客户端创建独立线程处理连接
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(conn, addr),
                        daemon=True  # 设置为守护线程，主程序退出时自动结束
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
        """
        向所有连接的客户端广播数据包
        参数:
        - packet: 要发送的数据包（bytes）
        """
        if not packet:
            return

        # 线程安全地获取客户端列表副本
        with self.clients_lock:
            clients_copy = dict(self.clients)  # 创建副本，避免在迭代时修改原字典

        # 记录需要清理的断开连接
        dead_clients = []

        # 向所有客户端发送数据
        for addr, conn in clients_copy.items():
            if not self.send_packet_to_client(conn, packet):
                # 发送失败，标记为断开连接
                dead_clients.append(addr)

        # 清理断开的连接
        if dead_clients:
            with self.clients_lock:
                for addr in dead_clients:
                    self.clients.pop(addr, None)
                    try:
                        # 尝试关闭连接（如果还未关闭）
                        if addr in clients_copy:
                            clients_copy[addr].close()
                    except:
                        pass

    def shutdown(self):
        """
        优雅关闭服务器
        关闭所有客户端连接并停止服务器
        """
        print("Shutting down TCP server...")

        # 停止服务器运行标志
        self.server_running = False

        # 关闭所有客户端连接（线程安全）
        with self.clients_lock:
            for addr, conn in self.clients.items():
                try:
                    conn.close()
                    print(f"Closed connection to {addr}")
                except:
                    pass
            self.clients.clear()

        # 关闭服务器 socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        print("TCP server shutdown complete")

    def collect_car_state(self, carState) -> Dict[str, Any]:
        """收集本车状态数据 - 简化版（只保留超车决策必需字段）
        """
        # 数据验证：确保 vEgo 为有效值
        vEgo = float(carState.vEgo)
        if vEgo < 0:
            print(f"Warning: Invalid vEgo value: {vEgo}, using 0.0")
            vEgo = 0.0

        return {
            'vEgo': vEgo,  # 实际速度
            'steeringAngleDeg': float(carState.steeringAngleDeg),  # 方向盘角度
            'leftLatDist': float(carState.leftLatDist),  # 车道距离（返回原车道）
            'leftBlindspot': bool(carState.leftBlindspot) if hasattr(carState, 'leftBlindspot') else False,  # 左盲区
            'rightBlindspot': bool(carState.rightBlindspot) if hasattr(carState, 'rightBlindspot') else False,  # 右盲区
        }

    def _update_lane_cache(self, modelV2):
        """更新车道线数据缓存，避免重复计算"""
        try:
            # 修复：需要至少 3 条线才能访问索引 1 和 2
            if not hasattr(modelV2, 'laneLines') or len(modelV2.laneLines) < 3:
                self._lane_cache['cache_valid'] = False
                return

            # 修复：验证索引 1 和 2 是否存在（需要至少 3 个元素才能访问索引 2）
            if len(modelV2.laneLines) <= 2:
                self._lane_cache['cache_valid'] = False
                return

            # 提取车道线数据
            lane_xs = [float(x) for x in modelV2.laneLines[1].x]
            left_ys = [float(y) for y in modelV2.laneLines[1].y]
            right_ys = [float(y) for y in modelV2.laneLines[2].y]

            # 修复：验证 x 坐标是否单调递增（np.interp() 要求）
            if not (len(lane_xs) == len(left_ys) == len(right_ys)):
                self._lane_cache['cache_valid'] = False
                return

            # 修复：验证 x 坐标是否单调递增（np.interp() 要求）
            if len(lane_xs) < 2 or not all(lane_xs[i] < lane_xs[i + 1] for i in range(len(lane_xs) - 1)):
                self._lane_cache['cache_valid'] = False
                return

            self._lane_cache['lane_xs'] = lane_xs
            self._lane_cache['left_ys'] = left_ys
            self._lane_cache['right_ys'] = right_ys

            # 更新规划路径数据
            # 修复：在缓存更新时验证单调性，并缓存验证结果，避免在 d_path_interp() 中重复检查
            if hasattr(modelV2, 'position') and len(modelV2.position.x) > 0:
                position_x = [float(x) for x in modelV2.position.x]
                position_y = [float(y) for y in modelV2.position.y]

                # 验证规划路径数据长度一致性和单调性，并缓存验证结果
                if len(position_x) == len(position_y) and len(position_x) >= 2:
                    # 验证 x 坐标单调递增（只验证一次，结果缓存到 position_valid）
                    if all(position_x[i] < position_x[i + 1] for i in range(len(position_x) - 1)):
                        self._lane_cache['position_x'] = position_x
                        self._lane_cache['position_y'] = position_y
                        self._lane_cache['position_valid'] = True  # 缓存验证结果
                    else:
                        self._lane_cache['position_x'] = None
                        self._lane_cache['position_y'] = None
                        self._lane_cache['position_valid'] = False
                else:
                    self._lane_cache['position_x'] = None
                    self._lane_cache['position_y'] = None
                    self._lane_cache['position_valid'] = False
            else:
                self._lane_cache['position_x'] = None
                self._lane_cache['position_y'] = None
                self._lane_cache['position_valid'] = None

            self._lane_cache['cache_valid'] = (
                len(self._lane_cache['lane_xs']) > 0 and
                len(self._lane_cache['left_ys']) > 0 and
                len(self._lane_cache['right_ys']) > 0
            )
        except (IndexError, AttributeError, ValueError):
            # 修复：使用具体的异常类型
            self._lane_cache['cache_valid'] = False

    def _calculate_dpath(self, dRel: float, yRel: float, yvRel: float = 0.0, vLead: float = 0.0) -> Tuple[float, float, float]:
        """
        计算车辆相对于规划路径的横向偏移 (dPath) 和车道内概率 (in_lane_prob)
        参考 radard.py:74-87 的 d_path() 方法

        参数:
        - dRel: 相对于雷达的距离（已考虑 RADAR_TO_CAMERA 偏移）
        - yRel: 相对于相机的横向位置
        - yvRel: 横向速度（用于未来位置预测，可选）
        - vLead: 前车速度（用于未来位置预测，可选）

        返回: (dPath, in_lane_prob, in_lane_prob_future)
        - dPath: 相对于规划路径的横向偏移
        - in_lane_prob: 当前时刻在车道内的概率
        - in_lane_prob_future: 未来时刻在车道内的概率（用于 Cut-in 检测）
        """
        if not self._lane_cache['cache_valid']:
            return 0.0, 0.0, 0.0

        try:
            # 优化：移除重复的单调性检查，因为 cache_valid 已经保证了数据的有效性
            # 单调性验证已在 _update_lane_cache() 中完成
            lane_xs = self._lane_cache['lane_xs']
            left_ys = self._lane_cache['left_ys']
            right_ys = self._lane_cache['right_ys']

            def d_path_interp(dRel_val: float, yRel_val: float) -> Tuple[float, float]:
                """内部函数：计算指定距离处的 dPath 和 in_lane_prob"""
                # 在距离 dRel_val 处插值计算左右车道线的横向位置
                left_lane_y = np.interp(dRel_val, lane_xs, left_ys)
                right_lane_y = np.interp(dRel_val, lane_xs, right_ys)

                # 计算车道中心位置
                center_y = (left_lane_y + right_lane_y) / 2.0

                # 计算车道半宽
                # 优化：使用类常量替代魔法数字
                lane_half_width = abs(right_lane_y - left_lane_y) / 2.0
                if lane_half_width < self.MIN_LANE_HALF_WIDTH:
                    lane_half_width = self.DEFAULT_LANE_HALF_WIDTH

                # 修复：使用正确的符号计算相对于车道中心的偏移
                # yRel_val 和 center_y 都是相对于相机的，所以相减得到相对于车道中心的偏移
                dist_from_center = yRel_val - center_y

                # 计算在车道内的概率（距离中心越近，概率越高）
                # 参考 radard.py:82 的计算方法
                in_lane_prob = max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width))

                # 计算 dPath（相对于规划路径的横向偏移）
                # 修复：使用缓存的验证结果，避免重复的单调性检查（性能优化）
                # 单调性验证已在 _update_lane_cache() 中完成并缓存到 position_valid
                if self._lane_cache.get('position_valid', False):
                    path_y = np.interp(dRel_val, self._lane_cache['position_x'], self._lane_cache['position_y'])
                    # 修复：同样修复符号， dPath = yRel - path_y
                    dPath = yRel_val - path_y
                else:
                    dPath = dist_from_center

                return dPath, in_lane_prob

            # 计算当前时刻的值
            dPath, in_lane_prob = d_path_interp(dRel, yRel)

            # 计算未来时刻的值(用于 Cut-in 检测）
            # 参考 radard.py:30-72 的 Track.update() 方法
            # yRel_future = yRel + yvLead * radar_lat_factor
            # dRel_future = dRel + vLead * radar_lat_factor
            future_dRel = dRel + vLead * self.RADAR_LAT_FACTOR
            future_yRel = yRel + yvRel * self.RADAR_LAT_FACTOR
            _, in_lane_prob_future = d_path_interp(future_dRel, future_yRel)

            return float(dPath), float(in_lane_prob), float(in_lane_prob_future)

        except (IndexError, ValueError, TypeError):
            # 修复：使用具体的异常类型
            # 调试信息（可选）
            # print(f"Error in _calculate_dpath: {e}")
            return 0.0, 0.0, 0.0

    def _estimate_lateral_velocity(self, current_yRel: float, current_dRel: float, history: List[Dict[str, float]]) -> float:
        """
        估计横向速度（yvRel）
        通过历史数据计算 yRel 的变化率

        参数:
        - current_yRel: 当前横向位置（未使用，保留用于接口兼容性）
        - current_dRel: 当前距离（未使用，保留用于接口兼容性）
        - history: 历史数据列表，包含{'yRel': float, 'dRel': float, 'timestamp': float}

        返回: 横向速度（m/s）
        """
        if len(history) < 2:
            return 0.0

        try:
            # 修复：使用历史数据中最近两帧的差值计算速度
            # 取最近的两帧
            recent = history[-2:]
            if len(recent) < 2:
                return 0.0

            dt = recent[1]['timestamp'] - recent[0]['timestamp']
            if dt <= 0:
                return 0.0

            # 修复：使用历史数据中最近两帧的差值，而不是当前值与历史值的差值
            dyRel = recent[1]['yRel'] - recent[0]['yRel']
            yvRel = dyRel / dt

            return float(yvRel)
        except (KeyError, IndexError, ZeroDivisionError):
            # 修复：使用具体的异常类型
            return 0.0

    def _calculate_lane_width(self, modelV2) -> float:
        """
        使用车道线坐标数据计算在约 20 米处计算车道宽度（使用插值方法）
        参考 carrot.cc:2119-2130

        优化：优先使用缓存的数据（已验证单调性），避免重复验证和重复数据转换
        """
        try:
            # 优化：优先使用缓存的数据，因为 _update_lane_cache() 已经验证过单调性
            # 这样可以避免重复验证和重复的数据转换，提升性能
            if self._lane_cache.get('cache_valid', False):
                lane_xs = self._lane_cache['lane_xs']
                left_ys = self._lane_cache['left_ys']
                right_ys = self._lane_cache['right_ys']

                # 使用类常量替代魔法数字
                target_distance = self.TARGET_LANE_WIDTH_DISTANCE

                # 检查目标距离是否在范围内（缓存数据已保证单调性）
                if (
                    len(lane_xs) > 0 and
                    target_distance <= max(lane_xs) and target_distance >= min(lane_xs)
                ):

                    # 使用缓存的数据进行插值计算
                    left_y_at_dist = np.interp(target_distance, lane_xs, left_ys)
                    right_y_at_dist = np.interp(target_distance, lane_xs, right_ys)
                    lane_width = abs(right_y_at_dist - left_y_at_dist)
                    return lane_width

            # 如果缓存无效，回退到直接从 modelV2 读取（需要验证单调性）
            # 需要至少 3 条车道线（ 0=左路边线, 1=左车道线, 2=右车道线, 3=右路边线）
            if not hasattr(modelV2, 'laneLines') or len(modelV2.laneLines) < 3:
                return 0.0

            left_lane = modelV2.laneLines[1]  # 左车道线
            right_lane = modelV2.laneLines[2]  # 右车道线

            target_distance = self.TARGET_LANE_WIDTH_DISTANCE

            if (
                len(left_lane.x) > 0 and len(left_lane.y) > 0 and
                len(right_lane.x) > 0 and len(right_lane.y) > 0
            ):

                left_x = [float(x) for x in left_lane.x]
                left_y = [float(y) for y in left_lane.y]
                right_x = [float(x) for x in right_lane.x]
                right_y = [float(y) for y in right_lane.y]

                # 验证列表非空后再调用 max/min，并验证 x 坐标单调性
                # 注意：只有在缓存无效时才需要验证，因为缓存已经验证过了
                if (
                    len(left_x) > 0 and len(right_x) > 0 and
                    # 验证 x 坐标单调递增（缓存无效时才需要）
                    len(left_x) >= 2 and all(left_x[i] < left_x[i + 1] for i in range(len(left_x) - 1)) and
                    len(right_x) >= 2 and all(right_x[i] < right_x[i + 1] for i in range(len(right_x) - 1)) and
                    # 检查目标距离是否在范围内
                    target_distance <= max(left_x) and target_distance <= max(right_x) and
                    target_distance >= min(left_x) and target_distance >= min(right_x)
                ):

                    left_y_at_dist = np.interp(target_distance, left_x, left_y)
                    right_y_at_dist = np.interp(target_distance, right_x, right_y)
                    lane_width = abs(right_y_at_dist - left_y_at_dist)
                    return lane_width
        except (IndexError, ValueError, TypeError):
            # 修复：使用具体的异常类型
            pass

        return 0.0

    def collect_model_data(self, modelV2, carState=None) -> Dict[str, Any]:
        """
        收集模型数据 - 优化版本
        通过 modelV2 数据间接推断侧方车辆情况，替代 radarState

        参数:
        - modelV2: 模型数据
        - carState: 车辆状态数据（可选，用于获取更准确的自车速度）
        """
        data = {}

        # 修复：优先使用 carState.vEgo（来自 CAN总线，更准确），如果不可用则使用模型估计
        v_ego = 0.0
        if carState is not None and hasattr(carState, 'vEgo'):
            v_ego = float(carState.vEgo)
        elif hasattr(modelV2, 'velocity') and len(modelV2.velocity.x) > 0:
            v_ego = float(modelV2.velocity.x[0])

        # modelVEgo 和 laneWidth 已删除
        # 更新车道线数据缓存（每帧更新一次，避免重复计算）
        self._update_lane_cache(modelV2)

        # 获取当前时间戳（用于计算横向速度）
        current_time = time.time()

        # 分类所有检测到的车辆（左/右/中车道）
        left_vehicles: List[Dict[str, Any]] = []
        right_vehicles: List[Dict[str, Any]] = []
        center_vehicles: List[Dict[str, Any]] = []

        # 遍历所有检测车辆
        for i, lead in enumerate(modelV2.leadsV3):
            lead_prob = float(lead.prob)

            # 动态置信度阈值：根据距离和速度调整
            # 参考 radard.py:126-157 的匹配逻辑
            x = float(lead.x[0]) if len(lead.x) > 0 else 0.0  # 纵向距离
            v = float(lead.v[0]) if len(lead.v) > 0 else 0.0  # 速度

            # 优化：使用类常量配置动态置信度阈值
            # 动态调整置信度阈值：距离越远或速度差异越大，要求置信度越高
            min_prob = self.CONFIDENCE_BASE_THRESHOLD
            if x > self.CONFIDENCE_DISTANCE_THRESHOLD:
                min_prob = max(min_prob, self.CONFIDENCE_DISTANCE_BOOST)
            if abs(v - v_ego) > self.CONFIDENCE_VELOCITY_DIFF_THRESHOLD:
                min_prob = max(min_prob, self.CONFIDENCE_VELOCITY_BOOST)

            # 过滤低置信度目标
            if lead_prob < min_prob:
                continue

            # 提取车辆数据
            y = float(lead.y[0]) if len(lead.y) > 0 else 0.0  # 横向位置
            a = float(lead.a[0]) if len(lead.a) > 0 else 0.0  # 加速度

            # 计算相对速度（使用更准确的自车速度）
            v_rel = v - v_ego  # 修复：使用 v_ego

            # 计算 dRel（考虑雷达到相机的偏移，参考 radard.py:220-243）
            # 注意：虽然不使用雷达，但 RADAR_TO_CAMERA 是相机到车辆中心的偏移
            dRel = x - self.RADAR_TO_CAMERA
            yRel = -y  # 注意符号：modelV2.leadsV3[i].y 与 yRel 符号相反

            # 浼拌妯悜閫熷害锛坹vRel锛� 鐢ㄤ簬鏈潵浣嶇疆棰勬祴
            # 瀵逛簬褰撳墠妫€娴嬭溅杈嗭紝浣跨敤绠€鍖栫殑鏂规硶锛氬亣璁炬í鍚戦€熷害涓庣浉瀵归€熷害鐩稿叧
            # 鍦ㄥ疄闄呭簲鐢ㄤ腑锛屽彲浠ラ€氳繃鍘嗗彶鏁版嵁璁＄畻锛岃繖閲屼娇鐢ㄧ畝鍖栦及璁�
            yvRel = 0.0  # 榛樿鍊硷紝灏嗗湪鍚庣画閫氳繃鍘嗗彶鏁版嵁鏀硅繘

            # 璁＄畻鍓嶈溅閫熷害锛坴Lead = vEgo + vRel锛�
            vLead = v_ego + v_rel  # 淇锛氫娇鐢� v_ego

            # 璁＄畻璺緞鍋忕Щ鍜岃溅閬撳唴姒傜巼锛堜娇鐢ㄧ紦瀛樺拰鏈潵浣嶇疆棰勬祴锛�
            dPath, in_lane_prob, in_lane_prob_future = self._calculate_dpath(dRel, yRel, yvRel, vLead)

            vehicle_data = {
                'x': x,
                'dRel': dRel,  # 鐩稿浜庨浄杈剧殑璺濈锛堝凡鑰冭檻 RADAR_TO_CAMERA 鍋忕Щ锛�
                'y': y,
                'yRel': yRel,  # 鐩稿浜庣浉鏈虹殑妯悜浣嶇疆
                'v': v,
                'vLead': vLead,  # 鍓嶈溅缁濆閫熷害
                'a': a,
                'vRel': v_rel  # 鐩稿閫熷害
            }

            # 为零提供此处数据
            vehicle_data.update({
                'yvRel': yvRel,  # 横向速度（用于未来位置预测）
                'dPath': dPath,  # 路径偏移
                'inLaneProb': in_lane_prob,  # 车道内概率
                'inLaneProbFuture': in_lane_prob_future,  # 未来车道内概率（用于 Cut-in 检测）
                'prob': lead_prob,
                'timestamp': current_time  # 时间戳，用于计算横向速度
            })

            # 优化：使用类常量配置车道分类阈值
            # 根据车道内概率和横向位置分类车辆
            # 参考 radard.py:520-546 的分类逻辑
            if in_lane_prob > self.LANE_PROB_THRESHOLD:
                # 当前车道车辆
                center_vehicles.append(vehicle_data)
            elif yRel < 0:  # 左侧车道
                left_vehicles.append(vehicle_data)
            else:  # 右侧车道
                right_vehicles.append(vehicle_data)

        # 前车检测 - 选择当前车道最近的前车（lead0）
        # 简化版：只保留超车决策必需的字段
        if center_vehicles:
            # 选择距离最近的前车
            lead0 = min(center_vehicles, key=lambda v: v['x'])
            data['lead0'] = {
                'x': lead0['x'],
                'y': lead0['y'],  # 横向位置（用于返回原车道判断）
                'v': lead0['v'],
                'prob': lead0['prob'],
            }
        elif len(modelV2.leadsV3) > 0:
            # 如果没有明确的中心车道车辆，使用第一个检测车辆
            lead0 = modelV2.leadsV3[0]
            x = float(lead0.x[0]) if len(lead0.x) > 0 else 0.0
            y = float(lead0.y[0]) if len(lead0.y) > 0 else 0.0
            v = float(lead0.v[0]) if len(lead0.v) > 0 else 0.0
            data['lead0'] = {
                'x': x,
                'y': y,  # 横向位置
                'v': v,
                'prob': float(lead0.prob),
            }
        else:
            data['lead0'] = {
                'x': 0.0, 'y': 0.0, 'v': 0.0, 'prob': 0.0
            }

        # 第二前车（lead1）已删除 - 简化版不再需要
        # 优化：使用类常量配置侧方车辆筛选参数
        # 侧方车辆检测 - 选择最近的左侧和右侧车辆
        # 参考 radard.py:560-569 的筛选逻辑
        left_filtered = [
            v for v in left_vehicles
            if v['dRel'] > self.SIDE_VEHICLE_MIN_DISTANCE and abs(v['dPath']) < self.SIDE_VEHICLE_MAX_DPATH
        ]
        right_filtered = [
            v for v in right_vehicles
            if v['dRel'] > self.SIDE_VEHICLE_MIN_DISTANCE and abs(v['dPath']) < self.SIDE_VEHICLE_MAX_DPATH
        ]

        # Cut-in 检测已删除 - 简化版不再需要
        # 选择左侧最近的车辆 - 简化版：只保留超车决策必需的字段
        if left_filtered:
            lead_left = min(left_filtered, key=lambda vehicle: vehicle['dRel'])
            data['leadLeft'] = {
                'dRel': lead_left['dRel'],  # 相对于雷达的距离
                'vRel': lead_left['vRel'],  # 相对速度
                'status': True,
            }
        else:
            data['leadLeft'] = {
                'dRel': 0.0,
                'vRel': 0.0,
                'status': False
            }

        # 选择右侧最近的车辆 - 简化版：只保留超车决策必需的字段
        if right_filtered:
            lead_right = min(right_filtered, key=lambda vehicle: vehicle['dRel'])
            data['leadRight'] = {
                'dRel': lead_right['dRel'],  # 相对于雷达的距离
                'vRel': lead_right['vRel'],  # 相对速度
                'status': True,
            }
        else:
            data['leadRight'] = {
                'dRel': 0.0,
                'vRel': 0.0,
                'status': False
            }

        # Cut-in 检测已删除 - 简化版不再需要
        # 车道线置信度 - 超车决策需要
        data['laneLineProbs'] = [
            float(modelV2.laneLineProbs[1]) if len(modelV2.laneLineProbs) >= 3 else 0.0,  # 左车道线置信度
            float(modelV2.laneLineProbs[2]) if len(modelV2.laneLineProbs) >= 3 else 0.0,  # 右车道线置信度
        ]

        # 车道宽度和变道状态 - 保留（超车决策需要）
        meta = modelV2.meta
        data['meta'] = {
            'distanceToRoadEdgeLeft': float(meta.distanceToRoadEdgeLeft),  # 左侧距离道路边缘
            'distanceToRoadEdgeRight': float(meta.distanceToRoadEdgeRight),  # 右侧距离道路边缘
        }

        # 曲率信息 - 用于判断弯道（超车决策关键数据）
        # 修复：改进空列表检查逻辑，使代码更清晰
        if hasattr(modelV2, 'orientationRate') and len(modelV2.orientationRate.z) > 0:
            orientation_rate_z = [float(x) for x in modelV2.orientationRate.z]
            data['curvature'] = {
                'maxOrientationRate': max(orientation_rate_z, key=abs),  # 最大方向变化率 (rad/s)
            }
        else:
            data['curvature'] = {'maxOrientationRate': 0.0}

        return data

    def collect_system_state(self, selfdriveState) -> Dict[str, Any]:
        """收集系统状态"""
        return {
            'enabled': bool(selfdriveState.enabled) if selfdriveState else False,
            'active': bool(selfdriveState.active) if selfdriveState else False,
        }

    # 移除 collect_carrot_data() - CarrotMan 数据已不再需要
    # 移除 collect_blindspot_data() - 盲区数据已直接 from carState 获取

    def create_packet(self, data: Dict[str, Any]) -> bytes:
        """
        创建数据包
        返回: UTF-8 编码的 JSON 字节串
        """
        packet_data = {
            'version': 1,
            'sequence': self.sequence,
            'timestamp': time.time(),
            'ip': self.device_ip,
            'data': data
        }

        # 转换为JSON
        json_str = json.dumps(packet_data)
        packet_bytes = json_str.encode('utf-8')

        # 直接返回 JSON 字节数据，由发送函数负责添加长度头
        # TCP 协议本身保证数据完整性，无需应用层 CRC32 校验

        # 检查数据包大小
        if len(packet_bytes) > 1024 * 1024:  # 1MB 警告
            print(f"Warning: Large packet size {len(packet_bytes)} bytes")

        return packet_bytes

    def broadcast_data(self):
        """主循环：收集数据并通过 TCP 推送给所有连接的客户端"""
        rk = Ratekeeper(20, print_delay_threshold=None)  # 20Hz

        # 启动 TCP 服务器（在独立线程中运行）
        server_thread = threading.Thread(
            target=self.start_tcp_server,
            daemon=True  # 设置为守护线程
        )
        server_thread.start()

        # 等待服务器启动
        time.sleep(0.5)

        print(f"XiaogeDataBroadcaster started, TCP server listening on port {self.tcp_port}")

        try:
            while True:
                try:
                    # 性能监控
                    start_time = time.perf_counter()

                    # 更新所有消息
                    self.sm.update(0)

                    # 收集数据
                    data = {}

                    # 本车状态 - 始终收集（数据验证已在 collect_car_state() 内部完成）
                    if self.sm.alive['carState']:
                        data['carState'] = self.collect_car_state(self.sm['carState'])

                    # 模型数据
                    if self.sm.alive['modelV2']:
                        # 修复：传递 carState 以获取更准确的自车速度
                        carState = self.sm['carState'] if self.sm.alive['carState'] else None
                        data['modelV2'] = self.collect_model_data(self.sm['modelV2'], carState)

                    # 系统状态
                    if self.sm.alive['selfdriveState']:
                        data['systemState'] = self.collect_system_state(
                            self.sm['selfdriveState']
                        )

                    # 盲区数据已包含在 carState 中

                    # 性能监控
                    processing_time = time.perf_counter() - start_time
                    if processing_time > 0.05:  # 超过50ms
                        print(f"Warning: Slow processing detected: {processing_time * 1000:.1f}ms")

                    # 如果有数据则推送给所有连接的客户端
                    # 注意：如果 openpilot 系统正常运行，至少会有 carState 数据
                    # 心跳机制已在 handle_client() 中实现（30秒间隔）
                    if data:
                        packet = self.create_packet(data)

                        try:
                            # 向所有连接的客户端广播数据包
                            self.broadcast_to_clients(packet)
                            self.sequence += 1

                            # 每 100 帧打印一次日志
                            if self.sequence % 100 == 0:
                                with self.clients_lock:
                                    client_count = len(self.clients)
                                print(f"Sent {self.sequence} packets to {client_count} clients, last size: {len(packet)} bytes")
                        except Exception as e:
                            print(f"Failed to send packet to clients: {e}")
                    else:
                        # 如果没有数据，发送一个最小的心跳数据包，保持连接活跃
                        # 这样客户端就不会因为超时而断开连接
                        try:
                            # 创建一个最小的心跳数据包（只包含基本结构， data 字段为空对象）
                            # 注意：data 字段必须是有效的 JSON 对象，不能为 null，否则Android 端解析会失败
                            heartbeat_packet = {
                                'version': 1,
                                'sequence': self.sequence,
                                'timestamp': time.time(),
                                'ip': self.device_ip,
                                'data': {}  # 空对象，而不是null，确保Android 端能正确解析
                            }
                            json_str = json.dumps(heartbeat_packet)
                            packet_bytes = json_str.encode('utf-8')
                            self.broadcast_to_clients(packet_bytes)
                            self.sequence += 1
                        except Exception:
                            # 心跳包发送失败不影响主流程
                            pass

                    rk.keep_time()

                except KeyboardInterrupt:
                    # 捕获 Ctrl+C，优雅关闭
                    print("\nReceived shutdown signal, closing gracefully...")
                    break
                except Exception as e:
                    print(f"XiaogeDataBroadcaster error: {e}")
                    traceback.print_exc()
                    time.sleep(1)
        finally:
            # 确保优雅关闭
            self.shutdown()


def main():
    broadcaster = XiaogeDataBroadcaster()
    broadcaster.broadcast_data()


if __name__ == "__main__":
    main()