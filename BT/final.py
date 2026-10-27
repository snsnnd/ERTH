import sys
import json
import re
import struct
import serial
import time
import hashlib
from html import escape
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QComboBox, QTextEdit, QLabel,
                             QGroupBox, QTabWidget, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QFileDialog, QMessageBox, QSplitter,
                             QProgressDialog, QListWidget, QDialog, QDialogButtonBox,
                             QLineEdit, QStackedWidget, QGridLayout)
from PyQt5.QtCore import QTimer, Qt, QPointF, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor
import pyqtgraph as pg
from pyqtgraph import PlotWidget, PlotItem
from battle_protocol import PlatformProtocolV2
from battle_protocol.platform import DRIVE_LEASE_MS

# 添加蓝牙支持（需要安装 bleak 库）
try:
    from bleak import BleakScanner, BleakClient

    BLUETOOTH_SUPPORTED = True
except ImportError:
    BLUETOOTH_SUPPORTED = False
    print("未安装 bleak 库，蓝牙功能不可用。请运行: pip install bleak")

import asyncio

BLE_UART_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
BLE_UART_CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


# ==================== 对战协议 v1：图结构 / 裁判（纯函数） ====================

GRID_STEP = 400.0      # 地图网格间距（估算跳数折算基准）
RPOS_MAX_HOPS = 2      # RPOS 允许的最大图上跳数

LOG_COLOR_A = '#1565C0'     # Car A 日志：蓝
LOG_COLOR_B = '#C62828'     # Car B 日志：红
LOG_COLOR_SYS = '#00695C'   # 转发/系统事件
LOG_COLOR_WARN = '#D84315'  # 告警
LOG_COLOR_GREY = '#757575'  # 离线等提示


def build_adjacency(nodes):
    """由节点列表构建无向邻接表: {node_id: set(neighbor_id)}（map.c 的边按无向处理）"""
    adj = {}
    ids = set()
    for node in nodes:
        nid = node['node_id']
        ids.add(nid)
        adj.setdefault(nid, set())
    for node in nodes:
        nid = node['node_id']
        for nb in node.get('neighbors', []):
            if nb in ids:
                adj[nid].add(nb)
                adj[nb].add(nid)
    return adj


def nearest_node(xy, nodes):
    """欧氏距离最近节点，返回 node_id；坐标或节点缺失时返回 None"""
    if not xy or not nodes:
        return None
    x, y = xy
    best_id = None
    best_d2 = None
    for node in nodes:
        d2 = (node['x'] - x) ** 2 + (node['y'] - y) ** 2
        if best_d2 is None or d2 < best_d2:
            best_id = node['node_id']
            best_d2 = d2
    return best_id


def graph_dist(n1, n2, adj):
    """BFS 求无向图两节点跳数；节点不存在或不可达返回 None"""
    if adj is None or n1 is None or n2 is None:
        return None
    if n1 not in adj or n2 not in adj:
        return None
    if n1 == n2:
        return 0
    visited = {n1}
    frontier = [n1]
    dist = 0
    while frontier:
        dist += 1
        nxt = []
        for cur in frontier:
            for nb in adj[cur]:
                if nb == n2:
                    return dist
                if nb not in visited:
                    visited.add(nb)
                    nxt.append(nb)
        frontier = nxt
    return None


def _estimate_hops(pos_xy, rpos_xy):
    """节点缺失时的降级估算：欧氏距离 / GRID_STEP"""
    if not pos_xy or not rpos_xy:
        return 0.0
    dx = pos_xy[0] - rpos_xy[0]
    dy = pos_xy[1] - rpos_xy[1]
    return (dx * dx + dy * dy) ** 0.5 / GRID_STEP


def judge_rpos_ex(pos_xy, rpos_xy, graph):
    """裁判 RPOS 上报，返回 (ok, d, degraded)。

    ok        是否放行（图距离 <= RPOS_MAX_HOPS）
    d         图上跳数；降级路径返回欧氏距离估算跳数
    degraded  是否走了降级放行路径（需日志告警）
    """
    nodes = graph.get('nodes') if graph else None
    adj = graph.get('adj') if graph else None
    n1 = nearest_node(pos_xy, nodes)
    n2 = nearest_node(rpos_xy, nodes)
    if n1 is None or n2 is None:
        return True, _estimate_hops(pos_xy, rpos_xy), True
    d = graph_dist(n1, n2, adj)
    if d is None:
        return True, _estimate_hops(pos_xy, rpos_xy), True
    return d <= RPOS_MAX_HOPS, d, False


def judge_rpos(pos_xy, rpos_xy, graph):
    """裁判接口：(ok, d)。ok 为 False 时 d 为实际图距离（用于 ERR:RPOS,<d>）"""
    ok, d, _degraded = judge_rpos_ex(pos_xy, rpos_xy, graph)
    return ok, d


class CarLink:
    """单车 SPP 串口链路封装（对战协议 v1）

    - bytes 级缓冲按 \\n 切分后再逐行解码，避免粘包时劈坏多字节字符
    - 接收缓冲上限 64KB 防溢出，超限丢弃最旧数据并告警一次
    - per-car 状态：pos / rpos / 最后收时间 / 最后 POS 时间（3 秒离线判定）
    """
    BUFFER_LIMIT = 64 * 1024  # 接收缓冲上限（字节，64KB）
    OFFLINE_TIMEOUT = 3.0     # 秒：未收到 POS 判离线

    def __init__(self, tag, on_line=None, encoding='utf-8'):
        self.tag = tag                     # 'A' / 'B'
        self.port = None
        self.baudrate = 115200
        self.serial_obj = None
        self.encoding = encoding
        self.on_line = on_line             # callback(tag, line)
        self.protocol_mode = 'V1'
        self.rx_buffer = b''               # bytes 级粘包缓冲
        self._buffer_overflow_warned = False  # 缓冲超限告警只发一次
        # per-car 状态
        self.pos = None                    # (x, y) 实际位置
        self.rpos = None                   # (x, y) 上报位置
        self.last_rx_time = 0.0            # 最后收到任意数据的时间
        self.last_pos_time = 0.0           # 最后收到 POS 的时间
        # 10ms 轮询读取定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self._read_tick)

    # ---------- 连接管理 ----------
    @property
    def is_open(self):
        return bool(self.serial_obj is not None and getattr(self.serial_obj, 'is_open', False))

    def connect(self, port, baudrate):
        """打开串口并启动轮询读取"""
        self.disconnect()
        self.port = port
        self.baudrate = int(baudrate)
        self.serial_obj = serial.Serial(port, self.baudrate, timeout=1)
        self.rx_buffer = b''
        self._buffer_overflow_warned = False  # 新连接允许再告警一次
        self.timer.start(10)  # 每 10ms 读取一次
        return True

    def disconnect(self):
        """关闭串口并停止读取定时器"""
        self.timer.stop()
        if self.serial_obj is not None:
            try:
                if self.serial_obj.is_open:
                    self.serial_obj.close()
            except Exception as e:
                print(f"[{self.tag}] 关闭串口错误: {e}")
        self.serial_obj = None
        self.rx_buffer = b''

    def close(self):
        """关窗清理"""
        self.disconnect()

    # ---------- 接收：bytes 缓冲按 \n 切分，再逐行解码 ----------
    def feed_bytes(self, data):
        """喂入原始字节，返回解码完成的完整行列表（不含换行符）"""
        if not data:
            return []
        self.rx_buffer += data
        if len(self.rx_buffer) > self.BUFFER_LIMIT:
            # 防溢出：丢弃最旧数据，只保留缓冲区尾部；告警一次避免刷屏
            self.rx_buffer = self.rx_buffer[-self.BUFFER_LIMIT:]
            if not self._buffer_overflow_warned:
                self._buffer_overflow_warned = True
                print(f"[{self.tag}] 接收缓冲超过 {self.BUFFER_LIMIT // 1024}KB 上限，"
                      f"已丢弃最旧数据")
        lines = []
        while b'\n' in self.rx_buffer:
            raw, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            try:
                encoding = 'ascii' if self.protocol_mode == 'V2' else self.encoding
                line = raw.decode(encoding, errors='strict' if self.protocol_mode == 'V2' else 'ignore').rstrip('\r')
            except UnicodeDecodeError:
                continue
            line = line.strip()
            if line:
                lines.append(line)
        return lines

    def _read_tick(self):
        """定时读取串口（10ms 轮询）"""
        if not self.is_open:
            return
        try:
            bytes_to_read = min(self.serial_obj.in_waiting, 1024)
            if bytes_to_read <= 0:
                return
            data = self.serial_obj.read(bytes_to_read)
        except Exception as e:
            print(f"[{self.tag}] 串口读取错误: {e}")
            return
        # 逐行隔离：单行处理异常只记日志，不中断本批次其余行
        for line in self.feed_bytes(data):
            self.last_rx_time = time.time()
            if not self.on_line:
                continue
            try:
                self.on_line(self.tag, line)
            except Exception as e:
                print(f"[{self.tag}] 行处理错误: {e}")

    # ---------- 发送 ----------
    def send_line(self, text):
        """发送一行文本（自动补 \\n），成功返回 True"""
        if not self.is_open:
            return False
        try:
            if not text.endswith('\n'):
                text += '\n'
            self.serial_obj.write(text.encode(self.encoding, errors='ignore'))
            return True
        except Exception as e:
            print(f"[{self.tag}] 串口发送错误: {e}")
            return False

    # ---------- 在线判定 ----------
    def note_pos(self):
        """记录收到 POS 的时间（离线判定依据）"""
        self.last_pos_time = time.time()

    def is_online(self, now=None):
        """3 秒内收到过 POS 才算在线"""
        if self.last_pos_time <= 0:
            return False
        if now is None:
            now = time.time()
        return (now - self.last_pos_time) <= self.OFFLINE_TIMEOUT


class BluetoothScanWorker(QThread):
    """蓝牙扫描工作线程"""
    devices_found = pyqtSignal(list)
    error_occurred = pyqtSignal(str)

    def run(self):
        if not BLUETOOTH_SUPPORTED:
            self.error_occurred.emit("未安装蓝牙支持库")
            return

        try:
            # 在新事件循环中运行异步函数
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            devices = loop.run_until_complete(self._discover_devices())
            loop.close()

            device_list = []
            for device in devices:
                device_list.append({
                    'name': device.name or "未知设备",
                    'address': device.address
                })
            self.devices_found.emit(device_list)
        except Exception as e:
            self.error_occurred.emit(str(e))

    async def _discover_devices(self):
        """异步扫描蓝牙设备"""
        devices = await BleakScanner.discover(timeout=5)
        return devices


class BluetoothConnectionWorker(QThread):
    """蓝牙连接工作线程"""
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)
    disconnected = pyqtSignal()
    services_found = pyqtSignal(object)  # 发送服务对象

    def __init__(self, device_address):
        super().__init__()
        self.device_address = device_address
        self.client = None
        self.is_running = False
        self.read_characteristic = None  # 需要根据实际设备设置
        self.write_characteristic = None  # 需要根据实际设备设置
        self.max_write_size = 20
        self.notify_characteristic = None
        self.write_with_response = True
        self.loop = None
        self.buffer = ""  # 用于存储未完成的数据

    async def _connect_and_listen(self):
        """连接蓝牙设备并监听数据"""
        try:
            self.client = BleakClient(self.device_address)
            await self.client.connect()

            if not self.client.is_connected:
                self.connection_status.emit(False, "连接失败")
                return

            # 获取服务和特征信息（使用新API）
            try:
                services = self.client.services  # 新的API
                self.services_found.emit(services)  # 发送服务
                write_candidates = []
                notify_candidates = []
                for service in services:
                    for char in service.characteristics:
                        properties = {str(prop).lower() for prop in char.properties}
                        if "write" in properties or "write-without-response" in properties:
                            write_candidates.append((service.uuid.lower(), char))
                        if "notify" in properties or "indicate" in properties:
                            notify_candidates.append(char)

                preferred_write = next((char for service_uuid, char in write_candidates
                                         if service_uuid == BLE_UART_SERVICE_UUID and
                                         char.uuid.lower() == BLE_UART_CHARACTERISTIC_UUID), None)
                write_char = preferred_write or (write_candidates[0][1] if write_candidates else None)
                if write_char is None:
                    self.connection_status.emit(False, "未找到可写入的 GATT 特征")
                    return
                self.write_characteristic = write_char.uuid
                self.max_write_size = max(
                    1,
                    int(getattr(write_char, "max_write_without_response_size", 20) or 20),
                )
                self.write_with_response = "write" in {
                    str(prop).lower() for prop in write_char.properties
                }
                preferred_notify = next(
                    (char for service in services for char in service.characteristics
                     if service.uuid.lower() == BLE_UART_SERVICE_UUID and
                     char.uuid.lower() == BLE_UART_CHARACTERISTIC_UUID and
                     ({str(prop).lower() for prop in char.properties} & {"notify", "indicate"})),
                    None
                )
                notify_char = preferred_notify or (notify_candidates[0] if notify_candidates else None)
                self.notify_characteristic = notify_char.uuid if notify_char else None
                self.read_characteristic = next(
                    (char.uuid for service in services for char in service.characteristics
                     if "read" in {str(prop).lower() for prop in char.properties}), None
                )

                if self.notify_characteristic:
                    try:
                        await self.client.start_notify(self.notify_characteristic,
                                                       self._notification_handler)
                    except Exception as e:
                        print(f"启用蓝牙通知失败: {e}")
                        self.notify_characteristic = None

                self.is_running = True
                self.connection_status.emit(
                    True, "连接成功" if self.notify_characteristic else "连接成功（设备未提供通知特征）"
                )
            except Exception as e:
                print(f"获取服务信息失败: {e}")
                self.connection_status.emit(False, f"读取 GATT 服务失败: {e}")
                return

            # 接收依赖 GATT 通知；保持连接循环直到用户断开或设备离线。
            while self.is_running and self.client.is_connected:
                try:
                    await asyncio.sleep(0.1)
                except Exception as e:
                    print(f"读取数据错误: {e}")
                    break

        except Exception as e:
            self.connection_status.emit(False, f"连接失败: {str(e)}")
        finally:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
            self.disconnected.emit()

    def process_data(self, data):
        """处理接收到的数据，以\n为一段数据的结束"""
        self.buffer += data
        while '\n' in self.buffer:
            # 分割数据，以\n为分隔符
            message, self.buffer = self.buffer.split('\n', 1)
            if message:
                self.data_received.emit(message.strip())

    def _notification_handler(self, sender, data):
        """通知回调处理"""
        try:
            self.process_data(data.decode('utf-8'))
        except Exception as e:
            print(f"处理通知数据错误: {e}")

    def run(self):
        if not BLUETOOTH_SUPPORTED:
            self.connection_status.emit(False, "未安装蓝牙支持库")
            return

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._connect_and_listen())
        finally:
            self.loop.close()
            self.loop = None

    def stop(self):
        self.is_running = False

    def send_data(self, data):
        """发送数据到蓝牙设备"""
        if not self.is_running or not self.write_characteristic or self.loop is None:
            return False
        if isinstance(data, str) and not data.endswith('\n'):
            data += '\n'
        coroutine = self._send_data_async(data)
        try:
            future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        except RuntimeError:
            coroutine.close()
            return False
        future.add_done_callback(self._send_finished)
        return True

    @staticmethod
    def _send_finished(future):
        try:
            future.result()
        except Exception as e:
            print(f"发送数据错误: {e}")

    async def _send_data_async(self, data):
        """异步发送数据"""
        if isinstance(data, str):
            data = data.encode('utf-8')
        if not self.client or not self.client.is_connected:
            raise ConnectionError("蓝牙设备未连接")
        for offset in range(0, len(data), self.max_write_size):
            chunk = data[offset:offset + self.max_write_size]
            await self.client.write_gatt_char(self.write_characteristic, chunk,
                                              response=self.write_with_response)


class DeviceSelectionDialog(QDialog):
    """设备选择对话框"""

    def __init__(self, devices, parent=None):
        super().__init__(parent)
        self.devices = devices
        self.selected_device = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("选择蓝牙设备")
        self.setGeometry(300, 300, 400, 300)

        layout = QVBoxLayout()

        self.device_list = QListWidget()
        for device in self.devices:
            self.device_list.addItem(f"{device['name']} ({device['address']})")
        self.device_list.itemDoubleClicked.connect(self.accept)
        layout.addWidget(self.device_list)

        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)

    def accept(self):
        current_row = self.device_list.currentRow()
        if current_row >= 0:
            self.selected_device = self.devices[current_row]
        super().accept()


# 地图可视化组件
class MapWidget(QWidget):
    """双车对战地图：A=蓝、B=红；实心点=真实位置(POS)，空心点=上报位置(RPOS)，虚线=谎报偏差"""

    # 图例定义（名称 + 样式）
    LEGEND_SPECS = [
        ('A 实际位置', {'pen': None, 'symbol': 'o', 'symbolSize': 12,
                       'symbolBrush': 'b', 'symbolPen': 'b'}),
        ('A 上报位置(谎报)', {'pen': None, 'symbol': 'o', 'symbolSize': 12,
                            'symbolBrush': None, 'symbolPen': 'b'}),
        ('B 实际位置', {'pen': None, 'symbol': 'o', 'symbolSize': 12,
                       'symbolBrush': 'r', 'symbolPen': 'r'}),
        ('B 上报位置(谎报)', {'pen': None, 'symbol': 'o', 'symbolSize': 12,
                            'symbolBrush': None, 'symbolPen': 'r'}),
        ('谎报偏差', {'pen': pg.mkPen('gray', width=2, style=Qt.DashLine), 'symbol': None}),
    ]

    def __init__(self):
        super().__init__()
        self.nodes = []
        self.edges = []
        self.adj = {}                          # 无向邻接表（裁判用）
        self.graph = {'nodes': [], 'adj': {}}   # judge_rpos 使用的图结构
        self.map_version = 'NONE'

        # 每辆车独立的显示状态与防抖状态
        self.car_state = {}
        for tag in ('A', 'B'):
            self.car_state[tag] = {
                'pos': None,       # 真实位置 (x, y)
                'rpos': None,      # 上报位置 (x, y)
                'pos_item': None,
                'rpos_item': None,
                'link_item': None,
                'pos_debounce': {'last_update_time': 0, 'pending_update': False},
                'rpos_debounce': {'last_update_time': 0, 'pending_update': False},
            }

        self.update_interval = 100
        self.init_ui()

    def update_car_position(self, tag, x, y):
        """更新某车真实位置（POS 直接定位）"""
        st = self.car_state[tag]
        st['pos'] = (x, y)
        self._update_marker_with_debounce(
            st['pos_debounce'],
            lambda t=tag: self._redraw_car_marker(t)
        )

    def update_car_report(self, tag, x, y):
        """更新某车上报位置（RPOS 幽灵点 + 谎报偏差虚线）"""
        st = self.car_state[tag]
        st['rpos'] = (x, y)
        self._update_marker_with_debounce(
            st['rpos_debounce'],
            lambda t=tag: self._redraw_car_marker(t)
        )

    # 替换 MapWidget 类中的防抖相关方法
    def _update_marker_with_debounce(self, state, update_func):
        """带防抖的标记更新通用方法"""
        import time
        current_time = time.time() * 1000  # 转换为毫秒

        # 检查是否需要立即更新
        if current_time - state['last_update_time'] > self.update_interval:
            state['last_update_time'] = current_time
            state['pending_update'] = False  # 确保重置状态
            update_func()
        else:
            # 设置延迟更新标志
            if not state['pending_update']:
                state['pending_update'] = True
                # 使用lambda确保正确传递参数，并确保状态重置
                QTimer.singleShot(self.update_interval, lambda: self._delayed_update(state, update_func))

    def _delayed_update(self, state, update_func):
        """延迟更新通用方法"""
        state['last_update_time'] = time.time() * 1000
        state['pending_update'] = False  # 重置pending状态
        update_func()

    def _car_color(self, tag):
        """车辆配色：A=蓝, B=红"""
        return 'b' if tag == 'A' else 'r'

    def _redraw_car_marker(self, tag):
        """重绘某车的三个标记：真实点（实心）、上报幽灵点（空心）、谎报偏差虚线"""
        st = self.car_state[tag]
        color = self._car_color(tag)

        for key in ('pos_item', 'rpos_item', 'link_item'):
            if st[key] is not None:
                try:
                    self.plot_widget.removeItem(st[key])
                except Exception:
                    pass
                st[key] = None

        if st['pos']:
            st['pos_item'] = pg.PlotDataItem(
                [st['pos'][0]], [st['pos'][1]],
                pen=None, symbol='o', symbolSize=15,
                symbolBrush=color, symbolPen=color
            )
            self.plot_widget.addItem(st['pos_item'])

        if st['rpos']:
            st['rpos_item'] = pg.PlotDataItem(
                [st['rpos'][0]], [st['rpos'][1]],
                pen=None, symbol='o', symbolSize=15,
                symbolBrush=None, symbolPen=pg.mkPen(color, width=2)
            )
            self.plot_widget.addItem(st['rpos_item'])

        if st['pos'] and st['rpos']:
            st['link_item'] = pg.PlotDataItem(
                [st['pos'][0], st['rpos'][0]],
                [st['pos'][1], st['rpos'][1]],
                pen=pg.mkPen(color, width=2, style=Qt.DashLine)
            )
            self.plot_widget.addItem(st['link_item'])

    def _add_legend_entries(self):
        """（重）建图例条目，避免 clear() 后条目丢失或重复"""
        legend = getattr(self.plot_widget, 'legend', None)
        if legend is None:
            return
        for name, kw in self.LEGEND_SPECS:
            try:
                legend.removeItem(name)
            except Exception:
                pass
            try:
                self.plot_widget.plot([], [], name=name, **kw)
            except Exception:
                pass

    def init_ui(self):
        layout = QVBoxLayout()

        # 地图显示区域
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Y坐标')
        self.plot_widget.setLabel('bottom', 'X坐标')
        self.plot_widget.setTitle('地图可视化')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.addLegend()

        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def load_map_from_c_file(self, file_path):
        """从.c文件加载地图数据"""
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                content = file.read()

            # 改进的节点解析正则表达式：坐标/费用支持负数（-30.0f 等负坐标不再漏解析）
            node_pattern = (r'\{(-?\d+),\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*(-?\d+),'
                            r'\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*(-?\d+),\s*\{([^}]+)\}\}')
            node_matches = re.findall(node_pattern, content)

            self.nodes = []
            for match in node_matches:
                node_id = int(match[0])
                x = float(match[1])
                y = float(match[2])
                base_cost = float(match[4])
                cost = float(match[5])
                neighbor_count = int(match[6])
                neighbors_str = match[7]

                # 改进邻居节点解析
                neighbors = []
                for n in neighbors_str.split(','):
                    n = n.strip()
                    if n != '-1' and n:  # 过滤掉-1和空值
                        try:
                            neighbors.append(int(n))
                        except ValueError:
                            pass

                self.nodes.append({
                    'node_id': node_id,
                    'x': x,
                    'y': y,
                    'base_cost': base_cost,
                    'cost': cost,
                    'neighbor_count': neighbor_count,
                    'neighbors': neighbors
                })

            # 边解析：只在 .edges = ... node_count 区域内匹配——否则四邻居槽
            # {10,12,3,22}（全为整数）会被误判成 4 条伪边；端点与距离允许负号。
            # 无区域标记时退化为要求浮点字面量带小数点，同样排除邻居槽
            edge_start = content.find('.edges =')
            edge_end = content.find('node_count')
            if 0 <= edge_start < edge_end:
                edge_region = content[edge_start:edge_end]
                edge_pattern = r'\{(-?\d+),\s*(-?\d+),\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?\}'
            else:
                edge_region = content
                edge_pattern = r'\{(-?\d+),\s*(-?\d+),\s*(-?\d+\.\d*f?),\s*(-?\d+\.\d*f?)\}'
            edge_matches = re.findall(edge_pattern, edge_region)

            self.edges = []
            for match in edge_matches:
                node1 = int(match[0])
                node2 = int(match[1])
                distance = float(match[2])
                cost = float(match[3])

                self.edges.append({
                    'node1': node1,
                    'node2': node2,
                    'distance': distance,
                    'cost': cost
                })

            # 构建裁判用的无向邻接表与图结构
            self.adj = build_adjacency(self.nodes)
            self.graph = {'nodes': self.nodes, 'adj': self.adj}
            self.map_version = hashlib.sha256(content.encode('utf-8')).hexdigest()[:8].upper()

            # 绘制地图
            self.draw_map()
            return True

        except Exception as e:
            print(f"解析地图文件错误: {e}")
            return False

    # 在 MapWidget 类中优化 draw_map 方法
    def draw_map(self):
        """绘制地图（不包括当前位置）"""
        # 使用 QTimer.singleShot 将耗时的图形操作推迟到事件循环
        QTimer.singleShot(0, self._draw_map_internal)

    def _draw_map_internal(self):
        """实际的地图绘制操作"""
        self.plot_widget.clear()

        if not self.nodes:
            return

        # 绘制边（只连接相邻节点）
        for node in self.nodes:
            x1, y1 = node['x'], node['y']

            # 遍历当前节点的邻居
            for neighbor_id in node['neighbors']:
                neighbor = next((n for n in self.nodes if n['node_id'] == neighbor_id), None)
                if neighbor:
                    x2, y2 = neighbor['x'], neighbor['y']

                    # 判断是否为相邻节点（水平或垂直相邻）
                    if (abs(x1 - x2) > 0 and abs(y1 - y2) == 0) or \
                            (abs(y1 - y2) > 0 and abs(x1 - x2) == 0):
                        # 绘制水平或垂直连线
                        self.plot_widget.plot([x1, x2], [y1, y2],
                                              pen=pg.mkPen('gray', width=2))

        # 绘制节点（统一颜色，移除危险节点相关逻辑）
        x_coords = [node['x'] for node in self.nodes]
        y_coords = [node['y'] for node in self.nodes]
        colors = ['b' for _ in self.nodes]  # 所有节点都用蓝色

        scatter = pg.ScatterPlotItem(x=x_coords, y=y_coords, size=10,
                                     brush=colors, pen='w')
        self.plot_widget.addItem(scatter)

        # 添加节点标签
        for node in self.nodes:
            text = pg.TextItem(str(node['node_id']), color='black', anchor=(0.5, 0.5))
            text.setPos(node['x'], node['y'])
            self.plot_widget.addItem(text)

        # 重新添加图例与所有车辆标记（真实点/上报点/偏差虚线）
        self._add_legend_entries()
        for tag in ('A', 'B'):
            self._redraw_car_marker(tag)


# PID可视化组件
class PIDWidget(QWidget):
    # 修改 PIDWidget 类的 __init__ 方法
    def __init__(self):
        super().__init__()
        self.motor_data = {i: [] for i in range(4)}  # 4个电机的数据
        self.time_data = []
        self.current_time = 0
        self.max_points = 100  # 显示的最大数据点数
        self.yaw_data = []  # 偏航角数据
        self.yaw_curve = None  # 偏航角曲线
        self.target_pid_data = {i: [] for i in range(4)}  # 每个电机各自的目标值数据
        self.target_yaw_data = []  # 偏航角目标值数据
        self.target_pid_curves = {}  # PID目标值曲线
        self.target_yaw_curve = None  # 偏航角目标值曲线
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # 创建4个图表，每个电机一个
        self.plot_widgets = []
        for i in range(4):
            plot = pg.PlotWidget()
            plot.setBackground('w')
            plot.setLabel('left', f'电机 {i + 1} PID值')
            plot.setLabel('bottom', '时间')
            plot.showGrid(x=True, y=True)
            plot.setYRange(-50, 50)  # 根据实际PID值范围调整
            # 移除这行: plot.setXRange(0, self.max_points)  # 设置X轴范围

            # 添加实际值曲线
            curve = plot.plot(pen=pg.mkPen('b', width=2))  # 实际值用蓝色
            # 添加目标值比对线（红色虚线）
            target_curve = plot.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine))

            self.motor_data[i] = {'curve': curve, 'data': []}
            self.target_pid_curves[i] = target_curve

            self.plot_widgets.append(plot)
            print(f"创建电机{i}的图表")  # 调试信息

        # 添加偏航角可视化图表
        self.yaw_plot = pg.PlotWidget()
        self.yaw_plot.setBackground('w')
        self.yaw_plot.setLabel('left', '偏航角 (度)')
        self.yaw_plot.setLabel('bottom', '时间')
        self.yaw_plot.showGrid(x=True, y=True)
        self.yaw_plot.setYRange(-180, 180)  # 偏航角范围 -180° 到 180°
        # 移除这行: self.yaw_plot.setXRange(0, self.max_points)
        self.yaw_curve = self.yaw_plot.plot(pen=pg.mkPen('b', width=2))  # 实际值用蓝色
        self.target_yaw_curve = self.yaw_plot.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine))  # 目标值用红色虚线
        self.yaw_plot.setTitle("偏航角可视化")

        # 将图表排列在网格中
        grid_layout = QHBoxLayout()
        # 左侧放置电机PID图表 (2x2网格)
        motor_grid = QWidget()
        motor_layout = QVBoxLayout(motor_grid)
        motor_row1 = QHBoxLayout()
        motor_row2 = QHBoxLayout()
        motor_row1.addWidget(self.plot_widgets[0])
        motor_row1.addWidget(self.plot_widgets[1])
        motor_row2.addWidget(self.plot_widgets[2])
        motor_row2.addWidget(self.plot_widgets[3])
        motor_layout.addLayout(motor_row1)
        motor_layout.addLayout(motor_row2)

        # 右侧放置偏航角图表
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.yaw_plot)

        grid_layout.addWidget(motor_grid, 70)  # 电机图表占70%宽度
        grid_layout.addLayout(right_layout, 30)  # 偏航角图表占30%宽度

        layout.addLayout(grid_layout)
        self.setLayout(layout)

    def update_pid_data(self, motor_id, pid_value, target_pid_value=0):
        """更新PID数据"""
        if motor_id < 0 or motor_id > 3:
            print(f"无效的电机ID: {motor_id}")
            return

        # 验证PID值
        try:
            pid_value = float(pid_value)
            target_pid_value = float(target_pid_value)
        except (ValueError, TypeError):
            print(f"无效的PID值: {pid_value} 或目标值: {target_pid_value}")
            return

        # 更新时间
        self.current_time += 1
        self.time_data.append(self.current_time)

        # 更新数据
        self.motor_data[motor_id]['data'].append(pid_value)

        # 为所有电机填充数据以保持同步
        for i in range(4):
            if i != motor_id:
                self.motor_data[i]['data'].append(0)  # 或使用上一个值

        # 每个电机维护各自的目标值；其他电机保持上次目标值以对齐时间轴
        self.target_pid_data[motor_id].append(target_pid_value)
        for i in range(4):
            if i != motor_id:
                last_target = self.target_pid_data[i][-1] if self.target_pid_data[i] else 0
                self.target_pid_data[i].append(last_target)

        # 为其他数据添加占位符以保持时间轴一致
        self.yaw_data.append(0)
        self.target_yaw_data.append(0)

        # 限制数据点数
        while len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            # 确保所有电机数据都移除第一个元素，保持长度一致
            for i in range(4):
                if self.motor_data[i]['data']:
                    self.motor_data[i]['data'].pop(0)
                # 同时移除该电机的目标值数据，保持时间轴对齐
                if self.target_pid_data[i]:
                    self.target_pid_data[i].pop(0)
            # 同时移除偏航角数据
            if self.yaw_data:
                self.yaw_data.pop(0)
            if self.target_yaw_data:
                self.target_yaw_data.pop(0)

        # 更新所有电机的图表
        for i in range(4):
            min_length = min(len(self.time_data), len(self.motor_data[i]['data']))
            if min_length > 0:
                x_data = self.time_data[-min_length:]
                y_data = self.motor_data[i]['data'][-min_length:]
                try:
                    self.motor_data[i]['curve'].setData(x_data, y_data)
                    # 动态调整X轴范围
                    if len(x_data) > 0:
                        self.plot_widgets[i].setXRange(max(0, x_data[-1] - self.max_points), x_data[-1])
                    # 该电机自己的目标值比对线（红色虚线）
                    target_len = min(len(self.time_data), len(self.target_pid_data[i]))
                    if target_len > 0:
                        self.target_pid_curves[i].setData(self.time_data[-target_len:],
                                                          self.target_pid_data[i][-target_len:])
                    if i == motor_id:
                        print(f"更新图表: 电机{i}, PID值{pid_value}, 数据点数{min_length}")
                except Exception as e:
                    print(f"更新图表失败: {e}")

    # 修改 PIDWidget 类的 update_yaw_data 方法
    def update_yaw_data(self, yaw_value, target_yaw_value=0):
        """更新偏航角数据"""
        try:
            yaw_value = float(yaw_value)
            target_yaw_value = float(target_yaw_value)
        except (ValueError, TypeError):
            print(f"无效的偏航角值: {yaw_value} 或目标值: {target_yaw_value}")
            return

        # 更新时间轴
        self.current_time += 1
        self.time_data.append(self.current_time)

        # 更新偏航角数据
        self.yaw_data.append(yaw_value)
        self.target_yaw_data.append(target_yaw_value)

        # 限制数据点数
        while len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            # 移除偏航角数据
            if self.yaw_data:
                self.yaw_data.pop(0)
            if self.target_yaw_data:
                self.target_yaw_data.pop(0)

        # 更新偏航角图表
        min_length = min(len(self.time_data), len(self.yaw_data))
        if min_length > 0:
            x_data = self.time_data[-min_length:]
            y_data = self.yaw_data[-min_length:]
            self.yaw_curve.setData(x_data, y_data)

            # 更新偏航角目标值比对线
            if len(self.target_yaw_data) > 0:
                min_length = min(len(self.time_data), len(self.target_yaw_data))
                if min_length > 0:
                    x_data = self.time_data[-min_length:]
                    y_data = self.target_yaw_data[-min_length:]
                    self.target_yaw_curve.setData(x_data, y_data)

        print(f"更新偏航角图表: 值{yaw_value}, 数据点数{min_length}")


# 主窗口
class BluetoothDebugTool(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bluetooth_connection = None
        self.is_bluetooth_connected = False
        self.received_data = ""
        self.bluetooth_scan_worker = None
        self.bluetooth_connection_worker = None
        self.services = None
        self.current_encoding = 'utf-8'
        # 双车对战链路（A/B 各一路 SPP 串口）
        self.links = {
            'A': CarLink('A', on_line=self.on_car_line),
            'B': CarLink('B', on_line=self.on_car_line),
        }
        self.v2_routes = {'A': None, 'B': None}
        self.ble_v2_role = None
        self.bluetooth_protocol_mode = 'V1'
        self.v2_platform = PlatformProtocolV2(
            self._send_v2_bytes,
            on_event=self._on_v2_event,
            on_frame=self._on_v2_frame,
        )
        self._prev_online = {'A': False, 'B': False}
        self.manual_drive_timer = QTimer(self)
        self.manual_drive_timer.setInterval(100)
        self.manual_drive_timer.timeout.connect(self._repeat_manual_drive)
        self.held_manual_pwm = None
        self.held_manual_target = None

        # 初始化 UI
        self.init_ui()
        self.v2_timer = QTimer(self)
        self.v2_timer.timeout.connect(self._tick_v2)
        self.v2_timer.start(50)

    def init_ui(self):
        self.setWindowTitle("STM32蓝牙调试工具")
        self.setGeometry(100, 100, 1200, 800)

        # 创建中央部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 创建左右分割
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # 左侧面板 - 控制和数据显示
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # 连接控制（双车对战：Car A / Car B 各一路 SPP 串口）
        connection_group = QGroupBox("连接控制")
        connection_layout = QVBoxLayout(connection_group)

        # 双车串口连接区：端口 + 刷新 + 波特率 + 连接/断开 + 在线状态灯
        self.car_ui = {}
        for tag in ('A', 'B'):
            connection_layout.addWidget(self._build_car_group(tag))

        v2_mode_row = QHBoxLayout()
        v2_mode_row.addWidget(QLabel("V2 对局模式:"))
        self.v2_mode_combo = QComboBox()
        self.v2_mode_combo.addItems(["BENCH", "OFFICIAL"])
        self.v2_mode_combo.currentTextChanged.connect(self._set_v2_mode)
        v2_mode_row.addWidget(self.v2_mode_combo)
        connection_layout.addLayout(v2_mode_row)

        # 解码方式选择（两路串口共用）
        decode_layout = QHBoxLayout()
        decode_layout.addWidget(QLabel("解码方式:"))
        self.decode_combo = QComboBox()
        self.decode_combo.addItems(["UTF-8", "GBK", "ASCII"])
        self.decode_combo.setCurrentText("UTF-8")
        self.decode_combo.currentTextChanged.connect(self.update_encoding)
        decode_layout.addWidget(self.decode_combo)
        connection_layout.addLayout(decode_layout)

        # 蓝牙连接部分
        if BLUETOOTH_SUPPORTED:
            bluetooth_layout = QHBoxLayout()
            self.scan_bluetooth_btn = QPushButton("扫描蓝牙设备")
            self.scan_bluetooth_btn.clicked.connect(self.scan_bluetooth_devices)
            bluetooth_layout.addWidget(self.scan_bluetooth_btn)

            self.bluetooth_connect_btn = QPushButton("蓝牙连接")
            self.bluetooth_connect_btn.clicked.connect(self.connect_bluetooth_device)
            self.bluetooth_connect_btn.setEnabled(False)
            bluetooth_layout.addWidget(self.bluetooth_connect_btn)

            bluetooth_layout.addWidget(QLabel("协议:"))
            self.bluetooth_protocol_combo = QComboBox()
            self.bluetooth_protocol_combo.addItems(["V1", "V2"])
            bluetooth_layout.addWidget(self.bluetooth_protocol_combo)
            bluetooth_layout.addWidget(QLabel("角色:"))
            self.bluetooth_role_combo = QComboBox()
            self.bluetooth_role_combo.addItems(["A", "B"])
            bluetooth_layout.addWidget(self.bluetooth_role_combo)

            connection_layout.addLayout(bluetooth_layout)

            # UUID搜索部分
            uuid_layout = QHBoxLayout()
            uuid_layout.addWidget(QLabel("搜索UUID:"))
            self.uuid_input = QLineEdit()
            uuid_layout.addWidget(self.uuid_input)
            search_uuid_btn = QPushButton("搜索")
            search_uuid_btn.clicked.connect(self.search_uuid)
            uuid_layout.addWidget(search_uuid_btn)
            connection_layout.addLayout(uuid_layout)

        left_layout.addWidget(connection_group)

        # 数据发送区域
        send_group = QGroupBox("发送数据")
        send_layout = QVBoxLayout(send_group)

        self.send_text = QTextEdit()
        self.send_text.setMaximumHeight(100)
        send_layout.addWidget(self.send_text)

        send_btn_layout = QHBoxLayout()
        send_btn_layout.addWidget(QLabel("目标:"))
        self.send_target_combo = QComboBox()
        self.send_target_combo.addItems(["A", "B", "蓝牙"])
        send_btn_layout.addWidget(self.send_target_combo)

        self.send_btn = QPushButton("发送")
        self.send_btn.clicked.connect(self.send_data)
        send_btn_layout.addWidget(self.send_btn)

        clear_send_btn = QPushButton("清空")
        clear_send_btn.clicked.connect(lambda: self.send_text.clear())
        send_btn_layout.addWidget(clear_send_btn)
        send_layout.addLayout(send_btn_layout)

        left_layout.addWidget(send_group)

        # 车辆直接控制：按住方向键持续续发，松开立即发送零速指令。
        manual_group = QGroupBox("车辆直接控制（按住行驶，松开停车）")
        manual_layout = QVBoxLayout(manual_group)
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("PWM 速度（0～999）:"))
        self.drive_speed_input = QSpinBox()
        self.drive_speed_input.setRange(0, 999)
        self.drive_speed_input.setSingleStep(50)
        self.drive_speed_input.setValue(300)
        speed_layout.addWidget(self.drive_speed_input)
        speed_layout.addWidget(QLabel("控制目标使用上方“目标”选择。"))
        manual_layout.addLayout(speed_layout)
        manual_layout.addWidget(QLabel("命令：DRV:左PWM,右PWM；STOP 停车；ESTOP 急停；START 清除急停并恢复自动。"))

        direction_layout = QGridLayout()
        self.drive_forward_btn = QPushButton("前进")
        self.drive_left_btn = QPushButton("左转")
        self.drive_right_btn = QPushButton("右转")
        self.drive_back_btn = QPushButton("后退")
        self.drive_stop_btn = QPushButton("停止")
        self.drive_estop_btn = QPushButton("急停")
        self.drive_auto_btn = QPushButton("恢复自动")
        self.drive_start_btn = QPushButton("启动车辆")
        self.drive_arm_btn = QPushButton("解除急停")
        for button in (self.drive_forward_btn, self.drive_left_btn,
                       self.drive_right_btn, self.drive_back_btn):
            button.setMinimumHeight(36)
        direction_layout.addWidget(self.drive_forward_btn, 0, 1)
        direction_layout.addWidget(self.drive_left_btn, 1, 0)
        direction_layout.addWidget(self.drive_stop_btn, 1, 1)
        direction_layout.addWidget(self.drive_right_btn, 1, 2)
        direction_layout.addWidget(self.drive_back_btn, 2, 1)
        direction_layout.addWidget(self.drive_auto_btn, 1, 3)
        direction_layout.addWidget(self.drive_estop_btn, 3, 1)
        direction_layout.addWidget(self.drive_start_btn, 3, 0)
        direction_layout.addWidget(self.drive_arm_btn, 3, 2)
        manual_layout.addLayout(direction_layout)
        self.drive_forward_btn.pressed.connect(lambda: self._begin_manual_drive(1, 1))
        self.drive_left_btn.pressed.connect(lambda: self._begin_manual_drive(-1, 1))
        self.drive_right_btn.pressed.connect(lambda: self._begin_manual_drive(1, -1))
        self.drive_back_btn.pressed.connect(lambda: self._begin_manual_drive(-1, -1))
        for button in (self.drive_forward_btn, self.drive_left_btn,
                       self.drive_right_btn, self.drive_back_btn):
            button.released.connect(self._end_manual_drive)
        self.drive_stop_btn.clicked.connect(self._send_manual_stop)
        self.drive_estop_btn.clicked.connect(self._send_manual_estop)
        self.drive_auto_btn.clicked.connect(self._resume_auto)
        self.drive_start_btn.clicked.connect(self._start_selected_vehicle)
        self.drive_arm_btn.clicked.connect(self._arm_selected_vehicle)
        left_layout.addWidget(manual_group)

        # V2 比赛级控制（单车练习仍使用上方目标车控制）
        v2_match_group = QGroupBox("V2 对局控制")
        v2_match_layout = QGridLayout(v2_match_group)
        self.v2_prepare_btn = QPushButton("准备比赛")
        self.v2_prepare_btn.clicked.connect(self._prepare_v2_match)
        self.v2_start_btn = QPushButton("开始比赛")
        self.v2_start_btn.clicked.connect(self._start_v2_match)
        self.v2_pause_btn = QPushButton("暂停比赛")
        self.v2_pause_btn.clicked.connect(self._pause_v2_match)
        self.v2_resume_btn = QPushButton("恢复比赛")
        self.v2_resume_btn.clicked.connect(self._resume_v2_match)
        self.v2_finish_btn = QPushButton("结束比赛")
        self.v2_finish_btn.clicked.connect(self._finish_v2_match)
        self.v2_abort_btn = QPushButton("中止比赛")
        self.v2_abort_btn.clicked.connect(self._abort_v2_match)
        self.v2_reset_btn = QPushButton("故障复位")
        self.v2_reset_btn.clicked.connect(self._reset_v2_fault)
        self.v2_match_status = QLabel("LOBBY")
        v2_match_layout.addWidget(self.v2_prepare_btn, 0, 0)
        v2_match_layout.addWidget(self.v2_start_btn, 0, 1)
        v2_match_layout.addWidget(self.v2_pause_btn, 1, 0)
        v2_match_layout.addWidget(self.v2_resume_btn, 1, 1)
        v2_match_layout.addWidget(self.v2_finish_btn, 2, 0)
        v2_match_layout.addWidget(self.v2_abort_btn, 2, 1)
        v2_match_layout.addWidget(self.v2_reset_btn, 3, 0)
        v2_match_layout.addWidget(QLabel("平台状态:"), 3, 1)
        v2_match_layout.addWidget(self.v2_match_status, 3, 2)
        left_layout.addWidget(v2_match_group)

        # 接收数据显示区域
        receive_group = QGroupBox("接收数据")
        receive_layout = QVBoxLayout(receive_group)

        self.receive_text = QTextEdit()
        receive_layout.addWidget(self.receive_text)

        clear_receive_btn = QPushButton("清空接收区")
        clear_receive_btn.clicked.connect(lambda: self.receive_text.clear())
        receive_layout.addWidget(clear_receive_btn)

        left_layout.addWidget(receive_group)

        # 右侧面板 - 可视化
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # 创建标签页
        self.tab_widget = QTabWidget()

        # PID可视化标签页（双车各自独立数据，下拉切换显示车辆）
        pid_tab = QWidget()
        pid_layout = QVBoxLayout(pid_tab)
        pid_sel_row = QHBoxLayout()
        pid_sel_row.addWidget(QLabel("车辆:"))
        self.pid_car_combo = QComboBox()
        self.pid_car_combo.addItems(["A", "B"])
        pid_sel_row.addWidget(self.pid_car_combo)
        pid_sel_row.addStretch(1)
        pid_layout.addLayout(pid_sel_row)

        self.pid_widgets = {'A': PIDWidget(), 'B': PIDWidget()}
        self.pid_stack = QStackedWidget()
        self.pid_stack.addWidget(self.pid_widgets['A'])
        self.pid_stack.addWidget(self.pid_widgets['B'])
        self.pid_car_combo.currentIndexChanged.connect(self.pid_stack.setCurrentIndex)
        pid_layout.addWidget(self.pid_stack)
        self.tab_widget.addTab(pid_tab, "PID可视化")

        # 地图可视化标签页
        self.map_widget = MapWidget()
        self.tab_widget.addTab(self.map_widget, "地图可视化")

        right_layout.addWidget(self.tab_widget)

        # 地图控制
        map_control_group = QGroupBox("地图控制")
        map_control_layout = QHBoxLayout(map_control_group)

        load_map_btn = QPushButton("加载地图文件")
        load_map_btn.clicked.connect(self.load_map_file)
        map_control_layout.addWidget(load_map_btn)

        test_pos_btn = QPushButton("测试位置")
        test_pos_btn.clicked.connect(self.test_position)
        map_control_layout.addWidget(test_pos_btn)

        right_layout.addWidget(map_control_group)

        # 将左右面板添加到分割器
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([400, 800])

        # 状态灯轮询定时器（3 秒无 POS 判离线 + 转发暂停提示）
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._refresh_status_lights)
        self.status_timer.start(500)

    # ---------- 双车连接区 UI 构建 ----------
    def _build_car_group(self, tag):
        """构建某辆车的串口连接分组（端口下拉 + 刷新 + 波特率 + 连接/断开 + 状态灯）"""
        box = QGroupBox(f"Car {tag}")
        layout = QVBoxLayout(box)

        serial_row = QHBoxLayout()
        serial_row.addWidget(QLabel("串口:"))
        port_combo = QComboBox()
        for port in serial.tools.list_ports.comports():
            port_combo.addItem(port.device)
        serial_row.addWidget(port_combo)
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        serial_row.addWidget(refresh_btn)
        layout.addLayout(serial_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("波特率:"))
        baud_combo = QComboBox()
        baud_combo.addItems(["9600", "115200", "230400", "460800", "921600"])
        baud_combo.setCurrentText("115200")
        baud_row.addWidget(baud_combo)
        layout.addLayout(baud_row)

        protocol_row = QHBoxLayout()
        protocol_row.addWidget(QLabel("协议:"))
        protocol_combo = QComboBox()
        protocol_combo.addItems(["V1", "V2"])
        protocol_row.addWidget(protocol_combo)
        layout.addLayout(protocol_row)

        btn_row = QHBoxLayout()
        connect_btn = QPushButton(f"连接 Car {tag}")
        connect_btn.clicked.connect(lambda _=False, t=tag: self.connect_car(t))
        btn_row.addWidget(connect_btn)
        disconnect_btn = QPushButton("断开")
        disconnect_btn.clicked.connect(lambda _=False, t=tag: self.disconnect_car(t))
        disconnect_btn.setEnabled(False)
        btn_row.addWidget(disconnect_btn)
        status_label = QLabel("● 离线")
        status_label.setStyleSheet("color: #9E9E9E;")
        btn_row.addWidget(status_label)
        layout.addLayout(btn_row)

        self.car_ui[tag] = {
            'port': port_combo,
            'baud': baud_combo,
            'protocol': protocol_combo,
            'connect': connect_btn,
            'disconnect': disconnect_btn,
            'status': status_label,
        }
        return box

    # ---------- 日志（HTML 着色：A=蓝, B=红, 告警/转发另有配色） ----------
    def append_log(self, prefix, text, color=None):
        """日志区追加一行（HTML 着色）；prefix 形如 'A'，显示为 [A] ..."""
        if color is None:
            color = {'A': LOG_COLOR_A, 'B': LOG_COLOR_B}.get(prefix, LOG_COLOR_SYS)
        escaped = escape(str(text)).replace('\n', '<br>')
        self.receive_text.append(
            f'<span style="color:{color};">[{escape(str(prefix))}] {escaped}</span>'
        )

    # ---------- 串口连接管理（per-car） ----------
    def connect_car(self, tag):
        """连接某车串口"""
        ui = self.car_ui[tag]
        port = ui['port'].currentText()
        if not port:
            QMessageBox.warning(self, "警告", f"请选择 Car {tag} 串口")
            return
        protocol_mode = ui['protocol'].currentText()
        if self.ble_v2_role == tag:
            QMessageBox.warning(self, "连接冲突", f"Car {tag} 已由 BLE 链路承载")
            return
        try:
            baudrate = int(ui['baud'].currentText())
            self.links[tag].connect(port, baudrate)
            self.links[tag].encoding = self.current_encoding
            self.links[tag].protocol_mode = protocol_mode
            self.v2_routes[tag] = 'serial' if protocol_mode == "V2" else None
        except Exception as e:
            QMessageBox.critical(self, "错误", f"Car {tag} 连接失败: {str(e)}")
            return
        ui['connect'].setEnabled(False)
        ui['disconnect'].setEnabled(True)
        ui['port'].setEnabled(False)
        ui['baud'].setEnabled(False)
        ui['protocol'].setEnabled(False)
        self.append_log(tag, f"已连接 {port}，波特率 {baudrate}")

    def disconnect_car(self, tag):
        """断开某车串口"""
        ui = self.car_ui[tag]
        link = self.links[tag]
        if self.v2_routes[tag] == 'serial':
            self.v2_platform.disconnect(tag)
        self.v2_routes[tag] = None
        link.disconnect()
        link.protocol_mode = 'V1'
        link.pos = None
        link.rpos = None
        link.last_pos_time = 0.0
        # 重置该车防抖状态
        st = self.map_widget.car_state.get(tag)
        if st:
            st['pos_debounce']['pending_update'] = False
            st['rpos_debounce']['pending_update'] = False
        ui['connect'].setEnabled(True)
        ui['disconnect'].setEnabled(False)
        ui['port'].setEnabled(True)
        ui['baud'].setEnabled(True)
        ui['protocol'].setEnabled(True)
        ui['status'].setText("● 离线")
        ui['status'].setStyleSheet("color: #9E9E9E;")
        self._prev_online[tag] = False
        self.append_log(tag, "已断开串口连接", LOG_COLOR_GREY)

    def _refresh_status_lights(self):
        """刷新两车在线状态灯；由在线转离线时记录“转发暂停”日志"""
        now = time.time()
        for tag, link in self.links.items():
            if self.v2_routes[tag] is not None:
                session = self.v2_platform.sessions[tag]
                online = session.online
                if online and session.ready:
                    status_text = "● V2 READY"
                    status_style = "color: #2E7D32; font-weight: bold;"
                elif online:
                    status_text = "● V2 握手中"
                    status_style = "color: #EF6C00; font-weight: bold;"
                else:
                    status_text = "● 离线"
                    status_style = "color: #9E9E9E;"
            else:
                online = link.is_online(now)
                status_text = "● 在线" if online else "● 离线"
                status_style = "color: #2E7D32; font-weight: bold;" if online else "color: #9E9E9E;"
            ui = self.car_ui[tag]
            ui['status'].setText(status_text)
            ui['status'].setStyleSheet(status_style)
            if self._prev_online[tag] and not online:
                detail = "V2 心跳超时" if self.v2_routes[tag] is not None else "3 秒未收到 POS，转发暂停"
                self.append_log(tag, f"离线（{detail}）", LOG_COLOR_GREY)
            self._prev_online[tag] = online

    # ---------- V2 协议 / 比赛控制 ----------
    def _set_v2_mode(self, mode):
        try:
            self.v2_platform.set_mode(mode)
        except RuntimeError as exc:
            self.v2_mode_combo.blockSignals(True)
            self.v2_mode_combo.setCurrentText(self.v2_platform.mode)
            self.v2_mode_combo.blockSignals(False)
            QMessageBox.warning(self, "模式切换被拒绝", str(exc))

    def _send_v2_bytes(self, car_id, data):
        route = self.v2_routes.get(car_id)
        if route == 'serial':
            link = self.links[car_id]
            try:
                return link.send_line(data.decode('ascii')) if link.is_open else False
            except UnicodeDecodeError:
                return False
        if route == 'ble' and self.ble_v2_role == car_id:
            worker = self.bluetooth_connection_worker
            if self.is_bluetooth_connected and worker is not None:
                return worker.send_data(data.decode('ascii'))
        return False

    def _on_v2_event(self, event, car_id, detail):
        color = LOG_COLOR_WARN if event in {
            "FRAME_REJECTED", "STALE_SESSION", "STALE_SEQUENCE", "SEND_FAILED",
            "ACK_TIMEOUT", "MATCH_FAULT", "OFFLINE", "NACK", "WELCOME_FAILED",
        } else LOG_COLOR_SYS
        self.append_log(car_id, f"V2 {event}: {detail}" if detail else f"V2 {event}", color)

    def _tick_v2(self):
        self.v2_platform.tick()
        if hasattr(self, 'v2_match_status'):
            state = self.v2_platform.match_state
            self.v2_match_status.setText(state)
            self.v2_match_status.setStyleSheet(
                "color: #C62828; font-weight: bold;" if state == "FAULT" else "font-weight: bold;"
            )

    def _on_v2_frame(self, car_id, frame):
        if frame.type == "POSE":
            x = int(frame.fields[0]) / 10.0
            y = int(frame.fields[1]) / 10.0
            self.links[car_id].pos = (x, y)
            self.links[car_id].note_pos()
            self.map_widget.update_car_position(car_id, x, y)
            return
        if frame.type == "CLAIM":
            self._judge_v2_claim(car_id, frame)
            return
        if frame.type == "EVENT":
            self.append_log(car_id, f"EVENT {frame.fields[0]}={frame.fields[1]}")
            return
        if frame.type in {"SENSOR", "SPEED", "OPPONENT"}:
            self.append_log(car_id, f"{frame.type} {','.join(frame.fields)}")

    def _judge_v2_claim(self, car_id, frame):
        x10, y10 = int(frame.fields[0]), int(frame.fields[1])
        x, y = x10 / 10.0, y10 / 10.0
        link = self.links[car_id]
        self.map_widget.update_car_report(car_id, x, y)
        reason = "OK"
        accepted = False
        graph = self.map_widget.graph
        if self.v2_platform.mode != "OFFICIAL" or self.v2_platform.match_state != "RUNNING":
            reason = "BAD_STATE"
        elif not graph or not graph.get('nodes') or not graph.get('adj'):
            reason = "MAP_UNAVAILABLE"
        elif time.time() - link.last_pos_time > 1.0:
            reason = "POSE_STALE"
        else:
            current_node = nearest_node(link.pos, graph.get('nodes'))
            claim_node = nearest_node((x, y), graph.get('nodes'))
            distance = graph_dist(current_node, claim_node, graph.get('adj'))
            if distance is None:
                reason = "UNMAPPED"
            elif distance > RPOS_MAX_HOPS:
                reason = "TOO_FAR"
            else:
                accepted = True

        verdict = (f"{frame.seq:08X}", "ACCEPT" if accepted else "REJECT", reason)
        self.v2_platform.send_message(car_id, "VERDICT", verdict, reliable=False)
        if accepted:
            other = 'B' if car_id == 'A' else 'A'
            opponent = self.v2_platform.sessions[other]
            if opponent.ready and opponent.online:
                self.v2_platform.send_message(
                    other,
                    "OPPONENT",
                    (car_id, str(x10), str(y10), f"{frame.seq:08X}", "0"),
                    reliable=False,
                )
        self.append_log(
            car_id,
            f"V2 CLAIM ({x:.1f},{y:.1f}) {'接受' if accepted else '拒绝'}: {reason}",
            LOG_COLOR_SYS if accepted else LOG_COLOR_WARN,
        )

    def _v2_tag_for_target(self, target):
        if target in ('A', 'B') and self.v2_routes.get(target) is not None:
            return target
        if target == '蓝牙' and self.ble_v2_role is not None:
            return self.ble_v2_role
        return None

    def _prepare_v2_match(self):
        if not self.v2_platform.prepare_official():
            self.append_log('SYS', "V2 准备比赛失败；请检查 OFFICIAL 模式、地图和两车 READY", LOG_COLOR_WARN)

    def _start_v2_match(self):
        if not self.v2_platform.start_match():
            self.append_log('SYS', "V2 开始比赛失败；需处于 PREPARING 且两车在线/未急停", LOG_COLOR_WARN)

    def _pause_v2_match(self):
        if not self.v2_platform.pause_match():
            self.append_log('SYS', "V2 暂停请求未发送；当前状态不是 RUNNING", LOG_COLOR_WARN)

    def _resume_v2_match(self):
        if not self.v2_platform.resume_match():
            self.append_log('SYS', "V2 恢复失败；需两车在线、READY 且先解除急停", LOG_COLOR_WARN)

    def _finish_v2_match(self):
        if not self.v2_platform.finish_match():
            self.append_log('SYS', "V2 结束比赛失败；当前没有可结束的官方对局", LOG_COLOR_WARN)

    def _abort_v2_match(self):
        if not self.v2_platform.finish_match(abort=True):
            self.append_log('SYS', "V2 中止比赛失败；当前没有可中止的官方对局", LOG_COLOR_WARN)

    def _reset_v2_fault(self):
        if not self.v2_platform.reset_fault():
            self.append_log('SYS', "故障复位失败；请先确认车辆已停车且链路/状态已恢复", LOG_COLOR_WARN)

    def _start_selected_vehicle(self):
        target = self.send_target_combo.currentText()
        car_id = self._v2_tag_for_target(target)
        if car_id:
            sent = self.v2_platform.start_bench_vehicle(car_id) is not None
            if not sent:
                self.append_log(car_id, "单车启动仅限 BENCH，且车辆必须 READY/已解除急停", LOG_COLOR_WARN)
            return
        if self._send_control_line("START", target):
            self.append_log(target, "发送启动/恢复自动导航命令", LOG_COLOR_SYS)
        else:
            QMessageBox.warning(self, "警告", f"控制目标 {target} 未连接或不可写")

    def _arm_selected_vehicle(self):
        target = self.send_target_combo.currentText()
        car_id = self._v2_tag_for_target(target)
        if car_id:
            if self.v2_platform.arm_vehicle(car_id) is None:
                self.append_log(car_id, "解除急停失败；车辆需停车且不处于 RUNNING", LOG_COLOR_WARN)
            return
        QMessageBox.information(self, "V1 控制", "当前 V1 使用 START 清除急停；独立 ARM 仅适用于 V2。")

    def update_encoding(self, text):
        """更新解码方式（两路串口共用）"""
        encoding_map = {
            "UTF-8": "utf-8",
            "GBK": "gbk",
            "ASCII": "ascii"
        }
        self.current_encoding = encoding_map.get(text, "utf-8")
        for link in self.links.values():
            link.encoding = self.current_encoding

    def refresh_ports(self):
        """刷新两路串口下拉的可用串口列表"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        for ui in self.car_ui.values():
            current = ui['port'].currentText()
            ui['port'].clear()
            ui['port'].addItems(ports)
            if current in ports:
                ui['port'].setCurrentText(current)

    def scan_bluetooth_devices(self):
        """扫描蓝牙设备"""
        if not BLUETOOTH_SUPPORTED:
            QMessageBox.warning(self, "警告", "未安装蓝牙支持库，请运行: pip install bleak")
            return

        self.scan_bluetooth_btn.setEnabled(False)
        self.scan_bluetooth_btn.setText("扫描中...")

        self.bluetooth_scan_worker = BluetoothScanWorker()
        self.bluetooth_scan_worker.devices_found.connect(self.on_bluetooth_devices_found)
        self.bluetooth_scan_worker.error_occurred.connect(self.on_bluetooth_error)
        self.bluetooth_scan_worker.finished.connect(self.on_bluetooth_scan_finished)
        self.bluetooth_scan_worker.start()

    def on_bluetooth_devices_found(self, devices):
        """蓝牙设备扫描完成"""
        if not devices:
            QMessageBox.information(self, "提示", "未发现蓝牙设备")
            return

        # 显示设备选择对话框
        dialog = DeviceSelectionDialog(devices, self)
        if dialog.exec_() == QDialog.Accepted and dialog.selected_device:
            self.selected_bluetooth_device = dialog.selected_device
            self.bluetooth_connect_btn.setEnabled(True)
            self.receive_text.append(f"已选择设备: {dialog.selected_device['name']}")

    def on_bluetooth_error(self, error):
        """蓝牙扫描出错"""
        QMessageBox.critical(self, "错误", f"蓝牙扫描失败: {error}")

    def on_bluetooth_scan_finished(self):
        """蓝牙扫描完成"""
        self.scan_bluetooth_btn.setEnabled(True)
        self.scan_bluetooth_btn.setText("扫描蓝牙设备")

    def connect_bluetooth_device(self):
        """连接选中的蓝牙设备"""
        if not hasattr(self, 'selected_bluetooth_device'):
            QMessageBox.warning(self, "警告", "请先选择蓝牙设备")
            return

        self.bluetooth_protocol_mode = self.bluetooth_protocol_combo.currentText()
        if self.bluetooth_protocol_mode == "V2":
            role = self.bluetooth_role_combo.currentText()
            if self.v2_routes[role] is not None or self.links[role].is_open:
                QMessageBox.warning(self, "连接冲突", f"Car {role} 已绑定其他链路")
                return
            self.ble_v2_role = role
            self.v2_routes[role] = 'ble'

        self.bluetooth_connect_btn.setEnabled(False)
        self.bluetooth_connect_btn.setText("连接中...")
        self.bluetooth_protocol_combo.setEnabled(False)
        self.bluetooth_role_combo.setEnabled(False)

        device_address = self.selected_bluetooth_device['address']
        self.bluetooth_connection_worker = BluetoothConnectionWorker(device_address)
        self.bluetooth_connection_worker.connection_status.connect(self.on_bluetooth_connection_status)
        self.bluetooth_connection_worker.data_received.connect(self.on_bluetooth_data_received)
        self.bluetooth_connection_worker.disconnected.connect(self.on_bluetooth_disconnected)
        self.bluetooth_connection_worker.services_found.connect(self.on_services_found)
        self.bluetooth_connection_worker.start()

    def on_services_found(self, services):
        """接收服务信息"""
        self.services = services

    def search_uuid(self):
        """搜索指定的UUID"""
        if not self.is_bluetooth_connected or not self.services:
            QMessageBox.warning(self, "警告", "未连接蓝牙设备或未获取服务信息")
            return

        target_uuid = self.uuid_input.text().strip().lower()
        if not target_uuid:
            QMessageBox.warning(self, "警告", "请输入UUID")
            return

        found = False
        result = "找到的匹配UUID:\n"
        for service in self.services:
            for char in service.characteristics:
                if char.uuid.lower() == target_uuid:
                    found = True
                    result += f"服务: {service.uuid}, 特征: {char.uuid}\n"

        if found:
            self.receive_text.append(result)
        else:
            self.receive_text.append(f"未找到UUID: {target_uuid}")

    def on_bluetooth_connection_status(self, connected, message):
        """蓝牙连接状态更新"""
        if connected:
            self.is_bluetooth_connected = True
            self.bluetooth_connect_btn.setText("已连接")
            self.bluetooth_connect_btn.setEnabled(False)
            self.receive_text.append(f"蓝牙连接成功: {message}")
        else:
            self.is_bluetooth_connected = False
            self.bluetooth_connect_btn.setText("蓝牙连接")
            self.bluetooth_connect_btn.setEnabled(True)
            if self.ble_v2_role is not None:
                self.v2_platform.disconnect(self.ble_v2_role)
                self.v2_routes[self.ble_v2_role] = None
                self.ble_v2_role = None
            self.bluetooth_protocol_combo.setEnabled(True)
            self.bluetooth_role_combo.setEnabled(True)
            self.receive_text.append(f"蓝牙连接失败: {message}")

    def on_bluetooth_data_received(self, data):
        """接收蓝牙数据；V2 按绑定角色进入统一对战会话。"""
        self.append_log('蓝牙', data)
        if self.bluetooth_protocol_mode == "V2" and self.ble_v2_role is not None:
            self.v2_platform.receive_line(self.ble_v2_role, data)

    def on_bluetooth_disconnected(self):
        """蓝牙断开连接"""
        if self.held_manual_target == '蓝牙':
            self._end_manual_drive(send_stop=False)
        if self.ble_v2_role is not None:
            self.links[self.ble_v2_role].pos = None
            self.links[self.ble_v2_role].rpos = None
            self.links[self.ble_v2_role].last_pos_time = 0.0
            self.v2_platform.disconnect(self.ble_v2_role)
            self.v2_routes[self.ble_v2_role] = None
            self.ble_v2_role = None
        self.is_bluetooth_connected = False
        self.bluetooth_connect_btn.setText("蓝牙连接")
        self.bluetooth_connect_btn.setEnabled(True)
        if hasattr(self, 'bluetooth_protocol_combo'):
            self.bluetooth_protocol_combo.setEnabled(True)
            self.bluetooth_role_combo.setEnabled(True)
        self.receive_text.append("蓝牙连接已断开")

    def send_data(self):
        """按所选目标发送数据（Car A / Car B / 蓝牙）"""
        target = self.send_target_combo.currentText()
        if self._v2_tag_for_target(target) is not None:
            QMessageBox.warning(self, "V2 控制", "V2 链路不允许发送原始文本；请使用平台协议控制按钮。")
            return
        try:
            data = self.send_text.toPlainText()
            # 确保发送的数据以\n结尾
            if not data.endswith('\n'):
                data += '\n'
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送失败: {str(e)}")
            return

        if target in ('A', 'B'):
            link = self.links[target]
            if not link.is_open:
                QMessageBox.warning(self, "警告", f"Car {target} 未连接")
                return
            if link.send_line(data):
                self.append_log(target, f"发送: {data.strip()}", LOG_COLOR_SYS)
            else:
                QMessageBox.critical(self, "错误", f"Car {target} 发送失败")
        else:
            if not self.is_bluetooth_connected or not self.bluetooth_connection_worker:
                QMessageBox.warning(self, "警告", "未连接蓝牙设备")
                return
            self.bluetooth_connection_worker.send_data(data)
            self.append_log('蓝牙', f"发送: {data.strip()}")

    def _send_control_line(self, command, target=None):
        target = target or self.send_target_combo.currentText()
        v2_car = self._v2_tag_for_target(target)
        if v2_car is not None:
            if command.startswith("DRV:"):
                try:
                    left_pwm, right_pwm = command[4:].split(",", 1)
                    seq = self.v2_platform.send_command(
                        v2_car, "DRIVE", str(int(left_pwm)), str(int(right_pwm)), str(DRIVE_LEASE_MS)
                    )
                    return seq is not None
                except (ValueError, TypeError):
                    return False
            if command == "STOP":
                if self.v2_platform.mode == "OFFICIAL" and self.v2_platform.match_state == "RUNNING":
                    return self.v2_platform.pause_match("SINGLE_CAR_STOP")
                return self.v2_platform.stop_vehicle(v2_car) is not None
            if command == "ESTOP":
                return self.v2_platform.emergency_stop(v2_car)
            if command == "START":
                return self.v2_platform.start_bench_vehicle(v2_car) is not None
            return False
        data = f"{command}\n"
        if target in ('A', 'B'):
            link = self.links[target]
            return link.send_line(data) if link.is_open else False
        if target == '蓝牙':
            if not self.is_bluetooth_connected or not self.bluetooth_connection_worker:
                return False
            return self.bluetooth_connection_worker.send_data(data)
        return False

    def _begin_manual_drive(self, left_sign, right_sign):
        if self.held_manual_pwm is not None:
            self._end_manual_drive()
        target = self.send_target_combo.currentText()
        speed = self.drive_speed_input.value()
        pwm = (left_sign * speed, right_sign * speed)
        if not self._send_control_line(f"DRV:{pwm[0]},{pwm[1]}", target):
            QMessageBox.warning(self, "警告", f"控制目标 {target} 未连接或不可写")
            return
        self.held_manual_pwm = pwm
        self.held_manual_target = target
        self.manual_drive_timer.start()
        self.append_log(target, f"直接控制: 左轮 {pwm[0]} / 右轮 {pwm[1]}", LOG_COLOR_SYS)

    def _repeat_manual_drive(self):
        if self.held_manual_pwm is None:
            self.manual_drive_timer.stop()
            return
        left_pwm, right_pwm = self.held_manual_pwm
        if not self._send_control_line(f"DRV:{left_pwm},{right_pwm}", self.held_manual_target):
            target = self.held_manual_target
            self._end_manual_drive(send_stop=False)
            self.append_log(target, "控制链路已断开，停止续发", LOG_COLOR_WARN)

    def _end_manual_drive(self, send_stop=True):
        self.manual_drive_timer.stop()
        target = self.held_manual_target
        was_driving = self.held_manual_pwm is not None
        self.held_manual_pwm = None
        self.held_manual_target = None
        if send_stop and was_driving and target:
            self._send_control_line("DRV:0,0", target)
            self.append_log(target, "直接控制松开：停车", LOG_COLOR_SYS)

    def _send_manual_stop(self):
        self._end_manual_drive(send_stop=False)
        target = self.send_target_combo.currentText()
        if self._send_control_line("STOP", target):
            self.append_log(target, "发送停车命令", LOG_COLOR_SYS)
        else:
            QMessageBox.warning(self, "警告", f"控制目标 {target} 未连接或不可写")

    def _send_manual_estop(self):
        self._end_manual_drive(send_stop=False)
        target = self.send_target_combo.currentText()
        if self._send_control_line("ESTOP", target):
            v2_car = self._v2_tag_for_target(target)
            reset_hint = "需 ARM 清除" if v2_car is not None else "需 START 清除"
            self.append_log(target, f"发送急停命令（{reset_hint}）", LOG_COLOR_WARN)
        else:
            QMessageBox.warning(self, "警告", f"控制目标 {target} 未连接或不可写")

    def _resume_auto(self):
        self._end_manual_drive(send_stop=False)
        target = self.send_target_combo.currentText()
        v2_car = self._v2_tag_for_target(target)
        if v2_car is not None:
            if self.v2_platform.send_command(v2_car, "MODE", "AUTO") is None:
                self.append_log(v2_car, "切换自动模式失败；检查 V2 会话和比赛状态", LOG_COLOR_WARN)
            return
        if self._send_control_line("START", target):
            self.append_log(target, "发送启动/恢复自动导航命令", LOG_COLOR_SYS)
        else:
            QMessageBox.warning(self, "警告", f"控制目标 {target} 未连接或不可写")

    # ---------- 对战协议 v1：行处理 / 裁判 / 转发 ----------
    def on_car_line(self, tag, line):
        """CarLink 收到一条完整行：入日志（[A]蓝 / [B]红）并解析"""
        self.append_log(tag, line)
        self.process_line(tag, line)

    def process_line(self, src, line):
        """按对战协议 v1 解析某车上报的一行数据（src: 'A'/'B'），双车数据互不串仓。

        先 startswith 校验前缀，再按 len(前缀) 切片；每个消息 handler 独立
        try/except，异常只记日志，不影响其他消息与读取循环。
        """
        try:
            line = line.strip()
            if not line:
                return
            link = self.links.get(src)
            if link is None:
                return

            if self.v2_routes.get(src) == 'serial':
                if not line.startswith("@2,"):
                    self.append_log(src, "V2 链路收到非 V2 帧，已丢弃", LOG_COLOR_WARN)
                    return
                self.v2_platform.receive_line(src, line)
                return

            if line.startswith("PID:"):
                # 解析PID数据 (格式: PID:电机ID,实际值,目标值)
                try:
                    pid_parts = line[len("PID:"):].split(",")
                    if len(pid_parts) >= 2:
                        motor_id = int(pid_parts[0])
                        pid_value = float(pid_parts[1])
                        target_pid_value = 0  # 默认目标值为0
                        if len(pid_parts) >= 3:
                            target_pid_value = float(pid_parts[2])
                        self.pid_widgets[src].update_pid_data(motor_id, pid_value, target_pid_value)
                except Exception as e:
                    print(f"[{src}] PID 消息处理错误: {e}")

            elif line.startswith("RPOS:"):
                # RPOS:上报位置（谎报），经裁判校验后决定转发或拒收
                try:
                    rpos_parts = line[len("RPOS:"):].split(",")
                    if len(rpos_parts) >= 2:
                        x_str = rpos_parts[0].strip()
                        y_str = rpos_parts[1].strip()
                        self._handle_rpos(src, x_str, y_str)
                except Exception as e:
                    print(f"[{src}] RPOS 消息处理错误: {e}")

            elif line.startswith("POS:"):
                # POS:实际位置（仅用于裁判校验与真实位置显示，绝不转发给对手）
                try:
                    pos_parts = line[len("POS:"):].split(",")
                    if len(pos_parts) >= 2:
                        x = float(pos_parts[0])
                        y = float(pos_parts[1])
                        link.pos = (x, y)
                        link.note_pos()
                        self.map_widget.update_car_position(src, x, y)
                except Exception as e:
                    print(f"[{src}] POS 消息处理错误: {e}")

            elif line.startswith("ENEMY:"):
                # 旧协议字段：前缀校验后按前缀长度切片，仅记录不接入对战通道
                try:
                    enemy_parts = line[len("ENEMY:"):].split(",")
                    if len(enemy_parts) >= 2:
                        x = float(enemy_parts[0])
                        y = float(enemy_parts[1])
                        self.append_log(src, f"[解析] ENEMY {x},{y}（旧协议字段，未接入对战通道）",
                                        LOG_COLOR_GREY)
                except Exception as e:
                    print(f"[{src}] ENEMY 消息处理错误: {e}")

            elif line.startswith("YAW:"):
                # 解析偏航角数据 (格式: YAW:实际值,目标值)
                try:
                    yaw_parts = line[len("YAW:"):].split(",")
                    yaw_value = yaw_parts[0]
                    target_yaw_value = 0  # 默认目标值为0
                    if len(yaw_parts) >= 2:
                        target_yaw_value = yaw_parts[1]
                    yaw_value = float(yaw_value)
                    target_yaw_value = float(target_yaw_value)
                    self.pid_widgets[src].update_yaw_data(yaw_value, target_yaw_value)
                except ValueError:
                    print(f"无效的偏航角数据: {yaw_parts[0] if yaw_parts else line}")
                except Exception as e:
                    print(f"[{src}] YAW 消息处理错误: {e}")

            elif line.startswith("track_data:"):
                try:
                    # 提取 track_data 前缀后的数值（前缀校验后按前缀长度切片）
                    value = int(line[len("track_data:"):])
                    # 转换为16位二进制字符串，然后只取后8位
                    binary_str = format(value, '016b')
                    last_8_bits = binary_str[8:]
                    formatted_binary = ' '.join([last_8_bits[i:i + 4] for i in range(0, len(last_8_bits), 4)])
                    self.append_log(src, f"[解析] track_data: {value} => 后8位二进制: {formatted_binary}")
                except ValueError:
                    self.append_log(src, f"[错误] 无法解析track_data: {line}", LOG_COLOR_WARN)
                except Exception as e:
                    print(f"[{src}] track_data 消息处理错误: {e}")

        except Exception as e:
            print(f"解析数据错误: {e}")

    def _handle_rpos(self, src, x_str, y_str):
        """裁判 + 转发（对战协议 v1 核心）；x_str/y_str 为上报原文坐标"""
        link = self.links[src]
        x = float(x_str)
        y = float(y_str)
        link.rpos = (x, y)
        self.map_widget.update_car_report(src, x, y)
        graph = getattr(self.map_widget, 'graph', None)
        ok, d, degraded = judge_rpos_ex(link.pos, (x, y), graph)
        other = 'B' if src == 'A' else 'A'

        if degraded:
            self.append_log(src, f"RPOS 节点匹配失败，按欧氏距离估算 {d:.2f} 跳，放行",
                            LOG_COLOR_WARN)

        if not ok:
            # 超限：回发 ERR:RPOS,<d> 给上报车，不转发，红色告警
            link.send_line(f"ERR:RPOS,{d}")
            self.append_log(src,
                            f"RPOS 超限：图距离 {d} > {RPOS_MAX_HOPS} 跳，"
                            f"已回发 ERR:RPOS,{d}，未转发",
                            LOG_COLOR_WARN)
            return

        # 通过：生成 ORP:x,y 发给另一辆车的串口
        if not link.is_online():
            self.append_log(src, "RPOS 校验通过，但该车离线（3 秒无 POS），暂停转发",
                            LOG_COLOR_GREY)
            return
        other_link = self.links[other]
        if not other_link.is_open:
            self.append_log(src, f"RPOS 校验通过，但 Car {other} 未连接，未转发",
                            LOG_COLOR_GREY)
            return
        if other_link.send_line(f"ORP:{x_str},{y_str}"):
            self.append_log(src, f"RPOS 转发给 {other}", LOG_COLOR_SYS)
        else:
            self.append_log(src, f"RPOS 转发给 {other} 失败", LOG_COLOR_WARN)

    def load_map_file(self):
        """加载地图文件"""
        if self.v2_platform.match_state in {"PREPARING", "RUNNING", "PAUSED"}:
            QMessageBox.warning(self, "地图锁定", "比赛进行中不能更换地图。")
            return
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择地图文件", "", "C Files (*.c);;All Files (*)")

        if file_path:
            if self.map_widget.load_map_from_c_file(file_path):
                self.v2_platform.set_map_version(self.map_widget.map_version)
                QMessageBox.information(self, "成功", "地图文件加载成功")
            else:
                QMessageBox.critical(self, "错误", "地图文件加载失败")

    def test_position(self):
        """测试位置更新（A/B 各生成真实位置与上报位置，含谎报偏差）"""
        # 随机生成一个测试位置
        import random
        for tag in ('A', 'B'):
            x = random.uniform(0, 1000)
            y = random.uniform(0, 1000)
            link = self.links[tag]
            link.pos = (x, y)
            link.note_pos()
            self.map_widget.update_car_position(tag, x, y)

            # 上报位置（谎报）：带随机偏差
            rx = x + random.uniform(-600, 600)
            ry = y + random.uniform(-600, 600)
            link.rpos = (rx, ry)
            self.map_widget.update_car_report(tag, rx, ry)

            self.append_log(tag, f"测试位置: ({x:.2f}, {y:.2f})，上报: ({rx:.2f}, {ry:.2f})")

    def closeEvent(self, event):
        """关窗清理：停止状态灯定时器与两路串口，避免关窗崩溃"""
        self._end_manual_drive()
        if self.v2_platform.mode == "OFFICIAL" and self.v2_platform.match_state in {
            "PREPARING", "RUNNING", "PAUSED",
        }:
            self.v2_platform.finish_match(abort=True)
        for car_id, session in self.v2_platform.sessions.items():
            if session.ready:
                self.v2_platform.send_command(car_id, "STOP")
        try:
            self.v2_timer.stop()
        except Exception:
            pass
        try:
            self.status_timer.stop()
        except Exception:
            pass
        for tag, link in self.links.items():
            try:
                link.close()
            except Exception as e:
                print(f"关闭 Car {tag} 串口错误: {e}")
        if self.bluetooth_connection_worker is not None:
            try:
                self.bluetooth_connection_worker.stop()
            except Exception:
                pass
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BluetoothDebugTool()
    window.show()
    sys.exit(app.exec_())
