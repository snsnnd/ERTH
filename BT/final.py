import sys
import json
import re
import struct
import serial
import time
from html import escape
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QComboBox, QTextEdit, QLabel,
                             QGroupBox, QTabWidget, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QFileDialog, QMessageBox, QSplitter,
                             QProgressDialog, QListWidget, QDialog, QDialogButtonBox,
                             QLineEdit, QStackedWidget)
from PyQt5.QtCore import QTimer, Qt, QPointF, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor
import pyqtgraph as pg
from pyqtgraph import PlotWidget, PlotItem

# 添加蓝牙支持（需要安装 bleak 库）
try:
    from bleak import BleakScanner, BleakClient

    BLUETOOTH_SUPPORTED = True
except ImportError:
    BLUETOOTH_SUPPORTED = False
    print("未安装 bleak 库，蓝牙功能不可用。请运行: pip install bleak")

import asyncio


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
            line = raw.decode(self.encoding, errors='ignore').rstrip('\r')
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
        self.buffer = ""  # 用于存储未完成的数据

    async def _connect_and_listen(self):
        """连接蓝牙设备并监听数据"""
        try:
            self.client = BleakClient(self.device_address)
            await self.client.connect()

            if not self.client.is_connected:
                self.connection_status.emit(False, "连接失败")
                return

            self.connection_status.emit(True, "连接成功")
            self.is_running = True

            # 获取服务和特征信息（使用新API）
            try:
                services = self.client.services  # 新的API
                self.services_found.emit(services)  # 发送服务
                print("可用服务:")
                for service in services:
                    print(f"  Service: {service.uuid}")
                    for char in service.characteristics:
                        print(f"    Characteristic: {char.uuid}")
            except Exception as e:
                print(f"获取服务信息失败: {e}")
                # 备用方法：尝试 discover_services
                try:
                    services = await self.client.get_services()
                    self.services_found.emit(services)  # 发送服务
                    print("可用服务:")
                    for service in services:
                        print(f"  Service: {service.uuid}")
                        for char in service.characteristics:
                            print(f"    Characteristic: {char.uuid}")
                except Exception as e2:
                    print(f"备用方法也失败: {e2}")

            # 轮询读取数据（替代通知机制）
            while self.is_running and self.client.is_connected:
                try:
                    # 示例读取（需要替换为实际特征UUID）
                    # data = await self.client.read_gatt_char(self.read_characteristic)
                    # self.process_data(data.decode('utf-8'))
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

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._connect_and_listen())
        loop.close()

    def stop(self):
        self.is_running = False

    def send_data(self, data):
        """发送数据到蓝牙设备"""
        if self.client and self.client.is_connected and self.write_characteristic:
            # 确保发送的数据以\n结尾
            if isinstance(data, str) and not data.endswith('\n'):
                data += '\n'
            asyncio.create_task(self._send_data_async(data))

    async def _send_data_async(self, data):
        """异步发送数据"""
        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            await self.client.write_gatt_char(self.write_characteristic, data)
        except Exception as e:
            print(f"发送数据错误: {e}")


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
        self._prev_online = {'A': False, 'B': False}

        # 初始化 UI
        self.init_ui()

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
        try:
            baudrate = int(ui['baud'].currentText())
            self.links[tag].connect(port, baudrate)
            self.links[tag].encoding = self.current_encoding
        except Exception as e:
            QMessageBox.critical(self, "错误", f"Car {tag} 连接失败: {str(e)}")
            return
        ui['connect'].setEnabled(False)
        ui['disconnect'].setEnabled(True)
        ui['port'].setEnabled(False)
        ui['baud'].setEnabled(False)
        self.append_log(tag, f"已连接 {port}，波特率 {baudrate}")

    def disconnect_car(self, tag):
        """断开某车串口"""
        ui = self.car_ui[tag]
        link = self.links[tag]
        link.disconnect()
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
        ui['status'].setText("● 离线")
        ui['status'].setStyleSheet("color: #9E9E9E;")
        self._prev_online[tag] = False
        self.append_log(tag, "已断开串口连接", LOG_COLOR_GREY)

    def _refresh_status_lights(self):
        """刷新两车在线状态灯；由在线转离线时记录“转发暂停”日志"""
        now = time.time()
        for tag, link in self.links.items():
            online = link.is_online(now)
            ui = self.car_ui[tag]
            if online:
                ui['status'].setText("● 在线")
                ui['status'].setStyleSheet("color: #2E7D32; font-weight: bold;")
            else:
                ui['status'].setText("● 离线")
                ui['status'].setStyleSheet("color: #9E9E9E;")
            if self._prev_online[tag] and not online:
                self.append_log(tag, "离线（3 秒未收到 POS），转发暂停", LOG_COLOR_GREY)
            self._prev_online[tag] = online

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

        self.bluetooth_connect_btn.setEnabled(False)
        self.bluetooth_connect_btn.setText("连接中...")

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
            self.receive_text.append(f"蓝牙连接失败: {message}")

    def on_bluetooth_data_received(self, data):
        """接收蓝牙数据（仅入日志，不接入对战数据通道，避免与车载串口数据串仓）"""
        self.append_log('蓝牙', data)

    def on_bluetooth_disconnected(self):
        """蓝牙断开连接"""
        self.is_bluetooth_connected = False
        self.bluetooth_connect_btn.setText("蓝牙连接")
        self.bluetooth_connect_btn.setEnabled(True)
        self.receive_text.append("蓝牙连接已断开")

    def send_data(self):
        """按所选目标发送数据（Car A / Car B / 蓝牙）"""
        target = self.send_target_combo.currentText()
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
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择地图文件", "", "C Files (*.c);;All Files (*)")

        if file_path:
            if self.map_widget.load_map_from_c_file(file_path):
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
