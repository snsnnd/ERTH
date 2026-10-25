import sys
import json
import re
import struct
import serial
import time
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QComboBox, QTextEdit, QLabel,
                             QGroupBox, QTabWidget, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QFileDialog, QMessageBox, QSplitter,
                             QProgressDialog, QListWidget, QDialog, QDialogButtonBox,
                             QLineEdit)
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
# 重构 MapWidget 类
class MapWidget(QWidget):
    # 在 MapWidget 的 __init__ 方法中添加敌人相关属性
    def __init__(self):
        super().__init__()
        self.nodes = []
        self.edges = []
        self.current_pos = None
        self.enemy_pos = None  # 添加这一行
        self._current_pos_item = None
        self._enemy_pos_item = None  # 添加这一行
        self.last_update_time = 0
        self.update_interval = 100
        self.pending_update = False
        self.init_ui()

    # 在 MapWidget 类中添加危险节点更新方法
    def update_danger_nodes(self, danger_nodes_list):
        """更新危险节点列表并可视化显示"""
        self.danger_nodes = danger_nodes_list
        self.update_danger_nodes_marker()

    def update_enemy_position(self, x, y):
        """更新敌人位置（带防抖）"""
        import time
        current_time = time.time() * 1000  # 转换为毫秒

        # 更新敌人位置数据
        self.enemy_pos = {'x': x, 'y': y}

        # 检查是否需要立即更新
        if current_time - self.last_update_time > self.update_interval:
            self.last_update_time = current_time
            self.update_enemy_position_marker()
        else:
            # 设置延迟更新标志
            if not self.pending_update:
                self.pending_update = True
                QTimer.singleShot(self.update_interval, self.delayed_update_enemy_position)

    def update_enemy_position_marker(self):
        """只更新敌人位置标记，避免重绘整个地图"""
        # 移除旧的敌人位置标记（如果存在）
        if hasattr(self, '_enemy_pos_item') and self._enemy_pos_item:
            self.plot_widget.removeItem(self._enemy_pos_item)

        # 添加新的敌人位置标记
        if self.enemy_pos:
            self._enemy_pos_item = pg.PlotDataItem(
                [self.enemy_pos['x']], [self.enemy_pos['y']],
                pen=None, symbol='o', symbolSize=15, symbolBrush='r'  # 红色标记敌人
            )
            self.plot_widget.addItem(self._enemy_pos_item)

    def delayed_update_enemy_position(self):
        """延迟更新敌人位置"""
        if self.pending_update and self.enemy_pos:
            self.last_update_time = time.time() * 1000
            self.update_enemy_position_marker()
        self.pending_update = False

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

        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    # 其他方法保持不变...
    def update_position(self, x, y):
        """更新当前位置（带防抖）"""
        import time
        current_time = time.time() * 1000  # 转换为毫秒

        # 更新位置数据
        self.current_pos = {'x': x, 'y': y}

        # 检查是否需要立即更新
        if current_time - self.last_update_time > self.update_interval:
            self.last_update_time = current_time
            self.update_current_position_marker()
        else:
            # 设置延迟更新标志
            if not self.pending_update:
                self.pending_update = True
                QTimer.singleShot(self.update_interval, self.delayed_update_position)

    def update_current_position_marker(self):
        """只更新当前位置标记，避免重绘整个地图"""
        # 移除旧的位置标记（如果存在）
        if hasattr(self, '_current_pos_item') and self._current_pos_item:
            self.plot_widget.removeItem(self._current_pos_item)

        # 添加新的位置标记
        if self.current_pos:
            self._current_pos_item = pg.PlotDataItem(
                [self.current_pos['x']], [self.current_pos['y']],
                pen=None, symbol='o', symbolSize=15, symbolBrush='g'
            )
            self.plot_widget.addItem(self._current_pos_item)

    def delayed_update_position(self):
        """延迟更新位置"""
        if self.pending_update and self.current_pos:
            self.last_update_time = time.time() * 1000
            self.update_current_position_marker()
        self.pending_update = False

    def load_map_from_c_file(self, file_path):
        """从.c文件加载地图数据"""
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                content = file.read()

            # 改进的节点解析正则表达式
            node_pattern = r'\{(\d+),\s*([\d.]+)f?,\s*([\d.]+)f?,\s*(\d+),\s*([\d.]+)f?,\s*([\d.]+)f?,\s*(\d+),\s*\{([^}]+)\}\}'
            node_matches = re.findall(node_pattern, content)

            self.nodes = []
            for match in node_matches:
                node_id = int(match[0])
                x = float(match[1])
                y = float(match[2])
                is_dangerous = int(match[3])
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
                    'is_dangerous': is_dangerous,
                    'base_cost': base_cost,
                    'cost': cost,
                    'neighbor_count': neighbor_count,
                    'neighbors': neighbors
                })

            # 改进的边解析正则表达式
            edge_pattern = r'\{(\d+),\s*(\d+),\s*([\d.]+)f?,\s*([\d.]+)f?\}'
            edge_matches = re.findall(edge_pattern, content)

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

            # 绘制地图
            self.draw_map()
            return True

        except Exception as e:
            print(f"解析地图文件错误: {e}")
            return False

    def draw_map(self):
        """绘制地图（不包括当前位置）"""
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

        # 绘制节点
        x_coords = [node['x'] for node in self.nodes]
        y_coords = [node['y'] for node in self.nodes]
        colors = ['r' if node['is_dangerous'] else 'b' for node in self.nodes]

        scatter = pg.ScatterPlotItem(x=x_coords, y=y_coords, size=10,
                                     brush=colors, pen='w')
        self.plot_widget.addItem(scatter)

        # 添加节点标签
        for node in self.nodes:
            text = pg.TextItem(str(node['node_id']), color='black', anchor=(0.5, 0.5))
            text.setPos(node['x'], node['y'])
            self.plot_widget.addItem(text)

        # 重新添加当前位置标记（如果存在）
        if self.current_pos:
            self.update_current_position_marker()

        # 重新添加敌人位置标记（如果存在）
        if self.enemy_pos:
            self.update_enemy_position_marker()

        # 重新添加危险节点标记（如果存在）
        if self.danger_nodes:
            self.update_danger_nodes_marker()

# PID可视化组件
class PIDWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.motor_data = {i: [] for i in range(4)}  # 4个电机的数据
        self.time_data = []
        self.current_time = 0
        self.max_points = 100  # 显示的最大数据点数
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
            plot.setXRange(0, self.max_points)  # 设置X轴范围

            # 添加曲线
            curve = plot.plot(pen=pg.mkPen('b', width=2))  # 使用固定颜色
            self.motor_data[i] = {'curve': curve, 'data': []}

            self.plot_widgets.append(plot)
            print(f"创建电机{i}的图表")  # 调试信息

        # 将4个图表排列在网格中
        grid_layout = QHBoxLayout()
        for i, plot in enumerate(self.plot_widgets):
            grid_layout.addWidget(plot)

        layout.addLayout(grid_layout)
        self.setLayout(layout)

    # 修改 PIDWidget 类的 update_pid_data 方法
    def update_pid_data(self, motor_id, pid_value):
        """更新PID数据"""
        if motor_id < 0 or motor_id > 3:
            print(f"无效的电机ID: {motor_id}")
            return

        # 验证PID值
        try:
            pid_value = float(pid_value)
        except (ValueError, TypeError):
            print(f"无效的PID值: {pid_value}")
            return

        # 更新时间
        self.current_time += 1
        self.time_data.append(self.current_time)

        # 更新数据
        self.motor_data[motor_id]['data'].append(pid_value)

        # 为其他电机填充默认值以保持同步
        for i in range(4):
            if i != motor_id:
                self.motor_data[i]['data'].append(0)  # 或使用上一个值

        # 偏航角数据也需要同步增长
        self.yaw_data.append(0)  # 为保持时间轴一致，添加默认偏航角值

        # 限制数据点数
        while len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            # 确保所有电机数据都移除第一个元素，保持长度一致
            for i in range(4):
                if self.motor_data[i]['data']:
                    self.motor_data[i]['data'].pop(0)
            # 同时移除偏航角数据
            if self.yaw_data:
                self.yaw_data.pop(0)

        # 更新所有电机图表 - 确保使用相同长度的数据
        for i in range(4):
            min_length = min(len(self.time_data), len(self.motor_data[i]['data']))
            if min_length > 0:
                x_data = self.time_data[-min_length:]
                y_data = self.motor_data[i]['data'][-min_length:]
                try:
                    self.motor_data[i]['curve'].setData(x_data, y_data)
                except Exception as e:
                    print(f"更新图表失败: {e}")

        # 更新偏航角图表
        if len(self.yaw_data) > 0:
            min_length = min(len(self.time_data), len(self.yaw_data))
            if min_length > 0:
                x_data = self.time_data[-min_length:]
                y_data = self.yaw_data[-min_length:]
                self.yaw_curve.setData(x_data, y_data)


# 主窗口
class BluetoothDebugTool(QMainWindow):
    # 修改 BluetoothDebugTool 类的 __init__ 方法
    def __init__(self):
        super().__init__()
        self.serial_connection = None
        self.bluetooth_connection = None
        self.is_connected = False
        self.is_bluetooth_connected = False
        self.received_data = ""
        self.bluetooth_scan_worker = None
        self.bluetooth_connection_worker = None
        self.services = None
        self.current_encoding = 'utf-8'
        self.serial_buffer = ""  # 用于存储未完成的串口数据

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

        # 蓝牙连接控制
        connection_group = QGroupBox("连接控制")
        connection_layout = QVBoxLayout(connection_group)

        # 串口连接部分
        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(refresh_btn)
        connection_layout.addLayout(serial_layout)

        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo)
        connection_layout.addLayout(baud_layout)

        # 解码方式选择
        decode_layout = QHBoxLayout()
        decode_layout.addWidget(QLabel("解码方式:"))
        self.decode_combo = QComboBox()
        self.decode_combo.addItems(["UTF-8", "GBK", "ASCII"])
        self.decode_combo.setCurrentText("UTF-8")
        self.decode_combo.currentTextChanged.connect(self.update_encoding)
        decode_layout.addWidget(self.decode_combo)
        connection_layout.addLayout(decode_layout)

        serial_btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("串口连接")
        self.connect_btn.clicked.connect(self.toggle_serial_connection)
        serial_btn_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)
        serial_btn_layout.addWidget(self.disconnect_btn)
        connection_layout.addLayout(serial_btn_layout)

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

        # PID可视化标签页
        self.pid_widget = PIDWidget()
        self.tab_widget.addTab(self.pid_widget, "PID可视化")

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

        # 创建定时器用于读取串口数据
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_data)

    def update_encoding(self, text):
        """更新解码方式"""
        encoding_map = {
            "UTF-8": "utf-8",
            "GBK": "gbk",
            "ASCII": "ascii"
        }
        self.current_encoding = encoding_map.get(text, "utf-8")

    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

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
        """接收蓝牙数据"""
        self.receive_text.append(f"[蓝牙] 接收: {data}")
        self.process_received_data(data)

    def on_bluetooth_disconnected(self):
        """蓝牙断开连接"""
        self.is_bluetooth_connected = False
        self.bluetooth_connect_btn.setText("蓝牙连接")
        self.bluetooth_connect_btn.setEnabled(True)
        self.receive_text.append("蓝牙连接已断开")

    def toggle_serial_connection(self):
        """切换串口连接状态"""
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        """连接串口设备"""
        try:
            port = self.port_combo.currentText()
            baudrate = int(self.baud_combo.currentText())

            if not port:
                QMessageBox.warning(self, "警告", "请选择串口")
                return

            self.serial_connection = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True

            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)

            # 启动定时器读取数据
            self.timer.start(10)  # 每10ms读取一次

            self.receive_text.append(f"已连接到 {port}，波特率 {baudrate}")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"连接失败: {str(e)}")

    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

        self.is_connected = False
        self.timer.stop()
        self.serial_buffer = ""  # 清空缓冲区

        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

        self.receive_text.append("已断开串口连接")

    def send_data(self):
        """发送数据"""
        if not self.is_connected and not self.is_bluetooth_connected:
            QMessageBox.warning(self, "警告", "未连接设备")
            return

        try:
            data = self.send_text.toPlainText()
            # 确保发送的数据以\n结尾
            if not data.endswith('\n'):
                data += '\n'
            data_bytes = data.encode('utf-8')
            if self.is_connected and self.serial_connection:
                self.serial_connection.write(data_bytes)
                self.receive_text.append(f"[串口] 发送: {data.strip()}")
            elif self.is_bluetooth_connected and self.bluetooth_connection_worker:
                self.bluetooth_connection_worker.send_data(data)
                self.receive_text.append(f"[蓝牙] 发送: {data.strip()}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送失败: {str(e)}")

    def read_serial_data(self):
        """读取串口数据"""
        if not self.is_connected or not self.serial_connection:
            return

        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.read(self.serial_connection.in_waiting).decode(self.current_encoding, errors='ignore')
                self.process_serial_data(data)
        except Exception as e:
            print(f"读取数据错误: {e}")

    def process_serial_data(self, data):
        """处理串口数据，以\n为一段数据的结束"""
        self.serial_buffer += data
        while '\n' in self.serial_buffer:
            message, self.serial_buffer = self.serial_buffer.split('\n', 1)
            if message:
                self.receive_text.append(f"[串口] 接收: {message.strip()}")
                self.process_received_data(message.strip())

    # 修改 process_received_data 方法，添加对敌人位置数据的处理
    def process_received_data(self, data):
        """处理接收到的数据"""
        try:
            # 处理多行数据
            lines = data.split('\n')
            for line in lines:
                line = line.strip()
                if not line:
                    continue

                if line.startswith("PID:"):
                    # 解析PID数据 (格式: PID:电机ID,实际值,目标值)
                    parts = line[4:]  # 去掉 "PID:" 前缀
                    pid_parts = parts.split(",")
                    if len(pid_parts) >= 2:
                        motor_id = int(pid_parts[0])
                        pid_value = float(pid_parts[1])
                        target_pid_value = 0  # 默认目标值为0
                        if len(pid_parts) >= 3:
                            target_pid_value = float(pid_parts[2])
                        self.pid_widget.update_pid_data(motor_id, pid_value, target_pid_value)
                        print(f"PID数据更新: 电机{motor_id}, 值{pid_value}, 目标值{target_pid_value}")

                elif line.startswith("POS:"):
                    # 解析位置数据
                    parts = line[4:]  # 去掉 "POS:" 前缀
                    pos_parts = parts.split(",")
                    if len(pos_parts) >= 2:
                        x = float(pos_parts[0])
                        y = float(pos_parts[1])
                        # 使用防抖更新位置
                        self.map_widget.update_position(x, y)

                elif line.startswith("ENEMY:"):
                    # 解析敌人位置数据
                    parts = line[7:]  # 去掉 "ENEMY:" 前缀
                    enemy_parts = parts.split(",")
                    if len(enemy_parts) >= 2:
                        x = float(enemy_parts[0])
                        y = float(enemy_parts[1])
                        # 使用防抖更新敌人位置
                        self.map_widget.update_enemy_position(x, y)

                elif line.startswith("YAW:"):
                    # 解析偏航角数据 (格式: YAW:实际值,目标值)
                    parts = line[4:]  # 去掉 "YAW:" 前缀
                    yaw_parts = parts.split(",")
                    yaw_value = yaw_parts[0]
                    target_yaw_value = 0  # 默认目标值为0
                    if len(yaw_parts) >= 2:
                        target_yaw_value = yaw_parts[1]
                    try:
                        yaw_value = float(yaw_value)
                        target_yaw_value = float(target_yaw_value)
                        self.pid_widget.update_yaw_data(yaw_value, target_yaw_value)
                        print(f"偏航角数据更新: {yaw_value}°, 目标值{target_yaw_value}°")
                    except ValueError:
                        print(f"无效的偏航角数据: {yaw_value}")

                elif line.startswith("track_data:"):
                    try:
                        # 提取track_data后面的数值
                        value_str = line[11:]  # 去掉 "track_data:" 前缀
                        value = int(value_str)
                        # 转换为16位二进制字符串，然后只取后8位
                        binary_str = format(value, '016b')
                        # 只显示后8位
                        last_8_bits = binary_str[8:]  # 取后8位
                        # 格式化显示，每4位用空格分隔，提高可读性
                        formatted_binary = ' '.join([last_8_bits[i:i + 4] for i in range(0, len(last_8_bits), 4)])
                        self.receive_text.append(f"[解析] track_data: {value} => 后8位二进制: {formatted_binary}")
                        print(f"track_data解析: {value} => 后8位 {last_8_bits}")
                    except ValueError:
                        self.receive_text.append(f"[错误] 无法解析track_data: {line}")
                        print(f"无效的track_data格式: {line}")

        except Exception as e:
            print(f"解析数据错误: {e}")

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
        """测试位置更新"""
        # 随机生成一个测试位置
        import random
        x = random.uniform(0, 1000)
        y = random.uniform(0, 1000)
        self.map_widget.update_position(x, y)
        self.receive_text.append(f"测试位置: ({x:.2f}, {y:.2f})")

        # 同时生成一个敌人的测试位置
        enemy_x = random.uniform(0, 1000)
        enemy_y = random.uniform(0, 1000)
        self.map_widget.update_enemy_position(enemy_x, enemy_y)
        self.receive_text.append(f"敌人测试位置: ({enemy_x:.2f}, {enemy_y:.2f})")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BluetoothDebugTool()
    window.show()
    sys.exit(app.exec_())
