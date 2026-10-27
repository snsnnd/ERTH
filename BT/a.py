import sys
import time
import asyncio
import struct
import traceback
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from bleak import BleakClient, BleakScanner
import qasync


# ------------------------------
# 协议类（增强数据校验）
# ------------------------------
class BLEProtocol:
    SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
    WRITE_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
    NOTIFY_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
    CMD_PID_READ = 0x02
    PID_DATA_LEN = 14  # 严格定义PID数据长度

    @staticmethod
    def is_ack(data):
        return isinstance(data, bytes) and len(data) > 0 and data[0] == 0xAA

    @staticmethod
    def create_pid_command(motor_id, kp, ki, kd):
        return struct.pack('BBfff', 0x01, motor_id, kp, ki, kd)

    @staticmethod
    def create_pid_read_command(motor_id):
        return struct.pack('BB', 0x02, motor_id)

    @staticmethod
    def create_data_request(req_type):
        return struct.pack('B', req_type)

    @staticmethod
    def parse_data_response(data):
        if not isinstance(data, bytes) or len(data) < 1:
            return None
        try:
            if data[0] == 0x01 and len(data) >= 10:  # 传感器数据（增加长度校验）
                return {'type': 'sensor', 'sensor_data': {
                    'gyro_x': struct.unpack('f', data[1:5])[0],
                    'gyro_y': struct.unpack('f', data[5:9])[0],
                    'gyro_z': struct.unpack('f', data[9:13])[0]
                }}
            elif data[0] == 0x02 and len(data) == BLEProtocol.PID_DATA_LEN:  # PID数据（严格长度）
                return {'type': 'pid', 'motor_id': data[1],
                        'pid_params': {
                            'kp': struct.unpack('f', data[2:6])[0],
                            'ki': struct.unpack('f', data[6:10])[0],
                            'kd': struct.unpack('f', data[10:14])[0]
                        }}
        except Exception as e:
            print(f"数据解析错误: {str(e)}")
        return None


# ------------------------------
# 1. 数据监控窗口
# ------------------------------
class DataMonitorWindow(QMainWindow):
    closed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("数据接收监控")
        self.setGeometry(400, 200, 600, 400)
        self.data_display = None
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        self.data_display = QTextEdit()
        self.data_display.setReadOnly(True)
        self.data_display.setStyleSheet("""
            background-color: #0d0d1a; color: #e6e6e6;
            border: 1px solid #2a82da; border-radius: 4px;
            padding: 10px; font-family: 'Consolas';
        """)
        self.data_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        close_btn = QPushButton("关闭窗口")
        close_btn.setStyleSheet("background-color: #e74c3c; color: white; border-radius: 4px; padding: 6px;")
        close_btn.clicked.connect(self.close)

        layout.addWidget(QLabel("接收数据详情（实时更新）：", styleSheet="font-weight: bold;"))
        layout.addWidget(self.data_display)
        layout.addWidget(close_btn, alignment=Qt.AlignRight)

    def update_data(self, data_str):
        if self.data_display and not self.data_display.isHidden():
            self.data_display.append(data_str)
            self.data_display.moveCursor(QTextCursor.End)

    def closeEvent(self, event):
        self.closed.emit()
        if self.data_display:
            self.data_display.clear()
            self.data_display.deleteLater()
        event.accept()


# ------------------------------
# 2. PID曲线窗口（兼容旧版本QtChart）
# ------------------------------
class PIDChartWindow(QMainWindow):
    closed = pyqtSignal()
    _ZOOM_STEP = 1.2
    _MIN_ZOOM = 0.1
    _MAX_ZOOM = 10

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID参数曲线可视化（实时更新）")
        self.setGeometry(400, 700, 800, 500)
        self.pid_history = {'kp': [], 'ki': [], 'kd': [], 'timestamps': []}
        self.max_data_points = 50
        self.current_zoom_x = 1.0
        self.current_zoom_y = 1.0
        self.chart = None
        self.chart_view = None
        self.kp_series = None
        self.ki_series = None
        self.kd_series = None
        self.axis_x = None
        self.axis_y = None
        self.init_ui()

    def init_ui(self):
        try:
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            layout = QVBoxLayout(central_widget)
            layout.setContentsMargins(10, 10, 10, 10)
            layout.setSpacing(10)

            # 初始化图表
            self.chart = QChart()
            self.chart.setTitle("PID参数实时变化曲线（Kp/Ki/Kd）")
            self.chart.setTheme(QChart.ChartThemeDark)

            # 初始化图表视图（关键：旧版本通过QChartView实现缩放）
            self.chart_view = QChartView(self.chart)
            self.chart_view.setRenderHint(QPainter.Antialiasing)
            # 启用交互功能（缩放、拖拽）
            self.chart_view.setInteractive(True)
            # 配置缩放模式（通过QChartView的rubberBand，兼容旧版本）
            self.chart_view.setRubberBand(QChartView.HorizontalRubberBand | QChartView.VerticalRubberBand)

            # 初始化曲线系列
            self.kp_series = QLineSeries()
            self.kp_series.setName("Kp (比例系数)")
            self.kp_series.setColor(QColor(255, 99, 71))
            self.ki_series = QLineSeries()
            self.ki_series.setName("Ki (积分系数)")
            self.ki_series.setColor(QColor(60, 179, 113))
            self.kd_series = QLineSeries()
            self.kd_series.setName("Kd (微分系数)")
            self.kd_series.setColor(QColor(70, 130, 180))

            # 添加系列到图表
            self.chart.addSeries(self.kp_series)
            self.chart.addSeries(self.ki_series)
            self.chart.addSeries(self.kd_series)

            # 初始化坐标轴
            self.axis_x = QValueAxis()
            self.axis_x.setTitleText("采样次数")
            self.axis_x.setLabelFormat("%d")
            self.axis_x.setTickCount(6)

            self.axis_y = QValueAxis()
            self.axis_y.setTitleText("参数值")
            self.axis_y.setLabelFormat("%.3f")
            self.axis_y.setTickCount(6)

            # 添加坐标轴到图表
            self.chart.addAxis(self.axis_x, Qt.AlignBottom)
            self.chart.addAxis(self.axis_y, Qt.AlignLeft)
            # 关联系列与坐标轴
            self.kp_series.attachAxis(self.axis_x)
            self.kp_series.attachAxis(self.axis_y)
            self.ki_series.attachAxis(self.axis_x)
            self.ki_series.attachAxis(self.axis_y)
            self.kd_series.attachAxis(self.axis_x)
            self.kd_series.attachAxis(self.axis_y)

            # 设置视图大小策略
            self.chart_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            # 关闭按钮
            close_btn = QPushButton("关闭窗口")
            close_btn.setStyleSheet("background-color: #e74c3c; color: white; border-radius: 4px; padding: 6px;")
            close_btn.clicked.connect(self.close)

            # 组装布局
            layout.addWidget(self.chart_view)
            layout.addWidget(close_btn, alignment=Qt.AlignRight)
        except Exception as e:
            print(f"图表初始化失败: {str(e)}")
            traceback.print_exc()

    def update_pid_data(self, kp, ki, kd):
        if not all(isinstance(v, (int, float)) for v in [kp, ki, kd]):
            print("无效的PID数据类型")
            return
        try:
            timestamp = len(self.pid_history['timestamps'])
            self.pid_history['timestamps'].append(timestamp)
            self.pid_history['kp'].append(kp)
            self.pid_history['ki'].append(ki)
            self.pid_history['kd'].append(kd)

            if len(self.pid_history['timestamps']) > self.max_data_points:
                for key in self.pid_history:
                    self.pid_history[key].pop(0)

            self._refresh_chart(keep_zoom=True)
        except Exception as e:
            print(f"PID数据更新错误: {str(e)}")
            traceback.print_exc()

    def _refresh_chart(self, keep_zoom=False):
        if not all([self.chart, self.chart_view, self.kp_series, self.ki_series, self.kd_series, self.axis_x,
                    self.axis_y]):
            print("图表组件未初始化完成")
            return
        try:
            self.kp_series.clear()
            self.ki_series.clear()
            self.kd_series.clear()

            if not self.pid_history['timestamps']:
                return

            for i in range(len(self.pid_history['timestamps'])):
                t = self.pid_history['timestamps'][i]
                self.kp_series.append(t, self.pid_history['kp'][i])
                self.ki_series.append(t, self.pid_history['ki'][i])
                self.kd_series.append(t, self.pid_history['kd'][i])

            x_min = min(self.pid_history['timestamps'])
            x_max = max(self.pid_history['timestamps'])
            all_values = self.pid_history['kp'] + self.pid_history['ki'] + self.pid_history['kd']

            if not all_values:
                y_min, y_max = 0, 1
            else:
                y_min, y_max = min(all_values), max(all_values)

            y_margin = (y_max - y_min) * 0.1 if (y_max - y_min) != 0 else 0.1

            if keep_zoom:
                x_range = (x_max - x_min) / self.current_zoom_x
                y_range = (y_max - y_min + 2 * y_margin) / self.current_zoom_y
                self.axis_x.setRange(x_max - x_range, x_max)
                self.axis_y.setRange((y_min + y_max - y_range) / 2, (y_min + y_max + y_range) / 2)
            else:
                self.axis_x.setRange(x_min, x_max)
                self.axis_y.setRange(y_min - y_margin, y_max + y_margin)
        except Exception as e:
            print(f"图表刷新错误: {str(e)}")
            traceback.print_exc()

    def wheelEvent(self, event):
        try:
            delta = event.angleDelta().y()
            zoom_factor = self._ZOOM_STEP if delta > 0 else 1 / self._ZOOM_STEP

            if event.modifiers() & Qt.ShiftModifier:
                new_zoom_x = self.current_zoom_x * zoom_factor
                self.current_zoom_x = max(self._MIN_ZOOM, min(new_zoom_x, self._MAX_ZOOM))
            else:
                new_zoom_y = self.current_zoom_y * zoom_factor
                self.current_zoom_y = max(self._MIN_ZOOM, min(new_zoom_y, self._MAX_ZOOM))

            self._refresh_chart(keep_zoom=True)
            event.accept()
        except Exception as e:
            print(f"缩放错误: {str(e)}")
            event.ignore()

    def resizeEvent(self, event):
        try:
            super().resizeEvent(event)
            if self.chart_view:
                self.chart_view.resize(event.size().width() - 20, event.size().height() - 60)
            event.accept()
        except Exception as e:
            print(f"窗口 resize 错误: {str(e)}")
            event.ignore()

    def closeEvent(self, event):
        self.closed.emit()
        if self.kp_series:
            self.kp_series.clear()
        if self.ki_series:
            self.ki_series.clear()
        if self.kd_series:
            self.kd_series.clear()
        if self.chart:
            self.chart.removeAllSeries()
        if self.chart_view:
            self.chart_view.deleteLater()
        self.pid_history = {}
        event.accept()


# ------------------------------
# 3. 主窗口
# ------------------------------
class BLEMainWindow(QMainWindow):
    signal_update_data = pyqtSignal(str)
    signal_update_pid = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.client = None
        self.device_list = []
        self.is_connected = False
        self.current_service_uuid = ""
        self.current_char_uuid = ""
        self.notify_char_uuid = BLEProtocol.NOTIFY_CHAR_UUID
        self.data_window = None
        self.chart_window = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('BLE主控制窗口')
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("""
            QMainWindow {background-color: #1a1a2e;}
            QWidget {background-color: #1a1a2e; color: #e6e6e6; font-family: 'Segoe UI'; font-size: 12px;}
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2a82da, stop:1 #1e5f9b);
                border: 1px solid #1e5f9b; border-radius: 5px; color: white; padding: 5px 15px; min-width: 80px;
            }
            QPushButton:hover {background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #3a92ea, stop:1 #2e6fab);}
            QLineEdit, QTextEdit, QListWidget {
                background-color: #0d0d1a; border: 1px solid #2a82da; border-radius: 4px; padding: 5px; color: #e6e6e6;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)

        # 过滤条件
        filter_layout = QHBoxLayout()
        self.filter_label = QLabel('过滤名称:')
        self.filter_label.setStyleSheet("font-weight: bold; color: #2a82da;")
        self.filter_input = QLineEdit('TW')
        self.filter_input.setPlaceholderText('输入设备名称关键字')
        filter_layout.addWidget(self.filter_label)
        filter_layout.addWidget(self.filter_input)
        layout.addLayout(filter_layout)

        # 设备列表区域
        device_group = QGroupBox("蓝牙设备列表")
        device_group.setStyleSheet("""
            QGroupBox {border: 2px solid #2a82da; border-radius: 5px; margin-top: 10px; padding-top: 15px; color: #2a82da; font-weight: bold;}
        """)
        device_layout = QHBoxLayout(device_group)
        self.device_list_widget = QListWidget()
        self.device_list_widget.setStyleSheet("""
            QListWidget::item {padding: 8px; border-bottom: 1px solid #2a82da;}
            QListWidget::item:selected {background: #2a82da; color: white;}
        """)
        self.device_list_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 设备操作按钮
        btn_layout = QVBoxLayout()
        btn_layout.setSpacing(10)
        self.scan_btn = self.create_icon_button('扫描设备', '#2a82da')
        self.connect_btn = self.create_icon_button('连接', '#27ae60')
        self.disconnect_btn = self.create_icon_button('断开', '#c0392b')
        self.reflash_btn = self.create_icon_button('刷新状态', '#8e44ad')

        # 连接状态灯
        self.connection_led = QLabel()
        self.connection_led.setFixedSize(25, 25)
        self.update_connection_led(False)

        # UUID显示
        self.uuid_label = QLabel()
        self.uuid_label.setStyleSheet(
            "background: #0d0d1a; padding: 10px; border-radius: 5px; border: 1px solid #2a82da;")
        self.update_uuid_display()

        # 组装设备按钮区域
        btn_layout.addWidget(self.scan_btn)
        btn_layout.addWidget(self.connect_btn)
        btn_layout.addWidget(self.disconnect_btn)
        btn_layout.addWidget(self.reflash_btn)
        btn_layout.addWidget(self.connection_led)
        btn_layout.addStretch()
        btn_layout.addWidget(QLabel("服务信息："))
        btn_layout.addWidget(self.uuid_label)

        device_layout.addWidget(self.device_list_widget, 70)
        device_layout.addLayout(btn_layout, 30)
        layout.addWidget(device_group)

        # 子窗口控制按钮
        subwindow_btn_layout = QHBoxLayout()
        self.open_data_btn = self.create_icon_button('打开数据监控', '#f39c12')
        self.close_data_btn = self.create_icon_button('关闭数据监控', '#e74c3c')
        self.open_chart_btn = self.create_icon_button('打开PID曲线', '#9b59b6')
        self.close_chart_btn = self.create_icon_button('关闭PID曲线', '#e74c3c')
        self.close_data_btn.setEnabled(False)
        self.close_chart_btn.setEnabled(False)
        subwindow_btn_layout.addWidget(self.open_data_btn)
        subwindow_btn_layout.addWidget(self.close_data_btn)
        subwindow_btn_layout.addWidget(self.open_chart_btn)
        subwindow_btn_layout.addWidget(self.close_chart_btn)
        layout.addLayout(subwindow_btn_layout)

        # 数据发送区域
        send_group = QGroupBox("数据传输")
        send_group.setStyleSheet(device_group.styleSheet())
        send_layout = QVBoxLayout(send_group)
        text_layout = QHBoxLayout()
        self.text_input = QLineEdit()
        self.text_input.setPlaceholderText("输入文本数据...")
        self.send_text_btn = self.create_icon_button('发送文本', '#3498db')
        text_layout.addWidget(self.text_input)
        text_layout.addWidget(self.send_text_btn)
        hex_layout = QHBoxLayout()
        self.hex_input = QLineEdit()
        self.hex_input.setPlaceholderText("输入HEX数据（如 AA BB CC）...")
        self.send_hex_btn = self.create_icon_button('发送HEX', '#2980b9')
        hex_layout.addWidget(self.hex_input)
        hex_layout.addWidget(self.send_hex_btn)
        send_layout.addLayout(text_layout)
        send_layout.addLayout(hex_layout)
        layout.addWidget(send_group)

        # PID控制区域
        pid_group = QGroupBox("PID控制器")
        pid_group.setStyleSheet(device_group.styleSheet())
        pid_layout = QVBoxLayout(pid_group)
        pid_input_layout = QHBoxLayout()
        self.motor_id_input = QSpinBox()
        self.motor_id_input.setRange(0, 10)
        self.kp_input = QDoubleSpinBox()
        self.kp_input.setRange(0, 100)
        self.kp_input.setSingleStep(0.1)
        self.ki_input = QDoubleSpinBox()
        self.ki_input.setRange(0, 100)
        self.ki_input.setSingleStep(0.01)
        self.kd_input = QDoubleSpinBox()
        self.kd_input.setRange(0, 100)
        self.kd_input.setSingleStep(0.01)
        pid_input_layout.addWidget(QLabel("电机ID:"))
        pid_input_layout.addWidget(self.motor_id_input)
        pid_input_layout.addWidget(QLabel("Kp:"))
        pid_input_layout.addWidget(self.kp_input)
        pid_input_layout.addWidget(QLabel("Ki:"))
        pid_input_layout.addWidget(self.ki_input)
        pid_input_layout.addWidget(QLabel("Kd:"))
        pid_input_layout.addWidget(self.kd_input)
        pid_btn_layout = QHBoxLayout()
        self.update_pid_btn = self.create_icon_button('更新PID', '#e67e22')
        self.read_pid_btn = self.create_icon_button('读取PID', '#16a085')
        pid_btn_layout.addWidget(self.update_pid_btn)
        pid_btn_layout.addWidget(self.read_pid_btn)
        pid_layout.addLayout(pid_input_layout)
        pid_layout.addLayout(pid_btn_layout)
        layout.addWidget(pid_group)

        # 传感器请求区域
        sensor_group = QGroupBox("传感器数据请求")
        sensor_group.setStyleSheet(device_group.styleSheet())
        sensor_layout = QHBoxLayout(sensor_group)
        self.request_sensor_btn = self.create_icon_button('请求陀螺仪数据', '#3498db')
        self.request_status_btn = self.create_icon_button('请求设备状态', '#34495e')
        sensor_layout.addWidget(self.request_sensor_btn)
        sensor_layout.addWidget(self.request_status_btn)
        layout.addWidget(sensor_group)

        # 日志区域
        log_group = QGroupBox("系统日志")
        log_group.setStyleSheet(device_group.styleSheet())
        log_layout = QVBoxLayout(log_group)
        self.log = QTextEdit()
        self.log.setStyleSheet("font-family: 'Consolas'; font-size: 11px;")
        self.log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)

        # 绑定信号槽
        self.scan_btn.clicked.connect(self.scan_devices)
        self.connect_btn.clicked.connect(self.connect_device)
        self.disconnect_btn.clicked.connect(self.disconnect_device)
        self.reflash_btn.clicked.connect(self.refresh_connection)
        self.send_text_btn.clicked.connect(self.send_text_data)
        self.send_hex_btn.clicked.connect(self.send_hex_data)
        self.update_pid_btn.clicked.connect(self.update_pid_params)
        self.read_pid_btn.clicked.connect(self.read_pid_params)
        self.request_sensor_btn.clicked.connect(self.request_sensor_data)
        self.request_status_btn.clicked.connect(self.request_status_data)
        self.open_data_btn.clicked.connect(self.open_data_window)
        self.close_data_btn.clicked.connect(self.close_data_window)
        self.open_chart_btn.clicked.connect(self.open_chart_window)
        self.close_chart_btn.clicked.connect(self.close_chart_window)
        self.signal_update_data.connect(self._on_data_updated, Qt.QueuedConnection)
        self.signal_update_pid.connect(self._on_pid_updated, Qt.QueuedConnection)

    def create_icon_button(self, text, color, size=None):
        btn = QPushButton(text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background: qradialgradient(cx:0.5, cy:0.5, radius: 0.5, fx:0.5, fy:0.5, stop:0 {color}80, stop:1 {color});
                border: 1px solid {color}60; border-radius: 8px; color: white; padding: 8px 15px; font-weight: bold;
            }}
            QPushButton:hover {{background: qradialgradient(cx:0.5, cy:0.5, radius: 0.5, fx:0.5, fy:0.5, stop:0 {color}90, stop:1 {color}80);}}
            QPushButton:pressed {{background: qradialgradient(cx:0.5, cy:0.5, radius: 0.5, fx:0.5, fy:0.5, stop:0 {color}, stop:1 {color}80);}}
        """)
        if size:
            btn.setFixedSize(*size)
        return btn

    def open_data_window(self):
        if not self.data_window or not self.data_window.isVisible():
            self.data_window = DataMonitorWindow()
            self.data_window.closed.connect(self.close_data_window)
            self.data_window.show()
            self.open_data_btn.setEnabled(False)
            self.close_data_btn.setEnabled(True)
            self.log_message("数据监控窗口已打开")

    def close_data_window(self):
        if self.data_window and self.data_window.isVisible():
            self.data_window.close()
        self.data_window = None
        self.open_data_btn.setEnabled(True)
        self.close_data_btn.setEnabled(False)
        self.log_message("数据监控窗口已关闭")

    def open_chart_window(self):
        if not self.chart_window or not self.chart_window.isVisible():
            self.chart_window = PIDChartWindow()
            self.chart_window.closed.connect(self.close_chart_window)
            self.chart_window.show()
            self.open_chart_btn.setEnabled(False)
            self.close_chart_btn.setEnabled(True)
            self.log_message("PID曲线窗口已打开")

    def close_chart_window(self):
        if self.chart_window and self.chart_window.isVisible():
            self.chart_window.close()
        self.chart_window = None
        self.open_chart_btn.setEnabled(True)
        self.close_chart_btn.setEnabled(False)
        self.log_message("PID曲线窗口已关闭")

    def _on_data_updated(self, data_str):
        if self.data_window and self.data_window.isVisible():
            self.data_window.update_data(data_str)

    def _on_pid_updated(self, kp, ki, kd):
        if self.chart_window and self.chart_window.isVisible():
            self.chart_window.update_pid_data(kp, ki, kd)

    def closeEvent(self, event):
        self.close_data_window()
        self.close_chart_window()
        if self.is_connected:
            asyncio.ensure_future(self.disconnect_device())
        event.accept()

    def update_connection_led(self, connected):
        color = QColor(0, 255, 0) if connected else QColor(255, 0, 0)
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setBrush(color)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 20, 20)
        painter.end()
        self.connection_led.setPixmap(pixmap)

    def update_uuid_display(self):
        if self.is_connected:
            self.uuid_label.setText(f"""
                <b>服务UUID:</b><br>{BLEProtocol.SERVICE_UUID}<br><br>
                <b>写特征:</b><br>{BLEProtocol.WRITE_CHAR_UUID}<br><br>
                <b>通知特征:</b><br>{BLEProtocol.NOTIFY_CHAR_UUID}
            """)
        else:
            self.uuid_label.setText("未连接设备")

    def log_message(self, message):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.log.append(f'[{timestamp}] {message}')
        self.log.moveCursor(QTextCursor.End)

    # BLE相关异步方法
    @qasync.asyncSlot()
    async def scan_devices(self):
        try:
            self.device_list_widget.clear()
            self.device_list = []
            self.log_message('开始扫描蓝牙设备...')
            devices = await BleakScanner.discover()
            if not devices:
                self.log_message('未发现任何蓝牙设备')
                return
            filter_name = self.filter_input.text().strip()
            for d in devices:
                device_name = getattr(d, 'name', '未知设备') or '未知设备'
                if filter_name and filter_name.upper() not in device_name.upper():
                    continue
                self.device_list.append(d)
                self.device_list_widget.addItem(f'{device_name} ({d.address})')
            self.log_message(f'共发现 {len(self.device_list)} 个设备')
        except Exception as e:
            self.log_message(f'扫描失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def connect_device(self):
        try:
            selected = self.device_list_widget.currentRow()
            if selected < 0:
                self.log_message('请先选择一个设备')
                return
            device = self.device_list[selected]
            self.log_message(f'正在连接设备: {device.name} ({device.address})')
            self.client = BleakClient(device.address)
            await self.client.connect()
            self.is_connected = self.client.is_connected
            self.update_connection_led(True)
            services = self.client.services
            target_service_uuid = BLEProtocol.SERVICE_UUID
            target_write_uuid = BLEProtocol.WRITE_CHAR_UUID
            target_notify_uuid = BLEProtocol.NOTIFY_CHAR_UUID
            self.current_service_uuid = ""
            self.current_char_uuid = ""
            self.notify_char_uuid = None
            for service in services:
                if service.uuid == target_service_uuid:
                    self.current_service_uuid = service.uuid
                    for char in service.characteristics:
                        if char.uuid == target_write_uuid and 'write' in char.properties:
                            self.current_char_uuid = char.uuid
                        elif char.uuid == target_notify_uuid and 'notify' in char.properties:
                            self.notify_char_uuid = char.uuid
            self.update_uuid_display()
            if self.notify_char_uuid:
                await self.client.start_notify(self.notify_char_uuid, self.notification_handler)
                self.log_message('已启用通知服务，等待接收数据...')
            self.text_input.setEnabled(True)
            self.hex_input.setEnabled(True)
            self.send_text_btn.setEnabled(True)
            self.send_hex_btn.setEnabled(True)
            self.log_message(f'成功连接到 {device.name}')
        except Exception as e:
            self.log_message(f'连接失败: {str(e)}')
            self.is_connected = False
            self.update_connection_led(False)
            traceback.print_exc()

    @qasync.asyncSlot()
    async def disconnect_device(self):
        if self.client and self.is_connected:
            try:
                if self.notify_char_uuid:
                    await self.client.stop_notify(self.notify_char_uuid)
                await self.client.disconnect()
                self.log_message('设备已断开连接')
            except Exception as e:
                self.log_message(f'断开失败: {str(e)}')
                traceback.print_exc()
            finally:
                self.is_connected = False
                self.update_connection_led(False)
                self.text_input.setEnabled(False)
                self.hex_input.setEnabled(False)
                self.send_text_btn.setEnabled(False)
                self.send_hex_btn.setEnabled(False)
                self.client = None

    @qasync.asyncSlot()
    async def refresh_connection(self):
        try:
            if self.client:
                self.is_connected = self.client.is_connected
                self.update_connection_led(self.is_connected)
                self.log_message(f'连接状态刷新: {"已连接" if self.is_connected else "已断开"}')
        except Exception as e:
            self.log_message(f'刷新失败: {str(e)}')
            self.is_connected = False
            self.update_connection_led(False)
            traceback.print_exc()

    @qasync.asyncSlot()
    async def send_text_data(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            text = self.text_input.text().strip()
            if not text:
                return
            data = text.encode('utf-8')
            await self.client.write_gatt_char(self.current_char_uuid, data, response=True)
            self.log_message(f'已发送文本: {text}')
            self.text_input.clear()
        except Exception as e:
            self.log_message(f'文本发送失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def send_hex_data(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            hex_str = self.hex_input.text().strip().replace(' ', '')
            if not hex_str:
                return
            data = bytes.fromhex(hex_str)
            await self.client.write_gatt_char(self.current_char_uuid, data, response=True)
            self.log_message(f'已发送HEX: {hex_str}')
            self.hex_input.clear()
        except Exception as e:
            self.log_message(f'HEX发送失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def update_pid_params(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            motor_id = self.motor_id_input.value()
            kp = self.kp_input.value()
            ki = self.ki_input.value()
            kd = self.kd_input.value()
            pid_cmd = BLEProtocol.create_pid_command(motor_id, kp, ki, kd)
            await self.client.write_gatt_char(self.current_char_uuid, pid_cmd, response=True)
            self.log_message(f'已发送PID更新: 电机{motor_id}, Kp={kp}, Ki={ki}, Kd={kd}')
        except Exception as e:
            self.log_message(f'PID更新失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def read_pid_params(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            motor_id = self.motor_id_input.value()
            cmd = BLEProtocol.create_pid_read_command(motor_id)
            await self.client.write_gatt_char(self.current_char_uuid, cmd, response=True)
            self.log_message(f'已发送PID读取请求: 电机{motor_id}')
        except Exception as e:
            self.log_message(f'读取请求失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def request_sensor_data(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            cmd = BLEProtocol.create_data_request(0x01)
            await self.client.write_gatt_char(self.current_char_uuid, cmd, response=True)
            self.log_message('已发送陀螺仪数据请求')
        except Exception as e:
            self.log_message(f'传感器请求失败: {str(e)}')
            traceback.print_exc()

    @qasync.asyncSlot()
    async def request_status_data(self):
        try:
            if not self.is_connected or not self.client or not self.client.is_connected:
                self.log_message('请先连接设备')
                return
            if not self.current_char_uuid:
                self.log_message('未找到有效的写入特征UUID')
                return
            cmd = BLEProtocol.create_data_request(0x03)
            await self.client.write_gatt_char(self.current_char_uuid, cmd, response=True)
            self.log_message('已发送设备状态请求')
        except Exception as e:
            self.log_message(f'状态请求失败: {str(e)}')
            traceback.print_exc()

    def notification_handler(self, sender, data):
        if not isinstance(data, bytes):
            print("无效的BLE数据格式")
            return
        try:
            if BLEProtocol.is_ack(data):
                self.log_message('收到设备确认信号(ACK)')
                self.signal_update_data.emit(f"[{time.strftime('%H:%M:%S')}] 收到确认信号(ACK)\n")
                return
            try:
                text_data = data.decode('utf-8')
                self.log_message(f'收到文本数据: {text_data}')
                self.signal_update_data.emit(f"[{time.strftime('%H:%M:%S')}] 文本数据: {text_data}\n")
            except UnicodeDecodeError:
                hex_str = data.hex()
                self.log_message(f'收到二进制数据: {hex_str}')
                self.signal_update_data.emit(f"[{time.strftime('%H:%M:%S')}] 二进制数据: {hex_str}\n")
            parsed_data = BLEProtocol.parse_data_response(data)
            if parsed_data and 'pid_params' in parsed_data:
                pid = parsed_data['pid_params']
                self.signal_update_pid.emit(pid['kp'], pid['ki'], pid['kd'])
            elif len(data) == BLEProtocol.PID_DATA_LEN and data[0] == BLEProtocol.CMD_PID_READ:
                motor_id = data[1]
                kp = struct.unpack('f', data[2:6])[0]
                ki = struct.unpack('f', data[6:10])[0]
                kd = struct.unpack('f', data[10:14])[0]
                self.log_message(f"收到电机 {motor_id} 的PID参数: Kp={kp:.2f}, Ki={ki:.4f}, Kd={kd:.4f}")
                self.signal_update_data.emit(
                    f"[{time.strftime('%H:%M:%S')}] 电机 {motor_id} PID参数: Kp={kp:.2f}, Ki={ki:.4f}, Kd={kd:.4f}\n")
                self.signal_update_pid.emit(kp, ki, kd)
        except Exception as e:
            print(f"BLE数据处理错误: {str(e)}")
            traceback.print_exc()


# ------------------------------
# 程序入口
# ------------------------------
if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 9))
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    try:
        main_window = BLEMainWindow()
        main_window.show()
        with loop:
            loop.run_forever()
    except Exception as e:
        print(f"主程序崩溃: {str(e)}")
        traceback.print_exc()
    finally:
        sys.exit(0)