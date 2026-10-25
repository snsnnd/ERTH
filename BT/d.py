import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QLineEdit, QPushButton, QTextEdit, QCheckBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer

class SerialThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        while self.running:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.data_received.emit(data)

    def stop(self):
        self.running = False

class BluetoothDebugger(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZS-040 蓝牙调试工具")
        self.resize(600, 400)

        self.ser = None
        self.thread = None

        # 布局
        layout = QVBoxLayout()

        # 串口选择
        port_layout = QHBoxLayout()
        port_label = QLabel("串口:")
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)

        # 波特率
        baud_label = QLabel("波特率:")
        self.baud_edit = QLineEdit("9600")
        port_layout.addWidget(baud_label)
        port_layout.addWidget(self.baud_edit)

        # 刷新按钮
        refresh_btn = QPushButton("刷新串口")
        refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(refresh_btn)

        layout.addLayout(port_layout)

        # 连接/断开按钮
        conn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        layout.addLayout(conn_layout)

        # 发送区
        send_layout = QHBoxLayout()
        send_label = QLabel("发送数据:")
        self.send_edit = QLineEdit()
        self.hex_check = QCheckBox("HEX 发送")
        self.send_btn = QPushButton("发送")
        self.send_btn.clicked.connect(self.send_data)
        send_layout.addWidget(send_label)
        send_layout.addWidget(self.send_edit)
        send_layout.addWidget(self.hex_check)
        send_layout.addWidget(self.send_btn)
        layout.addLayout(send_layout)

        # 接收区
        recv_label = QLabel("接收数据:")
        layout.addWidget(recv_label)
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        layout.addWidget(self.recv_text)

        # 清空按钮
        clear_btn = QPushButton("清空接收区")
        clear_btn.clicked.connect(self.clear_recv)
        layout.addWidget(clear_btn)

        self.setLayout(layout)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def connect_serial(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_edit.text())
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.thread = SerialThread(self.ser)
            self.thread.data_received.connect(self.update_recv)
            self.thread.start()
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.recv_text.append("已连接到 " + port)
        except Exception as e:
            self.recv_text.append("连接失败: " + str(e))

    def disconnect_serial(self):
        if self.thread:
            self.thread.stop()
            self.thread.wait()
        if self.ser:
            self.ser.close()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.recv_text.append("已断开连接")

    def send_data(self):
        if not self.ser:
            return
        data = self.send_edit.text()
        if self.hex_check.isChecked():
            try:
                data_bytes = bytes.fromhex(data)
            except ValueError:
                self.recv_text.append("HEX 格式错误")
                return
        else:
            data_bytes = data.encode('utf-8')
        self.ser.write(data_bytes)
        self.recv_text.append("发送: " + data)

    def update_recv(self, data):
        self.recv_text.append(data)

    def clear_recv(self):
        self.recv_text.clear()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BluetoothDebugger()
    window.show()
    sys.exit(app.exec_())