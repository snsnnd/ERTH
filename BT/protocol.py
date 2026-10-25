# protocol.py
import binascii
import struct


class BLEProtocol:
    """
    BLE通信协议类
    """

    # 服务和特征UUID
    SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
    WRITE_CHAR_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"
    NOTIFY_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

    # 协议标识
    FRAME_HEADER = bytes([0xAA, 0xBB])
    FRAME_FOOTER = bytes([0xCC, 0xDD])
    ACK_SIGNAL = bytes([0x55, 0xAA])

    # 命令类型
    CMD_PID_UPDATE = 0x10
    CMD_PID_READ = 0x11
    CMD_DATA_REQUEST = 0x20
    CMD_DATA_RESPONSE = 0x21

    @staticmethod
    def create_file_frame(file_index, file_data):
        """创建文件传输帧"""
        header = BLEProtocol.FRAME_HEADER + bytes([file_index + 1])
        footer = BLEProtocol.FRAME_FOOTER
        hex_data = binascii.hexlify(file_data)
        return header + hex_data + footer

    @staticmethod
    def create_pid_command(motor_id, kp, ki, kd):
        """创建PID参数更新命令"""
        cmd = bytearray([BLEProtocol.CMD_PID_UPDATE, motor_id])
        cmd.extend(struct.pack('f', kp))
        cmd.extend(struct.pack('f', ki))
        cmd.extend(struct.pack('f', kd))
        return bytes(cmd)

    @staticmethod
    def create_pid_read_command(motor_id):
        """创建读取PID参数命令"""
        return bytes([BLEProtocol.CMD_PID_READ, motor_id])

    @staticmethod
    def create_data_request(command_type):
        """创建数据请求命令"""
        return bytes([BLEProtocol.CMD_DATA_REQUEST, command_type])

    @staticmethod
    def parse_data_response(data):
        """解析设备返回的数据响应"""
        if len(data) < 2 or data[0] != BLEProtocol.CMD_DATA_RESPONSE:
            return None

        response_type = data[1]
        payload = data[2:]

        result = {
            'type': response_type,
            'payload': payload
        }

        # 解析不同类型数据
        if response_type == 0x01 and len(payload) >= 12:  # 传感器数据
            result['sensor_data'] = {
                'gyro_x': struct.unpack('f', payload[0:4])[0],
                'gyro_y': struct.unpack('f', payload[4:8])[0],
                'gyro_z': struct.unpack('f', payload[8:12])[0]
            }
        elif response_type == 0x02 and len(payload) >= 13:  # PID参数
            result['motor_id'] = payload[0]
            result['pid_params'] = {
                'kp': struct.unpack('f', payload[1:5])[0],
                'ki': struct.unpack('f', payload[5:9])[0],
                'kd': struct.unpack('f', payload[9:13])[0]
            }

        return result

    @staticmethod
    def is_ack(data):
        """判断是否为ACK确认信号"""
        return data == BLEProtocol.ACK_SIGNAL
