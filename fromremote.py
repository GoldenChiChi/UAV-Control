import time
from DroneClass import SbusMavlinkConverter


class SerialSbusReader:
    def __init__(self, sbus_serial, sbus_space, stop_event):
        self.sbus_serial = sbus_serial
        self.sbus_space = sbus_space
        self.stop_event = stop_event

    def read_sbus_packet(self):
        while not self.stop_event.is_set():
            try:
                # 读取数据，尝试找到0x0f开头的有效数据包
                data = self.sbus_serial.read(1)
                if data == b'\x0f':  # 检查是否以0x0f开头
                    packet = data + self.sbus_serial.read(34)  # 读取后续的34个字节
                    if len(packet) == 35:
                        return packet
                else:
                    # 如果不是0x0f开头，清空缓冲区并继续读取
                    # print("数据包错误，清空缓冲区")
                    self.sbus_serial.reset_input_buffer()
                    time.sleep(0.1)
            except Exception as e:
                # print('SerialSbusReader read_sbus_packet', e)
                return None
        return None

    def validate_packet(self, packet):
        # 在这里进行数据包的校验，返回True表示有效，False表示无效
        # 你可以根据需要添加CRC校验或其他规则
        if packet:
            return True
        return False

    def sbusTosbusMavlink(self, packet):
        if self.validate_packet(packet):
            return SbusMavlinkConverter().from_sbus(packet)
        return None

    def __call__(self):
        while not self.stop_event.is_set():
            try:
                packet = self.read_sbus_packet()
                current_sbus = self.sbusTosbusMavlink(packet)
                if hasattr(self.sbus_space, 'sbus'):
                    self.sbus_space.sbus_before = self.sbus_space.sbus
                self.sbus_space.sbus = current_sbus
            except Exception as e:
                print('SerialSbusReader __call__:', e)
            finally:
                time.sleep(0.01)
