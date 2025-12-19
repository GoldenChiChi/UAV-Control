import time
from typing import Optional
import json


class TargetTrackBoxClass:
    __slots__ = (
        'timestamp',  # 时间戳
        'camera_type',  # 相机类型
        'tracking_status',  # 跟踪状态
        'target_center_x',  # 目标中心点X坐标
        'target_center_y',  # 目标中心点Y坐标
        'target_width',  # 目标宽度
        'target_height',  # 目标高度
        'center_x',  # 图像中心点
        'center_y'  # 图像中心点
    )

    # 类型注解，初始化时这些参数没有默认值
    function: Optional[int]
    camera_type: Optional[str]
    tracking_status: Optional[bool]
    target_center_x: Optional[int]
    target_center_y: Optional[int]
    target_width: Optional[int]
    target_height: Optional[int]
    center_x: Optional[int]
    center_y: Optional[int]
    checksum: Optional[int]
    tail: Optional[int]

    def __init__(self):
        # 仅初始化时间戳
        self.timestamp = int(time.time() * 1000)

    def parse_data(self, data: bytes):
        # 解析字节数组并赋值
        self.camera_type = 'Visible'
        self.center_x = 640
        self.center_y = 360
        temp_data = []
        for d in data:
            temp_data.append(d)
        self.tracking_status = True if data[1] == 0x01 else False
        self.target_center_x = int(data[2] << 8) + data[3]
        self.target_center_y = int(data[4] << 8) + data[5]
        self.target_width = data[6] * 10
        self.target_height = data[7] * 10
        return self

    def parse_nodata(self):
        return self.parse_data(bytes([0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef]))

    def to_json(self):
        # 将对象转换为JSON字符串
        return json.dumps({slot: getattr(self, slot) for slot in self.__slots__})


def getTrackMessage(operate: bool = True):
    '''
    固定方法，生成跟踪的指令
    operate: 是启动还是关闭
    '''

    # 字段设置
    header = 0xFF
    tracking_status = 0x01 if operate else 0x00
    auto_mode = 0x01
    center_x_high = 0x00
    center_x_low = 0x00
    center_y_high = 0x00
    center_y_low = 0x00
    width = 0x00
    height = 0x00
    tail = 0xEF

    # 构造指令
    command = [
        header, tracking_status, auto_mode, center_x_high, center_x_low, center_y_high, center_y_low, width, height
    ]

    # 计算校验位（从相机标志位到被跟踪目标帧号）
    checksum = sum(command[1:]) & 0xFF  # 取低8位
    command.append(checksum)

    # 添加尾巴
    command.append(tail)

    # 转换为字节数组
    return bytes(command)


class Track:
    def __init__(self, udp_socket, track_space, stop_event):
        self.udp_socket = udp_socket
        self.track_space = track_space
        self.stop_event = stop_event
        pass

    def __check_data(self, data):
        if len(data) < 10:
            return False
        if data[0] != 0xff:
            return False
        if data[-1] != 0xef:
            return False
        if data[-2] != sum(data[1:-2]) & 0xff:
            return False
        return True

    def reciveUdp(self):
        while not self.stop_event.is_set():
            try:
                data, _ = self.udp_socket.recvfrom(1024)  # 接收数据
                if not self.__check_data(data):
                    print("算法数据校验不通过")
                    continue
                targetBoxClass = TargetTrackBoxClass().parse_data(data)
                self.track_space.track = targetBoxClass
            except Exception as e:
                print('udp_listener_algo is error', e)
            finally:
                time.sleep(0.001)
        pass

    def __call__(self, *args, **kwargs):
        pass
