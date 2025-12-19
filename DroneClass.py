import json
import time


class SbusMavlinkConverter:
    def __init__(self, channels=None, connection_status=0, checksum_valid=0):
        """
        初始化SbusMavlinkConverter对象

        :param channels: 通道数据，列表形式存储，每个元素是一个通道的值 (默认为空列表)

            1. 遥控器通道的概念
            遥控器通道（RC Channels） 代表了从遥控器发送到飞控（飞行控制器）的独立控制信号。每个通道通常对应一个独立的控制功能，例如：

            通道1：横滚（Roll）
            通道2：俯仰（Pitch）
            通道3：油门（Throttle）
            通道4：偏航（Yaw）
            通道5-16：用于控制其他辅助功能，如起落架、相机控制、模式切换等
            更多的通道数 意味着你可以控制更多的功能，从而实现更复杂和精细的操作。


            2.1 功能差异
            8通道：

            基本控制：通常前4个通道用于基本飞行控制（Roll、Pitch、Throttle、Yaw）。
            辅助功能：剩余的通道可用于简单的辅助功能，如模式切换、LED控制等。
            适用场景：适合一般的航拍、娱乐飞行等，对复杂功能需求不高的场景。
            16通道：

            扩展控制：除了基本飞行控制外，还有更多通道可用于复杂的辅助功能。
            高级应用：可用于控制云台、多种传感器、复杂的飞行任务、特殊动作等。
            适用场景：专业航拍、工业应用、竞速无人机等需要复杂控制的场景。

        :param connection_status: 连接状态，整数类型 (默认为0)
        :param checksum_valid: 校验状态，整数类型 (默认为0)
        """
        self.channels = channels if channels is not None else [0] * 16
        self.connection_status = connection_status
        self.checksum_valid = checksum_valid
        self.timeStamp = int(time.time() * 1000)

    def from_sbus(self, packet):
        channels = [0] * 16
        for i in range(1, 33, 2):
            channels[i // 2] = (packet[i] << 8) + packet[i + 1]
        self.channels = channels
        self.connection_status = int(packet[-2])
        self.checksum_valid = int(packet[-1])
        return self

    def to_json(self):
        """
        将对象转换为JSON字符串
        :return: JSON格式的字符串
        """
        return json.dumps({
            'channels': self.channels,
            'connection_status': self.connection_status,
            'checksum_valid': self.checksum_valid
        })

    @classmethod
    def from_json(cls, json_data):
        """
        从JSON字符串创建SbusMavlinkConverter对象
        :param json_data: JSON格式的字符串
        :return: SbusMavlinkConverter对象
        """
        data = json.loads(json_data)
        return cls(
            channels=data['channels'],
            connection_status=data['connection_status'],
            checksum_valid=data['checksum_valid']
        )

    def sbus_to_mavlink(self):
        """
        将SBUS数据转换为MAVLink数据
        :return: MAVLink数据，具体实现根据MAVLink协议进行转换
        """
        # 模拟MAVLink数据转换，可以根据MAVLink协议进行实现
        mavlink_data = {
            'mavlink_channels': self.channels,  # 假设MAVLink通道数据与SBUS相同
            'mavlink_connection_status': self.connection_status,
            'mavlink_checksum_valid': self.checksum_valid
        }
        return mavlink_data

    def mavlink_to_sbus(self, mavlink_data):
        """
        将MAVLink数据转换为SBUS数据
        :param mavlink_data: MAVLink数据
        """
        # 假设MAVLink通道数据与SBUS相同，可以根据实际需要调整
        self.channels = mavlink_data.get('mavlink_channels', [0] * 8)
        self.connection_status = mavlink_data.get('mavlink_connection_status', 0)
        self.checksum_valid = mavlink_data.get('mavlink_checksum_valid', 0)


class Gps():
    def __init__(self, lat=0, lon=0, alt=0, fix_type=0, eph=9999):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.fix_type = fix_type
        self.eph = eph

    __slots__ = ('lat', "lon", "alt", 'fix_type', 'eph')


class Better():
    def __init__(self, v=0, a=0, b=0):
        self.v = v
        self.a = a
        self.b = b

    __slots__ = ('v', "a", "b")


class Speed():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    __slots__ = ('x', "y", "z")


class Attitude():
    def __init__(self, course=0, heading=0):
        self.course = course
        self.heading = heading


class RemoteInfo():
    def __init__(self):
        self.apmConnect = 0
        self.apmMode = 0
        self.armable = 0
        self.armed = 0
        self.armedTime = 0
        self.gps = Gps()
        self.home = Gps()
        self.better = Better()
        self.attitude = Attitude()
        self.track = 0
        self.speed = Speed()
        self.warning = 0

    __slots__ = (
        'apmConnect', 'apmMode', 'armable', 'armed', 'armedTime', 'tracking_status', 'gps', 'home', 'better', 'speed',
        'attitude', 'track', 'warning')
