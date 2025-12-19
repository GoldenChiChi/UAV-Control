import json
import time
from typing import Optional


class VehicleClass:
    def __init__(self, version=None, capabilities=None, location=None, attitude=None,
                 velocity=None, gps=None, gimbal=None, battery=None, ekf_ok=False,
                 last_heartbeat=0.0, rangefinder=None, heading=0, is_armable=False,
                 system_status="", groundspeed=0.0, airspeed=0.0, mode="", armed=False, parameters=None,
                 channels=None, armed_time=int(time.time() * 1000)):
        self.version = version or {}
        self.capabilities = capabilities or {}
        self.location = location or {}
        self.attitude = attitude or {}
        self.velocity = velocity or ()
        self.gps = gps or {}
        self.gimbal = gimbal or {}
        self.battery = battery or {}
        self.ekf_ok = ekf_ok
        self.last_heartbeat = last_heartbeat
        self.rangefinder = rangefinder or {}
        self.heading = heading
        self.is_armable = is_armable
        self.system_status = system_status
        self.groundspeed = groundspeed
        self.airspeed = airspeed
        self.mode = mode
        self.armed = armed
        self.parameters = parameters or {}
        self.channels = channels or [None] * 8
        self.armed_time = armed_time

    def to_json(self):
        """序列化对象为 JSON 字符串。"""
        return json.dumps(self.__dict__)

    def to_proto(self):
        # 创建一个新的 Vehicle Protobuf 对象
        import vehicle_pb2
        vehicle_proto = vehicle_pb2.Vehicle()

        # 填充 version 信息
        vehicle_proto.version.major = self.version.get('major', 0)
        vehicle_proto.version.minor = self.version.get('minor', 0)
        vehicle_proto.version.patch = self.version.get('patch', 0)
        vehicle_proto.version.release_type = self.version.get('release_type', "")
        vehicle_proto.version.release_version = self.version.get('release_version', 0)
        vehicle_proto.version.is_stable = self.version.get('is_stable', False)

        # 填充 capabilities 信息
        for cap, value in self.capabilities.items():
            setattr(vehicle_proto.capabilities, cap, value)

        # 填充 location 信息
        vehicle_proto.location.global_frame = self.location.get('global_frame', "")
        vehicle_proto.location.global_relative_frame = self.location.get('global_relative_frame', "")
        vehicle_proto.location.local_frame = self.location.get('local_frame', "")
        vehicle_proto.location.alt = self.location.get('alt', 0.0)

        # 填充 attitude 信息
        vehicle_proto.attitude.pitch = self.attitude.get('pitch', 0.0) if self.attitude and self.attitude.get(
            'pitch') is not None else 0.0
        vehicle_proto.attitude.yaw = self.attitude.get('yaw', 0.0) if self.attitude and self.attitude.get(
            'yaw') is not None else 0.0
        vehicle_proto.attitude.roll = self.attitude.get('roll', 0.0) if self.attitude and self.attitude.get(
            'roll') is not None else 0.0

        # 填充 velocity 信息
        if isinstance(self.velocity, (list, tuple)):
            for v in self.velocity:
                vehicle_proto.velocity.velocity.append(v if v is not None else 0.0)  # 处理 None 值
        else:
            vehicle_proto.velocity.velocity.append(self.velocity if self.velocity is not None else 0.0)

        # 填充 GPS 信息
        vehicle_proto.gps.fix_type = self.gps.get('fix_type', 0) if self.gps else 0
        vehicle_proto.gps.satellites_visible = self.gps.get('satellites_visible', 0) if self.gps else 0
        vehicle_proto.gps.epv = self.gps.get('epv', 0.0) if self.gps and self.gps.get('epv') is not None else 0.0
        vehicle_proto.gps.eph = self.gps.get('eph', 0.0) if self.gps and self.gps.get('eph') is not None else 0.0
        vehicle_proto.gps.lat = self.gps.get('lat', 0.0) if self.gps and self.gps.get('lat') is not None else 0.0
        vehicle_proto.gps.lon = self.gps.get('lon', 0.0) if self.gps and self.gps.get('lon') is not None else 0.0

        # 填充 gimbal 信息
        vehicle_proto.gimbal.pitch = self.gimbal.get('pitch', 0.0) if self.gimbal and self.gimbal.get(
            'pitch') is not None else 0.0
        vehicle_proto.gimbal.yaw = self.gimbal.get('yaw', 0.0) if self.gimbal and self.gimbal.get(
            'yaw') is not None else 0.0
        vehicle_proto.gimbal.roll = self.gimbal.get('roll', 0.0) if self.gimbal and self.gimbal.get(
            'roll') is not None else 0.0

        # 填充 battery 信息
        vehicle_proto.battery.voltage = self.battery.get('voltage', 0.0) if self.battery and self.battery.get(
            'voltage') is not None else 0.0
        vehicle_proto.battery.current = self.battery.get('current', 0.0) if self.battery and self.battery.get(
            'current') is not None else 0.0
        vehicle_proto.battery.level = self.battery.get('level', 0.0) if self.battery and self.battery.get(
            'level') is not None else 0.0

        # 填充其他信息
        vehicle_proto.ekf_ok = self.ekf_ok
        vehicle_proto.last_heartbeat = self.last_heartbeat
        vehicle_proto.rangefinder.distance = self.rangefinder.get('distance',
                                                                  0.0) if self.rangefinder and self.rangefinder.get(
            'distance') is not None else 0.0
        vehicle_proto.rangefinder.voltage = self.rangefinder.get('voltage',
                                                                 0.0) if self.rangefinder and self.rangefinder.get(
            'voltage') is not None else 0.0

        vehicle_proto.heading = self.heading
        vehicle_proto.is_armable = self.is_armable
        vehicle_proto.system_status = self.system_status
        vehicle_proto.groundspeed = self.groundspeed
        vehicle_proto.airspeed = self.airspeed
        vehicle_proto.mode = self.mode
        vehicle_proto.armed = self.armed
        vehicle_proto.armed_time = self.armed_time

        # 填充 parameters 信息
        for param, value in self.parameters.items():
            vehicle_proto.parameters[param] = value if value else 0.0

        # 填充 channels 信息，过滤掉 None 或 null 值
        valid_channels = [ch if ch is not None else 0 for ch in self.channels]
        vehicle_proto.channels.extend(valid_channels)

        return vehicle_proto


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


class TargetTrackBoxClass:
    __slots__ = (
        'timestamp',  # 时间戳
        'camera_type',  # 相机类型
        'tracking_status',  # 跟踪状态
        'target_center_x',  # 目标中心点X坐标
        'target_center_y',  # 目标中心点Y坐标
        'target_width',  # 目标宽度
        'target_height',  # 目标高度
        'center_x',
        'center_y'
    )

    # 类型注解，初始化时这些参数没有默认值
    function: Optional[int]
    camera_type: Optional[str]
    tracking_status: Optional[bool]
    target_center_x: Optional[int]
    target_center_y: Optional[int]
    target_width: Optional[int]
    target_height: Optional[int]



    checksum: Optional[int]
    tail: Optional[int]

    def __init__(self):
        # 仅初始化时间戳
        self.timestamp = int(time.time() * 1000)
        self.center_x = 320
        self.center_y = 256

    # def parse_data(self, data: bytes):
    #     # 解析字节数组并赋值
    #     self.camera_type = 'Infra' if data[1] == 0x00 else 'Visible'
    #     self.tracking_status = True if data[2] == 0x01 else False
    #     self.target_center_x = (data[3] << 8) + data[4]
    #     self.target_center_y = (data[5] << 8) + data[6]
    #     self.target_width = data[7]
    #     self.target_height = data[8]
    #     return self

    def parse_data(self, data: bytes):
        temp_data = []
        for d in data:
            temp_data.append(d)
        # 解析字节数组并赋值
        self.camera_type = 'Visible'
        self.tracking_status = True if data[1] == 0x01 else False
        self.target_center_x = int(data[2] << 8) + data[3]
        self.target_center_y = int(data[4] << 8) + data[5]
        self.target_width = data[6] * 10
        self.target_height = data[7] * 10
        return self

    def to_json(self):
        # 将对象转换为JSON字符串
        return json.dumps({slot: getattr(self, slot) for slot in self.__slots__})


class GuidedControl:
    # setATargetAltitude, setLonLat, goto_time, setHomeLonLat, goto_home_time

    __slots__ = (
        'setATargetAltitude',  # 是否已经设置了起飞高度
        'setLonLat',  # 是否设置了飞到固定经纬度的
        'goto_time',  # 设置飞到固定经纬度的时间，单位毫秒
        'setHomeLonLat',  # 目标中心点X坐标
        'goto_home_time',  # 目标中心点Y坐标
        'armed_time',  # 目标宽度
        'idlePaddle',  # 目标高度
    )


class VehicleControl:
    __slots__ = (
        'frame_center_x',  # 相机中心点像素
        'frame_center_y',  # 相机中心点像素
        'x_pid_control',  # 跟踪状态
        'y_pid_control',  # 目标中心点X坐标
        'target_box',  # 目标中心点Y坐标
        'armed_time',  # 目标宽度
        'idlePaddle',  # 目标高度
    )

    # setATargetAltitude = False
    # setLonLat = False
    # setHomeLonLat = False
    # goto_time = 0
    # goto_home_time = 0
    # frame_center_x = 640
    # frame_center_y = 360


# 参数列表
param_names = [
    "SYSID_SW_MREV", "SYSID_THISMAV", "SYSID_MYGCS", "PILOT_THR_FILT", "PILOT_TKOFF_ALT",
    "PILOT_TKOFF_DZ", "PILOT_THR_BHV", "SERIAL0_BAUD", "SERIAL0_PROTOCOL", "SERIAL1_PROTOCOL",
    "SERIAL1_BAUD", "SERIAL2_PROTOCOL", "SERIAL2_BAUD", "SERIAL3_PROTOCOL", "SERIAL3_BAUD",
    "SERIAL4_PROTOCOL", "SERIAL4_BAUD", "SERIAL5_PROTOCOL", "SERIAL5_BAUD", "SERIAL6_PROTOCOL",
    "SERIAL6_BAUD", "TELEM_DELAY", "GCS_PID_MASK", "RTL_ALT", "RTL_CONE_SLOPE", "RTL_SPEED",
    "RTL_ALT_FINAL", "RTL_CLIMB_MIN", "RTL_LOIT_TIME", "RNGFND_GAIN", "FS_GCS_ENABLE",
    "GPS_HDOP_GOOD", "MAG_ENABLE", "SUPER_SIMPLE", "WP_YAW_BEHAVIOR", "LAND_SPEED",
    "LAND_SPEED_HIGH", "PILOT_SPEED_UP", "PILOT_ACCEL_Z", "FS_THR_ENABLE", "FS_THR_VALUE",
    "THR_DZ", "FLTMODE1", "FLTMODE2", "FLTMODE3", "STAT_RUNTIME", "FLTMODE4", "FLTMODE5"
]


def vehicle_to_vehicle_class(vehicle, armed_time):
    parameters = {}
    for param_name in param_names:
        parameters[param_name] = vehicle.parameters.get(param_name, wait_ready=False)
    channels = [None] * 8
    for i in range(8):
        channels[i] = vehicle.channels[i + 1]
    return VehicleClass(
        version={
            "major": vehicle.version.major,
            "minor": vehicle.version.minor,
            "patch": vehicle.version.patch,
            "release_type": vehicle.version.release_type(),
            "release_version": vehicle.version.release_version(),
            "is_stable": vehicle.version.is_stable(),
        },
        capabilities={
            "mission_float": vehicle.capabilities.mission_float,
            "param_float": vehicle.capabilities.param_float,
            "mission_int": vehicle.capabilities.mission_int,
            "command_int": vehicle.capabilities.command_int,
            "param_union": vehicle.capabilities.param_union,
            "ftp": vehicle.capabilities.ftp,
            "set_attitude_target": vehicle.capabilities.set_attitude_target,
            "set_attitude_target_local_ned": vehicle.capabilities.set_attitude_target_local_ned,
            "set_altitude_target_global_int": vehicle.capabilities.set_altitude_target_global_int,
            "terrain": vehicle.capabilities.terrain,
            "set_actuator_target": vehicle.capabilities.set_actuator_target,
            "flight_termination": vehicle.capabilities.flight_termination,
            "compass_calibration": vehicle.capabilities.compass_calibration,
        },
        location={
            "global_frame": str(vehicle.location.global_frame),
            "global_relative_frame": str(vehicle.location.global_relative_frame),
            "local_frame": str(vehicle.location.local_frame),
            "alt": vehicle.location.global_relative_frame.alt,
        },
        attitude={
            "pitch": vehicle.attitude.pitch,
            "yaw": vehicle.attitude.yaw,
            "roll": vehicle.attitude.roll,
        },
        velocity=vehicle.velocity,
        gps={
            "fix_type": vehicle.gps_0.fix_type,
            "satellites_visible": vehicle.gps_0.satellites_visible,
            "epv": vehicle.gps_0.epv,
            "eph": vehicle.gps_0.eph,
            "lat": vehicle.location.global_frame.lat,
            "lon": vehicle.location.global_frame.lon,
        },
        gimbal={
            "pitch": vehicle.gimbal.pitch,
            "yaw": vehicle.gimbal.yaw,
            "roll": vehicle.gimbal.roll,
        },
        battery={
            "voltage": vehicle.battery.voltage,
            "current": vehicle.battery.current,
            "level": vehicle.battery.level,
        },
        ekf_ok=vehicle.ekf_ok,
        last_heartbeat=vehicle.last_heartbeat,
        rangefinder={
            "distance": vehicle.rangefinder.distance,
            "voltage": vehicle.rangefinder.voltage,
        },
        heading=vehicle.heading,
        is_armable=vehicle.is_armable,
        system_status=vehicle.system_status.state,
        groundspeed=vehicle.groundspeed,
        airspeed=vehicle.airspeed,
        mode=vehicle.mode.name,
        armed=vehicle.armed,
        armed_time=armed_time,
        parameters=parameters,
        channels=channels
    )


if __name__ == '__main__':
    b = bytes([255, 1, 3, 181, 1, 32, 67, 56, 85, 239])
    print(b)
    t = TargetTrackBoxClass().parse_data(b)

    print(t.to_json())

    #
    #
    #
    #
    # # 示例使用
    #
    # # # 创建一个SbusMavlinkConverter对象
    # # sbus_data = SbusMavlinkConverter(
    # #     [1000, 1500, 2000, 1750, 1200, 1600, 1800, 1300, 1000, 1500, 2000, 1750, 1200, 1600, 1800, 1300], 1, 1)
    # #
    # # # 将对象转换为JSON
    # # json_data = sbus_data.to_json()
    # # print("转换为JSON:", json_data)
    # #
    # # # 从JSON创建对象
    # # new_data = SbusMavlinkConverter.from_json(json_data)
    # # print("从JSON创建对象:", new_data.channels, new_data.connection_status, new_data.checksum_valid)
    # #
    # # # 将SBUS数据转换为MAVLink数据
    # # mavlink_data = sbus_data.sbus_to_mavlink()
    # # print("转换为MAVLink数据:", mavlink_data)
    # #
    # # # 将MAVLink数据转换回SBUS数据
    # # sbus_data.mavlink_to_sbus(mavlink_data)
    # # print("MAVLink数据转换回SBUS:", sbus_data.channels, sbus_data.connection_status, sbus_data.checksum_valid)
    #
    # vehicle_data = {
    #     "version": {"firmware_version": "1.0.0", "major": 1, "minor": 0, "patch": 0},
    #     "capabilities": {"mission_float": True},
    #     "location": {"global_frame": "Lat: 34.5, Lon: -117.5, Alt: 1000m"},
    #     "attitude": {"pitch": 0.01, "yaw": 1.57, "roll": -0.01},
    #     "velocity": (1.0, 0.0, 0.0),
    #     "gps": {"fix_type": 3, "satellites_visible": 8},
    #     "gimbal": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
    #     "battery": {"voltage": 12.6, "current": 5.2, "level": 80},
    #     "ekf_ok": True,
    #     "last_heartbeat": 0.1,
    #     "rangefinder": {"distance": 5.0, "voltage": 3.3},
    #     "heading": 90,
    #     "is_armable": True,
    #     "system_status": "ACTIVE",
    #     "groundspeed": 5.5,
    #     "airspeed": 10.0,
    #     "mode": "GUIDED",
    #     "armed": True,
    #     "parameters": {"param1": 10, "param2": 20}
    # }
    #
    # # 创建Vehicle对象并赋值
    # vehicle = VehicleClass(**vehicle_data)
    # vehicle.armed_time = int(time.time() * 1000)
    # # vehicle.channels = [1000]*16
    # # 序列化为JSON并打印
    # print(vehicle.to_json())
    # print(vehicle.to_proto())
