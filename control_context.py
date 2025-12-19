import time
from dronekit import VehicleMode
from abc import ABC, abstractmethod
from util import ChannelUtils, MAVLinkCommands
from pymavlink import mavutil

class OperateHandler(ABC):

    def __init__(self):
        self.running = False
        self.outer = False
        self.next = True

        self.command_running = False
        self.exec_mode = False
        self.exec_channel = False
        self.mode = None
        self.set_armed = False
        self.armed = False
        self.idle = False
        self.can_gps = False
        self.remote_connect = False
        self.armed_time = 0
        self.commands_msg = []
        self.GLOBAL_POSITION = None
        self.attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.acceleration = [0, 0, 0]  # 三轴加速度 (m/s²)
        self.angular_velocity = [0, 0, 0]  # 角速度 (rad/s)

    @abstractmethod
    def handle_enter_operate(self, sbus, sub_before=None):
        pass

    @abstractmethod
    def handler_running_operate(self, sbus):
        pass

    @abstractmethod
    def handler_outer_operate(self, sbus):
        pass

    @abstractmethod
    def handler_add_command(self, sbus):
        pass

    def exec_handler(self, sbus, sbus_before):
        self.next = True
        if not self.running:
            self.running = self.handle_enter_operate(sbus, sbus_before)
        if self.running:
            self.next = self.handler_running_operate(sbus)
            self.outer = self.handler_outer_operate(sbus)
            if self.outer:
                self.running = False
        if self.command_running:
            self.handler_add_command(sbus)
        return self.next

    def add_command(self, method_name, *args):
        """Adds a command method and its arguments to the command list."""
        self.commands_msg.append((method_name, args))


class ControlContext:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._setup_message_listeners()
        self.sbus = None
        self.sbus_before = None
        self.remote_connect = False
        self.mode = "Stabilize"
        self.command_running = False
        self.idle = False

        self.arme_time = 0
        self.commands_msg = []
        self.operate_handlers = []
        self.can_gps = False
        # 无人机的飞行参数
        # {time_boot_ms: 560133, lat: 0, lon: 0, alt: 540, relative_alt: 149647, vx: 221, vy: -367, vz: 265, hdg: 14058}
        # print(self.GLOBAL_POSITION.hdg * 1e-2)
        self.GLOBAL_POSITION = None
        self.attitude = {'roll': 0, 'pitch': 0, 'yaw': 0, 'vyaw': 0, 'time_ms': 0}

        self.acceleration = [0, 0, 0]  # 三轴加速度 (m/s²)
        self.angular_velocity = [0, 0, 0]  # 角速度 (rad/s)
        self._last_command_time = 0
        pass

    def __update_vehicle_mode(self, mode):
        if mode and self.vehicle.mode.name.upper() != mode.upper():
            self.vehicle.mode = VehicleMode(mode)

    def update_vehicle_state(self):
        # 这是一个丢给下面实现的方法，保持数据和控制状态同步
        self.mode = self.vehicle.mode.name
        self.commands_msg = []
        for handler in self.operate_handlers:
            if self.vehicle.armed and not handler.armed:
                self.arme_time = int(time.time())
            handler.armed = self.vehicle.armed
            handler.mode = self.mode
            handler.idle = self.idle
            handler.can_gps = int(self.vehicle.gps_0.eph / 100.0) < 2 and self.vehicle.gps_0.satellites_visible >= 6
            handler.remote_connect = self.remote_connect
            handler.armed_time = self.arme_time
            handler.commands_msg = []
            handler.GLOBAL_POSITION = self.GLOBAL_POSITION
            handler.attitude = self.attitude
            handler.acceleration = self.acceleration
            handler.angular_velocity = self.angular_velocity
        pass

    def update_remote_status(self, sbus, sbus_before):
        self.sbus = sbus
        self.sbus_before = sbus_before
        self.remote_connect = sbus and time.time() * 1000 - sbus.timeStamp < 2000 and sbus.connection_status == 0
        pass

    def add_operate_handler(self, operate_handler: OperateHandler):
        self.operate_handlers.append(operate_handler)

    def execute(self):

        for handler in self.operate_handlers:
            handler.exec_handler(self.sbus, self.sbus_before)
            if handler.running or handler.outer:
                if not self.remote_connect:
                    self.sbus = getattr(handler, 'sbus', None)
                self.mode = handler.mode
                self.command_running = self.command_running | handler.command_running
                if handler.command_running:
                    self.commands_msg += handler.commands_msg
                self.idle = handler.idle
                if handler.set_armed:
                    self.vehicle.armed = handler.armed
                # self.vehicle.armed = handler.armed
                handler.outer = False
            if not handler.next:
                break
        if self.command_running:
            # 执行特殊的指令
            self.exec_commands()
            self.command_running = False
        self.exec_mode()
        # 适配遥控器的通道值
        self.exec_channel()
        # 记录无人机的状态，回传给返回端进程
        self.record_vehicle()

    def exec_mode(self):
        self.__update_vehicle_mode(self.mode.upper())
        pass

    def exec_commands(self):
        for method_name, args in self.commands_msg:
            method = getattr(MAVLinkCommands, method_name, None)
            if method:
                msg = method(self.vehicle, *args)
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()
        pass

    def exec_channel(self):
        # print('exec_channel', self.sbus.to_json())
        if self.sbus:
            channels = ChannelUtils.normalize_channels(self.sbus.channels)
            self.vehicle.channels.overrides = {1: ChannelUtils.constrain(channels[0], 1500, 500),
                                               2: ChannelUtils.constrain(channels[1], 1500, 500),
                                               3: ChannelUtils.constrain(channels[2], 1500, 500),
                                               4: ChannelUtils.constrain(channels[3], 1500, 300)}
        pass

    def record_vehicle(self):
        pass

    def _set_message_interval(self, msg_id, interval_us):
        """ 安全的频率设置方法 """
        try:
            self.vehicle.message_factory.command_long_send(
                0, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # confirmation
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f"指令发送失败: {e}")

    def _setup_message_listeners(self):
        rate_hz = 20

        @self.vehicle.on_message('GLOBAL_POSITION_INT')
        def handle_gps(_, name, msg):
            self.GLOBAL_POSITION = msg
            if time.time() * 1000 - self._last_command_time > 2000:
                '''设置消息上报的频率'''
                self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, int(1e6 / rate_hz))
                self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, int(1e6 / rate_hz))
                self._last_command_time = time.time() * 1000

        @self.vehicle.on_message('ATTITUDE')
        def handle_attitude(_, name, msg):
            current_time = int(time.time() * 1000)
            current_yaw = msg.yaw
            vyaw = msg.yawspeed
            # 更新当前姿态
            self.attitude = {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': current_yaw,
                'vyaw': vyaw,
                'time_ms': current_time
            }
