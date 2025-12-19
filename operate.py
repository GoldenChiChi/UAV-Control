import math
import time

from DroneClass import SbusMavlinkConverter
from control_context import OperateHandler
from util import ChannelUtils


class AddArmedOperate(OperateHandler):

    def __init__(self):
        super().__init__()
        self.idle = False
        self.armed = False
        self.mode = "Stabilize"

    def handle_enter_operate(self, sbus, sub_before=None):
        if ChannelUtils.is_add_armed_operate(sbus.channels):
            return True
        return False

    def handler_running_operate(self, sbus):
        self.set_armed = True
        self.armed = False
        sbus.channels[2] = 200
        pass

    def handler_outer_operate(self, sbus):
        self.idle = False
        if not ChannelUtils.is_add_armed_operate(sbus.channels):
            self.set_armed = False
        return not ChannelUtils.is_add_armed_operate(sbus.channels)

    def handler_add_command(self, sbus):
        self.add_command('get_force_disarm')
        pass


class IdleOperate(OperateHandler):

    def __init__(self):
        super().__init__()
        self.out_Idle = False

    def handle_enter_operate(self, sbus, sub_before=None):
        if self.armed and self.idle:
            self.out_Idle = False
            return True
        return False

    def handler_running_operate(self, sbus):
        if sbus.channels[2] > 1100:
            self.out_Idle = True
            self.idle = False
        sbus.channels[0] = 1000
        sbus.channels[1] = 1000
        sbus.channels[2] = 230
        sbus.channels[3] = 1000
        return False

    def handler_outer_operate(self, sbus):
        return self.out_Idle

    def handler_add_command(self, sbus):
        pass


class ArmedOperate(OperateHandler):
    def __init__(self):
        super().__init__()
        self.out_aremd = False

    def handle_enter_operate(self, sbus, sub_before=None):
        self.out_aremd = False
        return not self.armed and ChannelUtils.is_arme_Operate(sbus.channels)

    def handler_running_operate(self, sbus):
        if not ChannelUtils.is_arme_Operate(sbus.channels):
            self.out_aremd = True
            self.set_armed = False
            return False
        if not (ChannelUtils.is_stabilize_mode_operate(sbus.channels)
                or ChannelUtils.is_zero_mode_operate(sbus.channels)):
            if self.mode in ['RTL'.upper(), "AutoTune".upper()]:
                print(self.mode, '模式下不允许进行解锁，请恢复至可以解锁起飞的模式')
            return False
        if ChannelUtils.is_lock_target_operate(sbus.channels) or ChannelUtils.is_Attack_target_operate(sbus.channels):
            print('请恢复锁定和发射通道开关，否则不允许解锁',ChannelUtils.is_lock_target_operate(sbus.channels),ChannelUtils.is_Attack_target_operate(sbus.channels))
            return False
        if self.mode != "Stabilize".upper():
            self.mode = "Stabilize"
        self.set_armed = True
        self.armed = True
        self.idle = True
        sbus.channels[0] = 1000
        sbus.channels[1] = 1000
        sbus.channels[2] = 210
        sbus.channels[3] = 1000
        return False

    def handler_outer_operate(self, sbus):
        return self.out_aremd

    def handler_add_command(self, sbus):
        pass


class BreakRemoteOperate(OperateHandler):

    def __init__(self):
        super().__init__()
        self.sbus = None

    def handle_enter_operate(self, sbus, sub_before=None):
        return not self.remote_connect

    def handler_running_operate(self, sbus):
        if self.armed and not self.idle:
            if self.can_gps:
                self.mode = 'RTL'
            else:
                self.mode = "Stabilize"
        self.sbus = SbusMavlinkConverter(channels=[1000] * 16)
        return False

    def handler_outer_operate(self, sbus):
        return self.remote_connect

    def handler_add_command(self, sbus):
        pass


class LoggerOperate(OperateHandler):

    def __init__(self, config_yaml):
        super().__init__()
        self.config_yaml = config_yaml
        self.pre_time = time.time() * 1000

    def handle_enter_operate(self, sbus, sub_before=None):
        return self.config_yaml['log']['switch']

    def handler_running_operate(self, sbus):
        if self.config_yaml['log']['type'] == 'print':
            if time.time() * 1000 - self.pre_time < self.config_yaml['log']['interval']:
                return True
            if sbus:
                print(int(time.time() * 1000), self.mode, self.remote_connect, self.idle, self.armed, self.can_gps,
                      int(self.armed_time), sbus.channels)
            else:
                print(int(time.time() * 1000), self.mode, self.remote_connect, self.idle, self.armed, self.can_gps,
                      int(self.armed_time), sbus)
            self.pre_time = time.time() * 1000
        return True

    def handler_outer_operate(self, sbus):
        return False

    def handler_add_command(self, sbus):
        pass


class GuideShowOperate(OperateHandler):
    def __init__(self):
        super().__init__()
        self.is_a_target_altitude = False
        self.a_target_time = 0
        self.a_targetAltitude = 5

    def handle_enter_operate(self, sbus, sub_before=None):
        running = ChannelUtils.is_guide_show_operate(sbus.channels)
        if running:
            self.is_a_target_altitude = True
        else:
            self.command_running = False
        return running

    def handler_running_operate(self, sbus):
        sbus.channels[0] = 1000
        sbus.channels[1] = 1000
        sbus.channels[2] = 220
        sbus.channels[3] = 1000
        if not self.armed:
            if self.mode == 'GUIDED':
                self.armed = True
                self.set_armed = True
            self.a_target_time = int(time.time() * 1000)
            self.mode = 'GUIDED'
            return False
        self.command_running = True
        if int(time.time() * 1000) - self.a_target_time > 2000:
            self.command_running = False
        return False

    def handler_outer_operate(self, sbus):
        running = ChannelUtils.is_guide_show_operate(sbus.channels)
        if not running:
            if self.can_gps:
                self.mode = "PosHold"
            else:
                self.mode = "Stabilize"
            return True
        return False

    def handler_add_command(self, sbus):
        self.add_command('get_takeoff_msg', self.a_targetAltitude)
        pass


class ModeRTLOperate(OperateHandler):

    def handle_enter_operate(self, sbus, sub_before=None):
        return True

    def handler_running_operate(self, sbus):
        # 提升返航按钮的优先级
        if ChannelUtils.is_rtl_mode_operate(sbus.channels):
            self.mode = 'rtl'
            return False
        return True

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        pass


class ModeChangeBehindOperate(OperateHandler):

    def handle_enter_operate(self, sbus, sub_before=None):
        return True

    def handler_running_operate(self, sbus):

        mode = 'stabilize'
        if ChannelUtils.is_stabilize_mode_operate(sbus.channels):
            mode = 'stabilize'
        if ChannelUtils.is_fpv_mode_operate(sbus.channels):
            mode = 'land'
        if ChannelUtils.is_autotune_mode_operate(sbus.channels):
            mode = 'autotune'
        if ChannelUtils.is_rtl_mode_operate(sbus.channels):
            self.mode = 'rtl'
        if self.mode != mode.upper():
            self.mode = mode
            return False
        return True

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        pass


class LandModeOperate(OperateHandler):

    def __init__(self):
        super().__init__()
        self.pre_mode = "Stabilize"

    def handle_enter_operate(self, sbus, sub_before=None):
        if self.armed and ChannelUtils.is_land_operate(sbus.channels) and time.time() * 1000 - self.armed_time > 2000:
            if self.mode != "land".upper():
                self.pre_mode = self.mode
            return True
        pass

    def handler_running_operate(self, sbus):
        if self.armed:
            self.mode = 'land'
        pass

    def handler_outer_operate(self, sbus):
        out_flag = not ChannelUtils.is_land_operate(sbus.channels)
        if out_flag:
            self.mode = self.pre_mode
        return out_flag

    def handler_add_command(self, sbus):
        pass


class NoGpsLandModeOperate(OperateHandler):

    def __init__(self):
        super().__init__()

    def handle_enter_operate(self, sbus, sub_before=None):
        if self.armed and self.mode == "STABILIZE".upper():
            return True
        pass

    def handler_running_operate(self, sbus):
        self.mode = 'land'
        sbus.channels[2] = 200
        return False

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        pass


class SpeedZeroOperate(OperateHandler):
    def handler_add_command(self, sbus):
        pass

    def __init__(self, vehicle):
        super().__init__()
        self.vehicle = vehicle
        self.pre_mode = "Stabilize"
        self.command_time = time.time() * 1000
        self.wait_time = 5000

    def handle_enter_operate(self, sbus, sub_before=None):
        if ChannelUtils.is_zero_speed_operate(sbus.channels) \
                and sum(abs(num) for num in self.vehicle.velocity) > 1 \
                and self.mode not in ['RTL'.upper(), "AutoTune".upper(), "land".upper()]:
            self.pre_mode = self.mode
            return True
        return False

    def handler_running_operate(self, sbus):
        if time.time() * 1000 - self.command_time > self.wait_time:
            self.mode = "BRAKE"
            self.command_time = time.time() * 1000
        return False

    def handler_outer_operate(self, sbus):
        if (ChannelUtils.is_zero_speed_operate(sbus.channels)
                and sum(abs(num) for num in self.vehicle.velocity[:2]) > 1
                and time.time() * 1000 - self.command_time < self.wait_time):
            return False
        self.mode = self.pre_mode
        return True


class SteeringOperate(OperateHandler):
    '''
    操作舵机的类
    '''

    def __init__(self):
        super().__init__()

    def handle_enter_operate(self, sbus, sub_before=None):
        return True

    def handler_running_operate(self, sbus):
        return True

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        pass


class GuideAutoOperate(OperateHandler):
    '''
    使用引导的形式进行目标寻找和打击功能。飞行的方式应该是先起飞高度1米，然后再对准朝向进行前进飞行。
    '''

    def __init__(self, vehicle):
        super().__init__()
        self.vehicle = vehicle
        self.have_guide = False
        # 设置飞行距离为target_distance米
        self.target_distance = 20
        # 设置到达预定距离后搜寻目标的时间为search_time秒
        self.search_time = 30
        # 先起飞一定高度到达a点，然后再远飞
        self.a_target_time = 0
        self.a_targetAltitude = 2
        self.is_takeoff = False
        self.takeoff_num = 0
        self.is_forward = False
        self.forward_num = 0
        self.a_forward_time = 0
        # 是否飞行到达目的地
        self.is_arrived = False
        # 是否执行了搜索过程
        self.is_search = False
        self.search_num = 0
        self.a_search_time = 0
        # 无人机的飞控的欧拉角
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.MAX_SPEED = 1

        # 设置 前/右/下速度（m/s）
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_rate = 0

    def init_flag(self):
        self.have_guide = False
        self.is_takeoff = False
        self.takeoff_num = 0
        self.is_forward = False
        self.forward_num = 0
        self.is_arrived = False
        self.is_search = False
        self.search_num = 0
        # 设置 前/右/下速度（m/s）
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_rate = 0

    def calculate_limited_velocity(self, vx, pitch):
        """
        计算限制后的速度分量
        :param vx: 预设前进速度（m/s）
        :param pitch: 当前俯仰角（弧度）
        :return: (vx, vz) 限制后的速度分量
        """
        # 原始计算
        raw_vz = -vx * math.tan(pitch)

        # 计算合速度
        total_speed = math.sqrt(vx ** 2 + raw_vz ** 2)

        # 如果超过MAX_SPEED，按比例缩放
        if total_speed > self.MAX_SPEED:
            scale_factor = self.MAX_SPEED / total_speed
            vx *= scale_factor
            raw_vz *= scale_factor

        return vx, raw_vz

    def handle_enter_operate(self, sbus, subs_before=None):
        # 此处记录下无人机的姿态信息，然后确定飞行的速度。
        if ChannelUtils.is_guide_auto_operate(sbus.channels, subs_before.channels) and self.vehicle.attitude:
            if self.vehicle.attitude.roll:
                self.roll = self.vehicle.attitude.roll
            if self.vehicle.attitude.yaw:
                self.yaw = self.vehicle.attitude.yaw
            if self.vehicle.attitude.pitch:
                self.pitch = self.vehicle.attitude.pitch
            return True
        return False

    def handler_running_operate(self, sbus):
        # 切换到引导模式下进行解锁
        if not self.armed:
            if self.mode == 'GUIDED':
                self.armed = True
                self.set_armed = True
            self.a_target_time = int(time.time() * 1000)
            self.mode = 'GUIDED'
            return False
        self.command_running = True
        if self.takeoff_num < 5:
            self.command_running = True
            return False
        # 起飞悬停1秒钟
        if int(time.time() * 1000) - self.a_target_time < 5000:
            self.command_running = False
            return False
        self.is_takeoff = True
        self.have_guide = True
        # 未达到目的地，则持续飞行
        track_flag = False
        if ChannelUtils.is_lock_target_operate(sbus.channels):
            track_flag = True
        if not self.is_forward:
            self.vx, self.vz = self.calculate_limited_velocity(vx=self.MAX_SPEED, pitch=self.pitch)
            self.a_forward_time = int(time.time() * 1000)
            if self.forward_num > 5:
                self.is_forward = True
            self.forward_num += 1
            return track_flag
        if not self.is_arrived:
            # 持续判断当前无人机飞行距离是否达到预设距离了。
            t = int(time.time() * 1000) - self.a_forward_time
            # 计算各轴距离
            distance_x = self.vx * t
            distance_z = abs(self.vz) * t
            # 三维直线距离
            total_distance = math.sqrt(distance_x ** 2 + distance_z ** 2)
            if total_distance > self.target_distance * 1000:
                self.is_arrived = True
            return track_flag
        if not self.is_search:
            # 悬停无人机
            self.vx = 0.2
            self.vy = 0.2
            self.vz = 0
            self.yaw_rate = 10.0
            if self.search_num > 5:
                self.is_search = True
            self.search_num += 1
            self.a_search_time = int(time.time() * 1000)
            return track_flag
        if int(time.time() * 1000) - self.a_search_time < self.search_time * 1000:
            return track_flag

        self.command_running = False
        self.mode = "RTL"
        return False

    def handler_outer_operate(self, sbus):
        if (self.have_guide and not self.armed) or not ChannelUtils.is_guide_auto_operate(sbus.channels, None):
            self.init_flag()
            return True
        return False

    def handler_add_command(self, sbus):
        if not self.is_takeoff:
            self.add_command('get_takeoff_msg', self.a_targetAltitude)
            self.a_target_time = int(time.time() * 1000)
            self.takeoff_num += 1
            return
        self.add_command('get_force_speed_body', self.vx, self.vy, self.vz, self.yaw_rate)
        pass
