import math
import time

from lock_target import LockTargetWithGpsOperate, LockTargetNoGpsOperate
from util import ChannelUtils
from simple_pid import PID


class GuideLockTargetWithGpsOperate(LockTargetWithGpsOperate):
    def __init__(self, vehicle, track_space, servo_control, udp_socket, config_yaml):
        # 1. 先初始化父类参数
        super().__init__(track_space, servo_control, udp_socket, config_yaml)
        # 2. 子类独有参数（覆盖父类同名参数）
        self.vehicle = vehicle
        self.have_guide = False
        self.target_distance = 30
        self.search_time = 60
        self.wait_num = 10
        self.a_target_time = 0
        self.a_targetAltitude = 2
        self.is_takeoff = False
        self.takeoff_num = 0
        self.is_forward = False
        self.forward_num = 0
        self.a_forward_time = 0
        self.is_arrived = False
        self.is_search = False
        self.search_num = 0
        self.a_search_time = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.yaw_temp = 0
        self.MAX_SPEED = 3

        # 覆盖父类的速度参数（如果需要独立）
        self.vx = 0  # 覆盖父类的 self.vx = 0.1
        self.vy = 0  # 覆盖父类的 self.vy = 0.1
        self.vz = 0  # 保持与父类一致
        self.yaw_rate = 5  # 覆盖父类的 self.yaw_rate = 0.0
        self.in_track = False
        self.track_search = False
        self.have_track = False
        self.track_search_time = 0

        self.before_set_yaw = 0

        self.temp_before_sbus = None

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
        self.vd = 0

        self.in_track = False
        self.track_search = False
        self.have_track = False
        self.track_search_time = 0
        self.command_running = False
        self.temp_before_sbus = None

    def calculate_limited_velocity(self, vx, pitch):
        """
        计算限制后的速度分量
        :param vx: 预设前进速度（m/s）
        :param pitch: 当前俯仰角（弧度）
        :return: (vx, vz) 限制后的速度分量
        """
        # 原始计算
        vz = -vx * math.tan(pitch)

        # 计算合速度
        total_speed = math.sqrt(vx ** 2 + vz ** 2)

        # 如果超过MAX_SPEED，按比例缩放
        if total_speed > self.MAX_SPEED:
            scale_factor = self.MAX_SPEED / total_speed
            vx *= scale_factor
            vz *= scale_factor

        return vx, vz

    def handle_enter_operate(self, sbus, subs_before=None):
        if self.temp_before_sbus is None:
            self.temp_before_sbus = sbus
            return False
        # 此处记录下无人机的姿态信息，然后确定飞行的速度。
        operate_flag = ChannelUtils.is_guide_auto_operate(sbus.channels, self.temp_before_sbus.channels)
        self.temp_before_sbus = sbus
        if not self.have_guide and operate_flag and self.can_gps and not self.armed:
            return True
        return False

    def handler_running_operate(self, sbus):
        # 切换到引导模式下进行解锁
        if not self.vehicle.attitude:
            return False
        if self.have_guide and not self.armed or not self.can_gps:
            return False

        if not self.armed:
            if self.vehicle.attitude.roll:
                self.roll = self.vehicle.attitude.roll
            if self.vehicle.attitude.yaw:
                self.yaw = self.vehicle.attitude.yaw
            if self.vehicle.attitude.pitch:
                self.pitch = self.vehicle.attitude.pitch
        if not self.armed and self.mode == 'GUIDED':
            sbus.channels[2] = 200
            self.set_armed = True
            self.armed = True
            return False
        if self.mode != "GUIDED" and self.mode != "RTL":
            self.mode = 'GUIDED'
            return False
        if not self.armed:
            return False
        self.command_running = True
        if self.takeoff_num < self.wait_num:
            sbus.channels[2] = 200
            self.a_target_time = int(time.time() * 1000)
            self.takeoff_num += 1
            return False
        # 起飞悬停1秒钟
        if int(time.time() * 1000) - self.a_target_time < 5000:
            self.command_running = False
            return False
        self.is_takeoff = True
        self.have_guide = True
        # 执行是否发现目标，如果发现目标，则直接实现打击
        if not self.in_track and super().handle_enter_operate(sbus):
            self.in_track = True
            self.have_track = False
            self.track_search = False
        if self.in_track:
            if super().handler_outer_operate(sbus):
                self.in_track = False
                return False
            super().handler_running_operate(sbus)
            self.vx, self.vy, self.vz, self.vd = super().get_track_velocity()
            # 判断是否进入过跟踪
            if super().is_tracking()[1]:
                self.have_track = True
                self.track_search = False
                return False
            if self.have_track and not self.track_search and super().is_tracking()[0]:
                self.track_search_time = int(time.time() * 1000)
                self.track_search = True
            if self.track_search and (int(time.time() * 1000) - self.track_search_time) < (self.search_time * 1000) / 2:
                return False
            if self.have_track and self.track_search:
                self.mode = "RTL"
                return False

        self.vx, self.vz = self.calculate_limited_velocity(vx=self.MAX_SPEED, pitch=self.pitch)
        self.vy = 0.0
        self.vd = 0.0
        # 未达到目的地，则持续飞行
        if not self.is_forward:
            if self.forward_num < self.wait_num:
                self.forward_num += 1
                return False
            self.is_forward = True
            self.a_forward_time = int(time.time() * 1000)
            return False

        if not self.is_arrived:
            # 持续判断当前无人机飞行距离是否达到预设距离了。
            t = int(time.time() * 1000) - self.a_forward_time
            # 计算各轴距离
            distance_x = self.vx * t
            distance_z = abs(self.vz) * t
            # 三维直线距离
            total_distance = math.sqrt(distance_x ** 2 + distance_z ** 2)
            if total_distance < self.target_distance * 1000:
                return False
            self.is_arrived = True
            return False
        # 悬停无人机
        self.vx, self.vy, self.vz, self.vd = self.get_search_vx_vy_vz_vd()
        if not self.is_search:
            if self.search_num < self.wait_num:
                self.search_num += 1
                return False
            if self.yaw is not None:
                self.yaw_temp = self.yaw
                self.yaw = None
            self.is_search = True
            self.a_search_time = int(time.time() * 1000)
            return False
        if int(time.time() * 1000) - self.a_search_time < self.search_time * 1000:
            return False
        self.yaw = self.yaw_temp
        self.command_running = False
        self.mode = "RTL"
        return False

    def handler_outer_operate(self, sbus):
        if (self.have_guide and not self.armed) or not ChannelUtils.is_guide_auto_operate(sbus.channels, None):
            self.init_flag()
            print('GuideLockTargetWithGpsOperate 退出引导')
            return True
        if not self.can_gps:
            self.init_flag()
            return True
        return False

    def handler_add_command(self, sbus):
        if not self.is_takeoff:
            self.add_command('get_takeoff_msg', self.a_targetAltitude)
            return
        self.add_command("set_velocity_body_yawrate", self.vx, self.vy, self.vz, self.vd)
        pass


class StabilizeLockTargetOperate(LockTargetNoGpsOperate):
    def __init__(self, track_space, servo_control, udp_socket, config_yaml):
        # 1. 先初始化父类参数
        super().__init__(track_space, servo_control, udp_socket, config_yaml)
        self.yam_pid_control = self.yam_pid_control = PID(Kp=self.config_yaml['track']['yawpid']['kp'],
                                                          Ki=self.config_yaml['track']['yawpid']['ki'],
                                                          Kd=self.config_yaml['track']['yawpid']['kd'], setpoint=0,
                                                          output_limits=(-300, 300),
                                                          sample_time=0.02)

        self.vx_pid_control = self.vx_pid_control = PID(Kp=self.config_yaml['speed']['xpid']['kp'],
                                                        Ki=self.config_yaml['speed']['xpid']['ki'],
                                                        Kd=self.config_yaml['speed']['xpid']['kd'], setpoint=0,
                                                        output_limits=(-500, 500),
                                                        sample_time=0.02)

        self.vy_pid_control = self.vy_pid_control = PID(Kp=self.config_yaml['speed']['ypid']['kp'],
                                                        Ki=self.config_yaml['speed']['ypid']['ki'],
                                                        Kd=self.config_yaml['speed']['ypid']['kd'], setpoint=0,
                                                        output_limits=(-500, 500),
                                                        sample_time=0.02)

        self.vz_pid_control = self.vz_pid_control = PID(Kp=self.config_yaml['speed']['zpid']['kp'],
                                                        Ki=self.config_yaml['speed']['zpid']['ki'],
                                                        Kd=self.config_yaml['speed']['zpid']['kd'], setpoint=0,
                                                        output_limits=(-500, 500),
                                                        sample_time=0.02)

        # 通过pid来控制无人机的前进方向
        # 通过反馈的速度来控制中线的位置
        # 通过像素点来控制左右跟踪精度

        # 2. 子类独有参数（覆盖父类同名参数）

        self.have_guide = False
        self.target_distance = 30
        self.search_time = 30
        self.wait_num = 10
        self.a_target_time = 0
        self.a_targetAltitude = 20
        self.is_takeoff = False
        self.takeoff_num = 0
        self.is_forward = False
        self.forward_num = 0
        self.a_forward_time = 0
        self.is_arrived = False
        self.is_search = False
        self.search_num = 0
        self.a_search_time = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.yaw_temp = 0
        self.MAX_SPEED = 3

        # 覆盖父类的速度参数（如果需要独立）
        self.vx = 0  # 覆盖父类的 self.vx = 0.1
        self.vy = 0  # 覆盖父类的 self.vy = 0.1
        self.vz = 0  # 保持与父类一致
        self.yaw_rate = 5  # 覆盖父类的 self.yaw_rate = 0.0
        self.in_track = False
        self.track_search = False
        self.have_track = False
        self.track_search_time = 0

    def handle_enter_operate(self, sbus, subs_before=None):
        # 此处记录下无人机的姿态信息，然后确定飞行的速度。
        if not self.have_guide and ChannelUtils.is_guide_auto_operate(sbus.channels,
                                                                      subs_before.channels) and not self.can_gps:
            return True
        return False

    def handler_running_operate(self, sbus):
        self.vx = 2
        self.vy = 1
        self.vz = -1
        self.yaw = 0.2

        return False

    def handler_outer_operate(self, sbus):
        return False

    def handler_add_command(self, sbus):
        pass
