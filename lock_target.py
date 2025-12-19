import math
import time

from algo import getTrackMessage
from control_context import OperateHandler
from util import ChannelUtils
from simple_pid import PID


class AttackTargetOperate:

    def isAttack(self, sbus):
        return ChannelUtils.is_Attack_target_operate(sbus.channels)

    def AttackOperate(self, sbus):
        sbus.channels[1] = 200


class LockTargetWithGpsOperate(OperateHandler, AttackTargetOperate):
    def __init__(self, track_space, servo_control, udp_socket, config_yaml):
        super().__init__()
        self.track_space = track_space
        self.servo_control = servo_control
        self.udp_socket = udp_socket
        self.config_yaml = config_yaml
        self.x_pid_control = None
        self.y_pid_control = None
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vd = 0
        self.in_search = False
        self.send_algo_time = time.time() * 1000

    def init_flag(self):
        self.command_running = False
        self.x_pid_control = None
        self.y_pid_control = None
        self.in_search = False
        if self.servo_control:
            self.servo_control.track = False
            self.servo_control.pwm_servo.back_mid()

    def handle_enter_operate(self, sbus, sub_before=None):
        if self.armed and ChannelUtils.is_lock_target_operate(sbus.channels) and self.can_gps:
            self.udp_socket.sendto(getTrackMessage(True), (self.config_yaml['algo']['they']['ip'],
                                                           self.config_yaml['algo']['they']['port']))
            self.x_pid_control = None
            self.y_pid_control = None
            self.in_search = False
            return True
        return False

    def handler_running_operate(self, sbus):
        self.command_running = True
        self.mode = "GUIDED"
        if self.is_tracking()[1]:
            if self.servo_control:
                self.servo_control.track = True
            self.vx, self.vy, self.vz, self.vd = self.get_track_vx_vy_vz_vd(sbus)
        else:
            # 悬停无人机
            self.vx, self.vy, self.vz, self.vd = self.get_search_vx_vy_vz_vd()
            self.in_search = True
            if self.servo_control:
                self.servo_control.track = False
                # self.servo_control.pwm_servo.back_mid()
        return False

    def handler_outer_operate(self, sbus):
        if not self.armed or not ChannelUtils.is_lock_target_operate(sbus.channels):
            self.udp_socket.sendto(getTrackMessage(False), (self.config_yaml['algo']['they']['ip'],
                                                            self.config_yaml['algo']['they']['port']))
            self.init_flag()
            return True
        if not self.is_tracking()[0] and self.in_search:
            # 在搜索状态中不退出跟踪
            if time.time() * 1000 - self.send_algo_time > 200:
                self.send_algo_time = time.time() * 1000
                print('search')
                self.udp_socket.sendto(getTrackMessage(True), (self.config_yaml['algo']['they']['ip'],
                                                               self.config_yaml['algo']['they']['port']))
        self.x_pid_control = None
        self.y_pid_control = None
        self.in_search = False
        return False

    def handler_add_command(self, sbus):
        self.add_command('set_velocity_body_yawrate', self.vx, self.vy, self.vz, self.vd)
        pass

    def is_tracking(self):
        target_box_class = getattr(self.track_space, 'track', None)
        waiting = target_box_class and time.time() * 1000 - target_box_class.timestamp < 1000
        tracking = waiting and target_box_class.tracking_status
        return waiting, tracking

    def get_track_vx_vy_vz_vd(self, sbus):
        target_box_class = self.track_space.track
        if not self.x_pid_control:
            self.x_pid_control = PID(Kp=self.config_yaml['track']['xpid']['kp'],
                                     Ki=self.config_yaml['track']['xpid']['ki'],
                                     Kd=self.config_yaml['track']['xpid']['kd'], setpoint=0,
                                     output_limits=(-500, 500),
                                     sample_time=0.02)

        if not self.y_pid_control:
            self.y_pid_control = PID(Kp=self.config_yaml['track']['ypid']['kp'],
                                     Ki=self.config_yaml['track']['ypid']['ki'],
                                     Kd=self.config_yaml['track']['ypid']['kd'], setpoint=0,
                                     output_limits=(-200, 200),
                                     sample_time=0.02)
        # -32.00022750798962 20.400031234936254 720 640 309 360 0 0 0.20400031234936256 -0.9600068252396886
        # 跟踪舵机可用的情况下，努力让跟踪舵机的朝向角为90度
        if self.servo_control:
            servo_point = self.servo_control.get_point()
            y_control_signal = self.y_pid_control(servo_point - 90)
            vz = 0.05 * y_control_signal
        else:
            # 跟踪舵机不可用，使用固定角度使得目标落在y轴像素中心 640
            y_control_signal = self.y_pid_control(target_box_class.center_y - target_box_class.target_center_y)
            vz = 0.01 * y_control_signal

        # 控制无人机转动，使得目标拉到x方向的图像中心 1280
        x_control_signal = self.x_pid_control(target_box_class.center_x - target_box_class.target_center_x)
        vd = 0.02 * x_control_signal

        vx = 0
        vy = 0
        if self.isAttack(sbus):
            vx = 2

        print('使用引导模式下的上升下降速度来控制无人机高度', x_control_signal, y_control_signal,
              target_box_class.target_center_x, target_box_class.center_x, target_box_class.target_center_y,
              target_box_class.center_y, vx, vy, vz, vd)

        return vx, vy, vz, vd

    def get_search_vx_vy_vz_vd(self):
        return 0.2, 0.2, 0, 4

    def get_track_velocity(self):
        return self.vx, self.vy, self.vz, self.vd


class LockTargetNoGpsOperate(OperateHandler, AttackTargetOperate):
    def __init__(self, track_space, servo_control, udp_socket, config_yaml):
        super().__init__()
        self.track_space = track_space
        self.servo_control = servo_control
        self.udp_socket = udp_socket
        self.config_yaml = config_yaml

        # 全局的状态值
        self.before_GLOBAL_POSITION = None
        self.before_sbus_channel = None

        self.x_pid_control = None
        self.y_pid_control = None
        self.vx = 0.1
        self.vy = 0.1
        self.vz = 0
        self.yaw_rate = 2 * math.pi / 36
        self.in_search = False

    def handle_enter_operate(self, sbus, sub_before=None):
        if self.armed and ChannelUtils.is_lock_target_operate(sbus.channels):
            self.udp_socket.sendto(getTrackMessage(True), (self.config_yaml['algo']['they']['ip'],
                                                           self.config_yaml['algo']['they']['port']))
            self.x_pid_control = None
            self.y_pid_control = None
            self.in_search = False
            return True
        return False

    def handler_running_operate(self, sbus):
        self.mode = "STABILIZE"
        if not self.before_GLOBAL_POSITION or not self.before_sbus_channel:
            self.before_GLOBAL_POSITION = self.GLOBAL_POSITION
            self.before_sbus_channel = sbus.channels
            return False
        time_cha = self.GLOBAL_POSITION.time_boot_ms - self.before_GLOBAL_POSITION.time_boot_ms
        if time_cha < 20:
            return False
        # 相机在跟踪状态下
        if self.in_waiting_tracking()[1]:
            self.in_search = False
            # 启动舵机跟踪
            self.servo_control.track = True
            # 启动无人机的跟踪
            vx, vy, vz, vd = self.get_track_velocity(sbus)
            target_box_class = self.track_space.track
            print('使用自稳模式下的跟踪过程控制速度值', target_box_class.target_center_x, target_box_class.center_x,
                  target_box_class.target_center_y, target_box_class.center_y, vx, vy, vz, vd)

            channels_1 = self.get_channle1_by_vy(sbus, vy)
            channels_2 = self.get_channle2_by_vx(sbus, vx)
            channels_3 = self.get_channle3_by_vz(sbus, vz)
            channels_4 = self.get_channle4_by_vd(sbus, vd)
            sbus.channels[0] = channels_1
            sbus.channels[1] = channels_2
            sbus.channels[2] = channels_3
            sbus.channels[3] = channels_4
        # 相机在等待跟踪的状态下（初始跟踪选择目标期间和丢失跟踪目标后1秒内）
        elif self.in_waiting_tracking()[0]:
            # 悬停无人机
            channels_1, channels_2, channels_3, channels_4 = self.get_search_velocity(sbus)
            self.in_search = True
            # 设置舵机不跟踪，并且回中位置
            self.servo_control.track = False
            self.servo_control.pwm_servo.back_mid()
            sbus.channels[0] = channels_1
            sbus.channels[1] = channels_2
            sbus.channels[2] = channels_3
            sbus.channels[3] = channels_4
        return False

    def handler_outer_operate(self, sbus):
        # 退出跟踪的逻辑
        if not self.armed or not ChannelUtils.is_lock_target_operate(sbus.channels):
            self.udp_socket.sendto(getTrackMessage(False), (self.config_yaml['algo']['they']['ip'],
                                                            self.config_yaml['algo']['they']['port']))
            return True
        # 跟踪目标丢失了，重新下发启动跟踪的指令
        if not self.in_waiting_tracking()[0] and self.in_search:
            # 在搜索状态中不退出跟踪
            self.udp_socket.sendto(getTrackMessage(True), (self.config_yaml['algo']['they']['ip'],
                                                           self.config_yaml['algo']['they']['port']))
        self.x_pid_control = None
        self.y_pid_control = None
        self.in_search = False
        return False

    def in_waiting_tracking(self):
        target_box_class = getattr(self.track_space, 'track', None)
        waiting = target_box_class and time.time() * 1000 - target_box_class.timestamp < 1000
        tracking = waiting and target_box_class.tracking_status
        return waiting, tracking

    def get_track_velocity(self, sbus):
        '''
        获取在跟踪状态下，希望飞机到达的速度值-需要获取当前的速度值，也要获取跟踪的状态值。
        '''
        target_box_class = self.track_space.track
        # 计算出当前的速度值
        vx_body, vy_body, vz_body = self.global_to_body_velocity(self.GLOBAL_POSITION.vx,
                                                                 self.GLOBAL_POSITION.vy,
                                                                 self.GLOBAL_POSITION.vz,
                                                                 self.GLOBAL_POSITION.hdg / 100)

        vd_body = round(math.degrees(self.attitude['vyaw']) % 360, 3)
        if vd_body > 180:
            vd_body = vd_body - 360
        # 最大值 4096
        cha_x = math.pow((target_box_class.target_center_x - target_box_class.center_x) / 40, 3) / 200
        # 最大值 729
        cha_y = math.pow((target_box_class.target_center_y - target_box_class.center_y) / 40, 3) / 7
        # 前后移动速度
        vx_body = 0
        if self.isAttack(sbus):
            vx_body = self.config_yaml['speed']['max']['vx']
        # 水平移动速度
        vy_body = 0
        # 地速，向下为正
        vz_body = vz_body + cha_y
        vd_body = vd_body + cha_x

        vz_body = max(min(vz_body, self.config_yaml['speed']['max']['vz']), -self.config_yaml['speed']['max']['vz'])
        vd_body = max(min(vd_body, self.config_yaml['speed']['max']['vd']), -self.config_yaml['speed']['max']['vd'])

        return vx_body, vy_body, vz_body, vd_body

    def get_search_velocity(self, sbus):
        '''
        获取在搜索目标状态下，希望飞机到达的速度值
        '''

        return 0.1, 0.1, 0, 10

    def get_channle1_by_vy(self, sbus, vy):
        '''
        控制无人机左右横移的参数
        '''
        return 1000

    def get_channle2_by_vx(self, sbus, vx):
        '''
        控制无人机前进后退的参数
        '''
        if self.isAttack(sbus):
            return 500
        return 1000

    def get_channle3_by_vz(self, sbus, vz):
        '''
        控制无人机上升和下降的参数
        '''
        return 1000

    def get_channle4_by_vd(self, sbus, vd):
        '''
        控制无人机左右转动方向的参数
        '''
        return 1000

    def global_to_body_velocity(self, vn, ve, vd, yaw_deg):
        """
        将全局坐标系（NED）速度转换到机体坐标系（FRD）
        :param vn: 北向速度（m/s）
        :param ve: 东向速度（m/s）
        :param vd: 地速（m/s，向下为正）
        :param yaw_deg: 当前偏航角（0°~360°，正北为0°，顺时针增加）
        :return: (vx_body, vy_body, vz_body) 单位：m/s
        """
        # 将偏航角从度转换为弧度，并标准化到[-π, π]范围
        yaw_rad = math.radians(yaw_deg)

        # 计算机体坐标系速度
        vx_body = vn * math.cos(yaw_rad) + ve * math.sin(yaw_rad)
        vy_body = -vn * math.sin(yaw_rad) + ve * math.cos(yaw_rad)
        vz_body = vd
        return int(vx_body), int(vy_body), int(vz_body)
