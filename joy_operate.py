import math
import time

from util import ChannelUtils
from control_context import OperateHandler
from simple_pid import PID


class JoystickForwardOperate(OperateHandler):
    '''
    字段名	数据类型	单位	说明
    time_boot_ms	uint32_t	毫秒 (ms)	系统启动后的时间戳（从飞控启动开始计时）。
    lat	int32_t	度 × 10^7	纬度（Latitude），-353629037 / 1e7 ≈ -35.3629037°（南纬）。
    lon	int32_t	度 × 10^7	经度（Longitude），1491648254 / 1e7 ≈ 149.1648254°（东经）。
    alt	int32_t	毫米 (mm)	椭球高度（Altitude above ellipsoid），667150 / 1000 ≈ 667.15米（相对于 WGS84 椭球面）。
    relative_alt	int32_t	毫米 (mm) 相对高度（Altitude above home position），19520 / 1000 ≈ 19.52米（相对于起飞点）。
    vx	int16_t	厘米/秒 (cm/s)	X 轴速度（北向速度），233 cm/s ≈ 2.33 m/s（向北为正）。
    vy	int16_t	厘米/秒 (cm/s)	Y 轴速度（东向速度），-357 cm/s ≈ -3.57 m/s（向西为负）。
    vz	int16_t	厘米/秒 (cm/s)	Z 轴速度（地速，向下为正），1212 cm/s ≈ 12.12 m/s（向下为正）。
    hdg	uint16_t	厘度 (cdeg)	航向角（Heading），35303 / 100 ≈ 353.03°（0°=正北，顺时针增加）。
    '''

    def __init__(self, config_yaml):
        super().__init__()

        self.before_GLOBAL_POSITION = None
        self.before_sbus_channel = None

        # 初始化状态变量
        self.control_states = {
            'vx': {'prev_error': 0, 'integral': [], 'current_throttle': 1000, 'control_mode': "ACCELERATING", 'max': 0,
                   'accel': 1.5, 'dead': 15, 'kp': 0, 'ki': 0, 'kd': 0},
            'vy': {'prev_error': 0, 'integral': [], 'current_throttle': 1000, 'control_mode': "ACCELERATING", 'max': 0,
                   'accel': 1.5, 'dead': 15, 'kp': 0, 'ki': 0, 'kd': 0},
            'vz': {'prev_error': 0, 'integral': [], 'current_throttle': 1000, 'control_mode': "ACCELERATING", 'max': 0,
                   'accel': 1.0, 'dead': 10, 'kp': 0, 'ki': 0, 'kd': 0},
            'vd': {'prev_error': 0, 'integral': [], 'current_throttle': 1000, 'control_mode': "ACCELERATING", 'max': 0,
                   'accel': 0.5, 'dead': 5, 'kp': 0, 'ki': 0, 'kd': 0}
        }

        speed_config = config_yaml['speed']
        self.control_states['vx']['max'] = speed_config['max']['vx']
        self.control_states['vy']['max'] = speed_config['max']['vy']
        self.control_states['vz']['max'] = speed_config['max']['vz']
        self.control_states['vd']['max'] = speed_config['max']['vd']

        self.control_states['vx']['kp'] = speed_config['xpid']['kp']
        self.control_states['vx']['ki'] = speed_config['xpid']['ki']
        self.control_states['vx']['kd'] = speed_config['xpid']['kd']

        self.control_states['vy']['kp'] = speed_config['ypid']['kp']
        self.control_states['vy']['ki'] = speed_config['ypid']['ki']
        self.control_states['vy']['kd'] = speed_config['ypid']['kd']

        self.control_states['vz']['kp'] = speed_config['zpid']['kp']
        self.control_states['vz']['ki'] = speed_config['zpid']['ki']
        self.control_states['vz']['kd'] = speed_config['zpid']['kd']

        self.control_states['vd']['kp'] = speed_config['dpid']['kp']
        self.control_states['vd']['ki'] = speed_config['dpid']['ki']
        self.control_states['vd']['kd'] = speed_config['dpid']['kd']

        self.vd_pid_control = PID(Kp=speed_config['dpid']['kp'],
                                  Ki=speed_config['dpid']['ki'],
                                  Kd=speed_config['dpid']['kd'], setpoint=0,
                                  output_limits=(-300, 300),
                                  sample_time=0.1)

        self.ax = 0
        self.ay = 0
        self.az = 0
        self.vd = 0

    def handle_enter_operate(self, sbus, sub_before=None):
        # 穿越模式不理会
        if ChannelUtils.is_fpv_mode_operate(sbus.channels):
            return False
        return sbus and self.GLOBAL_POSITION

    def handler_running_operate(self, sbus):
        # print('JoystickForwardOperate')
        # 控制频率是每50毫秒执行一次
        if self.before_GLOBAL_POSITION and self.before_sbus_channel:
            time_cha = self.GLOBAL_POSITION.time_boot_ms - self.before_GLOBAL_POSITION.time_boot_ms
            if time_cha == 0:
                sbus.channels = self.before_sbus_channel
                return False
            # 计算出当前的速度值
            vx_body, vy_body, vz_body = self.global_to_body_velocity(self.GLOBAL_POSITION.vx,
                                                                     self.GLOBAL_POSITION.vy,
                                                                     self.GLOBAL_POSITION.vz,
                                                                     self.GLOBAL_POSITION.hdg / 100)

            remote_vy_body = self.map_channel_to_speed(sbus.channels[0], self.control_states['vy']['max'])
            sbus.channels[0] = self.update_axis_control_vy(remote_vy_body, vy_body)

            remote_vx_body = - self.map_channel_to_speed(sbus.channels[1], self.control_states['vx']['max'])
            sbus.channels[1] = self.update_axis_control_vx(remote_vx_body, vx_body)

            remote_vz_body = -self.map_channel_to_speed(sbus.channels[2], self.control_states['vz']['max'])
            sbus.channels[2] = self.update_axis_control_vz(remote_vz_body, vz_body)

            vd_body = round(math.degrees(self.attitude['vyaw']) % 360, 3)
            if vd_body > 180:
                vd_body = vd_body - 360
            remote_vd_body = self.map_channel_to_speed(sbus.channels[3], self.control_states['vd']['max'])
            sbus.channels[3] = self.update_axis_control_vd(remote_vd_body * 10, vd_body * 10)

        self.before_GLOBAL_POSITION = self.GLOBAL_POSITION
        self.before_sbus_channel = sbus.channels
        return False

    def clamp_channel_change(self, new_val, last_val, max_delta=200):
        """ 限制通道值变化幅度 """
        delta = new_val - last_val
        if abs(delta) > max_delta:
            return last_val + max_delta * (1 if delta > 0 else -1)
        return new_val

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        pass

    def update_axis_control_vx(self, target_speed, current_speed):
        axis = 'vx'
        state = self.control_states[axis]
        error = current_speed - target_speed
        derivative = error - state['prev_error']
        if abs(error) > 200:
            error = 200 * error / abs(error)
        output = 0
        # 状态转换逻辑
        if state['control_mode'] == "ACCELERATING":
            if abs(error) < 50:  # 接近目标速度，进入减速阶段
                state['control_mode'] = "DECELERATING"
                state['integral'] = []
        if state['control_mode'] == "DECELERATING":
            if abs(error) >= 50:  # 速度偏差过大，重新加速
                state['control_mode'] = "ACCELERATING"
                state['integral'] = []
        # 加速阶段
        if state['control_mode'] == "ACCELERATING":
            # 加速阶段使用全PID控制
            state['integral'].append(error)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)
        # 减速阶段
        if state['control_mode'] == "DECELERATING":
            # 减速阶段减弱积分项，增强微分项
            state['integral'].append(error * 0.4)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)

        if len(state['integral']) > 50:
            state['integral'].pop(0)
        state['current_throttle'] = 1000 - (1000 - state['current_throttle']) * 0.6 + int(output)
        # 限制摇杆量范围 (200-1800)
        state['current_throttle'] = int(max(200, min(1800, state['current_throttle'])))
        # 保存当前误差供下次使用
        state['prev_error'] = error

        return state['current_throttle']

    def update_axis_control_vy(self, target_speed, current_speed):
        axis = 'vy'
        state = self.control_states[axis]
        error = current_speed - target_speed
        derivative = error - state['prev_error']
        if abs(error) > 200:
            error = 200 * error / abs(error)
        output = 0
        # 状态转换逻辑
        if state['control_mode'] == "ACCELERATING":
            if abs(error) < 50:  # 接近目标速度，进入减速阶段
                state['control_mode'] = "DECELERATING"
                state['integral'] = []
        if state['control_mode'] == "DECELERATING":
            if abs(error) >= 50:  # 速度偏差过大，重新加速
                state['control_mode'] = "ACCELERATING"
                state['integral'] = []
        # 加速阶段
        if state['control_mode'] == "ACCELERATING":
            # 加速阶段使用全PID控制
            state['integral'].append(error)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)
        # 减速阶段
        if state['control_mode'] == "DECELERATING":
            # 减速阶段减弱积分项，增强微分项
            state['integral'].append(error * 0.4)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)

        if len(state['integral']) > 50:
            state['integral'].pop(0)
        state['current_throttle'] = 1000 - (1000 - state['current_throttle']) * 0.6 - int(output)
        # 限制摇杆量范围 (200-1800)
        state['current_throttle'] = int(max(200, min(1800, state['current_throttle'])))
        # 保存当前误差供下次使用
        state['prev_error'] = error

        return state['current_throttle']

    def update_axis_control_vd(self, target_speed, current_speed):
        # 旋转的速度单位是度每秒 量纲为 10
        axis = 'vd'
        state = self.control_states[axis]
        error = current_speed - target_speed
        if abs(error) > 200:
            error = 200 * error / abs(error)
        derivative = error - state['prev_error']
        output = 0
        # 状态转换逻辑
        if state['control_mode'] == "ACCELERATING":
            if abs(error) < 50:  # 接近目标速度，进入减速阶段
                state['control_mode'] = "DECELERATING"
                state['integral'] = []
        if state['control_mode'] == "DECELERATING":
            if abs(error) >= 50:  # 速度偏差过大，重新加速
                state['control_mode'] = "ACCELERATING"
                state['integral'] = []
        # 加速阶段
        if state['control_mode'] == "ACCELERATING":
            # 加速阶段使用全PID控制
            state['integral'].append(error)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)
        # 减速阶段
        if state['control_mode'] == "DECELERATING":
            # 减速阶段减弱积分项，增强微分项
            state['integral'].append(error * 0.5)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)

        # 只记录两秒钟以内的速度变化
        # state['integral'].append(error)
        if len(state['integral']) > 50:
            state['integral'].pop(0)
        state['current_throttle'] = 1000 - int(output) + target_speed / 1.5
        # 限制摇杆量范围 (200-1800)
        state['current_throttle'] = int(max(200, min(1800, state['current_throttle'])))
        # 保存当前误差供下次使用
        state['prev_error'] = error
        return state['current_throttle']

    def update_axis_control_vz(self, target_speed, current_speed):
        axis = 'vz'
        state = self.control_states[axis]
        # 根据控制模式计算输出
        error = int(current_speed - target_speed)
        if abs(error) > 100:
            error = 100 * error / abs(error)
        derivative = error - state['prev_error']
        output = 0
        # 状态转换逻辑
        if state['control_mode'] == "ACCELERATING":
            if abs(error) < 50:  # 接近目标速度，进入减速阶段
                state['control_mode'] = "DECELERATING"
                state['integral'] = []
        if state['control_mode'] == "DECELERATING":
            if abs(error) >= 50:  # 速度偏差过大，重新加速
                state['control_mode'] = "ACCELERATING"
                state['integral'] = []
        # 加速阶段
        if state['control_mode'] == "ACCELERATING":
            # 加速阶段使用全PID控制
            state['integral'].append(error)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)
        # 减速阶段
        if state['control_mode'] == "DECELERATING":
            # 减速阶段减弱积分项，增强微分项
            state['integral'].append(error * 0.4)
            output = (state['kp'] * error +
                      state['ki'] * sum(state['integral']) +
                      state['kd'] * derivative)
        # pid的p值控制到-400到400之间，则pi的和就成了最低将为100，最大为1800 2 0.075
        # 只记录两秒钟以内的速度数据，然后姑且每个数据最大差为100,则i的最大差为100*40，即8000 然后*0.05，i的控制范围为-400到400
        if len(state['integral']) > 50:
            state['integral'].pop(0)
        # 更新摇杆量 (1000是中立位置)
        output = int(-output if axis in ['vd'] else output)
        state['current_throttle'] = 1000 + int(output)

        # 限制摇杆量范围 (200-1800)
        state['current_throttle'] = int(max(200, min(1800, state['current_throttle'])))
        # 保存当前误差供下次使用
        state['prev_error'] = error
        return state['current_throttle']

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

    def map_channel_to_speed(self, channel_value, max_speed):
        """
        将通道值（1000~2000）线性映射到速度（-max_speed ~ +max_speed）
        :param channel_value: 通道值（如 1497）
        :param max_speed: 最大速度（如 self.MAX_VX）
        :return: 计算后的速度值
        """
        if abs(int((channel_value - 1000) * (max_speed / 800)) - 1000) < 100:
            return 0
        return int((channel_value - 1000) * (max_speed / 800))


class JoystickForwardWithGpsOperate(OperateHandler):
    '''
    在有gps的情况下，通过遥控给无人机上传预期达到的速度值，使用引导模式控制无人机的运行
    '''

    def __init__(self, config_yaml):
        super().__init__()
        self.before_GLOBAL_POSITION = None
        self.before_sbus_channel = None
        self.is_takeoff = False
        self.takeoff_time = time.time() * 1000

        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vd = 0

        # 初始化状态变量
        self.control_states = {'vx': {}, 'vy': {}, 'vz': {}, 'vd': {}}
        speed_config = config_yaml['speed']

        self.control_states['vx']['max'] = speed_config['max']['vx']
        self.control_states['vy']['max'] = speed_config['max']['vy']
        self.control_states['vz']['max'] = speed_config['max']['vz']
        self.control_states['vd']['max'] = speed_config['max']['vd']

    def handle_enter_operate(self, sbus, sub_before=None):
        # 穿越模式不理会
        if ChannelUtils.is_fpv_mode_operate(sbus.channels):
            return False
        self.command_running = False
        return sbus and self.can_gps

    def handler_running_operate(self, sbus):
        # 控制频率是每50毫秒执行一次
        if not self.before_GLOBAL_POSITION or not self.before_sbus_channel:
            self.before_GLOBAL_POSITION = self.GLOBAL_POSITION
            self.before_sbus_channel = sbus.channels
            return False

        time_cha = self.GLOBAL_POSITION.time_boot_ms - self.before_GLOBAL_POSITION.time_boot_ms
        if time_cha < 20:
            sbus.channels = self.before_sbus_channel
            return False
        self.before_GLOBAL_POSITION = self.GLOBAL_POSITION
        self.before_sbus_channel = sbus.channels
        if not self.armed:
            self.mode = 'STABILIZE'
            self.is_takeoff = False
            return False
        # 虽然gps可用，但是搜星状态不好，无人机无法进入到引导模式，出现进入穿越模式的风险
        if self.mode != "GUIDED".upper():
            sbus.channels[2] = 200
            self.mode = 'GUIDED'
            return False
        if not self.is_takeoff:
            self.is_takeoff = True
            self.takeoff_time = time.time() * 1000
            return False
        self.command_running = True
        self.vy = self.map_channel_to_speed(sbus.channels[0], self.control_states['vy']['max']) / 100
        self.vx = -self.map_channel_to_speed(sbus.channels[1], self.control_states['vx']['max']) / 100
        self.vz = -self.map_channel_to_speed(sbus.channels[2], self.control_states['vz']['max']) / 100
        self.vd = self.map_channel_to_speed(sbus.channels[3], self.control_states['vd']['max'])
        # 设置转动摇杆，避免转动速度过快
        sbus.channels[3] = 1000
        return False

    def handler_outer_operate(self, sbus):
        return True

    def handler_add_command(self, sbus):
        if (time.time() * 1000 - self.takeoff_time) < 2000:
            self.add_command('get_takeoff_msg', 1)
            return
        self.add_command('set_velocity_body_yawrate', self.vx, self.vy, self.vz, self.vd)
        pass

    def map_channel_to_speed(self, channel_value, max_speed):
        """
        将通道值（1000~2000）线性映射到速度（-max_speed ~ +max_speed）
        :param channel_value: 通道值（如 1497）
        :param max_speed: 最大速度（如 self.MAX_VX）
        :return: 计算后的速度值
        """
        if abs(int((channel_value - 1000) * (max_speed / 800)) - 1000) < 100:
            return 0
        return int((channel_value - 1000) * (max_speed / 800))
