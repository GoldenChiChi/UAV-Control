import time
import copy
from DroneClass import RemoteInfo
from auto_lock_target import GuideLockTargetWithGpsOperate, StabilizeLockTargetOperate
from lock_target import LockTargetWithGpsOperate, LockTargetNoGpsOperate
from joy_operate import JoystickForwardOperate, JoystickForwardWithGpsOperate
from operate import LoggerOperate, BreakRemoteOperate, AddArmedOperate, ArmedOperate, \
    IdleOperate, ModeChangeBehindOperate, ModeRTLOperate, NoGpsLandModeOperate

from control_context import ControlContext


class VehicleController:
    def __init__(self, sbus_space, track_space, servo_control, context_space, vehicle, udp_socket, config, stop_event):
        self.vehicle = vehicle
        self.sbus_space = sbus_space
        self.track_space = track_space
        self.servo_control = servo_control
        self.context_space = context_space
        self.config = config
        self.stop_event = stop_event

        self.context = ControlContext(self.vehicle)
        self.context.add_operate_handler(LoggerOperate(config['control']))
        self.context.add_operate_handler(BreakRemoteOperate())
        self.context.add_operate_handler(AddArmedOperate())
        self.context.add_operate_handler(ModeRTLOperate())
        self.context.add_operate_handler(ArmedOperate())
        self.context.add_operate_handler(IdleOperate())
        self.context.add_operate_handler(
            GuideLockTargetWithGpsOperate(self.vehicle, track_space, self.servo_control, udp_socket, config['control']))
        # self.context.add_operate_handler(StabilizeLockTargetOperate(track_space, servo_control,udp_socket, config['control']))
        self.context.add_operate_handler(LockTargetWithGpsOperate(self.track_space, self.servo_control, udp_socket,
                                                                  config['control']))
        # self.context.add_operate_handler(
        #     LockTargetNoGpsOperate(track_space, self.servo_control, udp_socket, config['control']))
        self.context.add_operate_handler(JoystickForwardWithGpsOperate(config['control']))
        # 解锁状态下，如果是自稳模式，则无人机自动降落，避免飞行中出现异常
        self.context.add_operate_handler(NoGpsLandModeOperate())
        self.context.add_operate_handler(ModeChangeBehindOperate())
        self.context.add_operate_handler(JoystickForwardOperate(config['control']))
        # 初始化发送停止跟踪指令
        from algo import getTrackMessage
        udp_socket.sendto(getTrackMessage(False),
                          (config['control']['algo']['they']['ip'], config['control']['algo']['they']['port']))
        self._setup_message_listeners()  # 初始化消息监听
        self.warning_text = 0
        self.warning_time = int(time.time() * 1000)

    def _setup_message_listeners(self):
        """注册 STATUSTEXT 消息的回调"""

        @self.vehicle.on_message('STATUSTEXT')
        def listener(vehicle, name, message):
            text = message.text.strip()
            self.warning_time = int(time.time() * 1000)  # 毫秒级时间戳

            # 状态码分段定义
            # 0 - 15: 无线电状态
            # 16 - 63: 解锁相关状态
            # 64 - 95: 遥控器状态
            # 96 - 127: 模式切换状态
            # 128 - 255: 系统状态
            # 状态码与中英文对照表（无符号字节数 0-255）

            status_info = {
                # 无线电状态 (0-15)
                1: ("Radio Failsafe Cleared", "无线电失控保护已解除"),

                # 解锁相关状态 (16-63)
                15: ("Arm: Need Position Estimate", "无有效的定位信息"),
                16: ("Arm: Compasses inconsistent", "解锁失败：指南针数据不一致"),
                17: ("Arm: Gyros not healthy", "解锁失败：陀螺仪异常"),
                18: ("Arm: Accels not healthy", "解锁失败：加速度计异常"),
                19: ("Arm: Preflight checks failed", "解锁失败：预检未通过"),
                20: ("PreArm: GPS 1: Bad fix", "预检失败：GPS定位质量差"),
                21: ("PreArm: No GPS lock", "预检失败：无GPS定位"),
                22: ("PreArm: Check battery", "预检失败：电池电压异常"),
                23: ("PreArm: Radio failsafe on", "安全开关已打开"),
                24: ("PreArm: Hardware safety switch", "硬件安全开关"),
                25: ("PreArm: Battery 1 low voltage failsafe", "电池电压过低"),
                # 遥控器状态 (64-95)
                64: ("Arm: Roll (RC1) is not neutral", "解锁失败：横滚通道(RC1)未回中"),
                65: ("Arm: Pitch (RC2) is not neutral", "解锁失败：俯仰通道(RC2)未回中"),
                66: ("Arm: Yaw (RC4) is not neutral", "解锁失败：偏航通道(RC4)未回中"),
                67: ("Arm: Throttle not down", "解锁失败：油门未置于最低位"),
                70: ("Disarming motors", "上锁"),
                71: ("Arming motors", "解锁"),
                72: ("CRITICAL:autopilot:Crash: Disarming", "上锁姿态异常，不满足上锁的条件"),
                80: ("SIM Hit ground at", "落地"),
                # 模式切换状态 (96-127)
                96: ("Command DENIED: Not in air", "模式切换失败：需在空中切换"),
                97: ("Command DENIED: Invalid transition", "模式切换失败：非法模式转换"),
                98: ("Command DENIED: Need 3D fix", "模式切换失败：需要GPS 3D定位"),
                99: ("Command DENIED: No terrain data", "模式切换失败：缺少地形数据"),
                100: ("Command DENIED: Geofence enabled", "模式切换失败：电子围栏激活"),
                101: ("Mode change timed out", "模式切换超时：传感器未响应"),
                102: ("Rejecting mode change", "模式切换拒绝：飞控主动拒绝"),
                103: ("Mode change failed", "模式切换失败：未知错误"),
                104: ("Mode change to GUIDED failed: requires position", "模式切换失败：进入GUIDED模式需要有效定位"),
                105: ("Mode change to AUTO failed: requires waypoints", "模式切换失败：进入AUTO模式需要预设航点"),
                106: ("Mode change to RTL failed: requires home position", "模式切换失败：进入返航模式需要设定返航点"),
                # 系统状态 (128-255)
                128: ("EKF variance", "导航异常：EKF方差过大"),
                129: ("Vibration high", "机体异常：振动过大"),
                130: ("Battery low", "电源警告：电池电量低"),
                140: ("EKF3 IMU1 is using GPS", "imu1"),
                141: ("EKF3 IMU0 is using GPS", "imu0"),
                142: ("EKF3 IMU1 origin set", "imu1"),
                143: ("EKF3 IMU0 origin set", "imu0"),
                144: ("GPS Glitch or Compass error", "gps出现数据跳跃性异常，磁罗盘数据出现错误"),
                145: ("Potential Thrust Loss", "推力异常")
            }

            # 默认状态
            self.warning_text = 0  # 正常状态
            status_msg = "系统正常运行"
            chinese_msg = "系统状态正常"

            # 匹配状态信息
            for code, (pattern, chinese) in status_info.items():
                if pattern in text:
                    self.warning_text = code & 0xFF  # 确保无符号字节
                    status_msg = text
                    chinese_msg = chinese
                    break

            # 未匹配的通用处理
            if self.warning_text == 0:
                if text.startswith("PreArm:"):
                    self.warning_text = 20  # 归入预检失败
                    status_msg = text
                    chinese_msg = f"预检异常：{text.split('PreArm:')[1].strip()}"
                elif text.startswith("Arm:"):
                    self.warning_text = 16  # 归入解锁失败
                    status_msg = text
                    chinese_msg = f"解锁异常：{text.split('Arm:')[1].strip()}"
                else:
                    self.warning_text = 255  # 未知状态
                    status_msg = text
                    chinese_msg = "未分类系统消息"
            if 96 <= self.warning_text <= 127:  # 模式切换类错误
                print(f"[模式切换][{self.warning_time}] {status_msg} | {chinese_msg}")
            else:
                print(f"[状态码:{self.warning_text:03d}][{self.warning_time}] {status_msg} | {chinese_msg}")

    def record_drone(self):
        vehicle = self.vehicle
        remote_info = RemoteInfo()
        if hasattr(self.track_space, 'track') and self.track_space.track.tracking_status:
            remote_info.track = 1
        if vehicle and vehicle.version and vehicle.version.major:
            remote_info.apmMode = ['ALT_HOLD', 'STABILIZE', 'LAND', 'POSHOLD', 'RTL', 'Loiter'.upper(),
                                   'AutoTune'.upper(), 'GUIDED'.upper(), 'BRAKE'].index(vehicle.mode.name)
            remote_info.apmConnect = 1
            remote_info.armable = vehicle.is_armable
            remote_info.armed = vehicle.armed
            if hasattr(self.context_space, 'armed_time') and remote_info.armed:
                remote_info.armedTime = int(time.time() * 1e3) - self.context.arme_time
            if vehicle.battery.voltage:
                remote_info.better.v = int(vehicle.battery.voltage * 1e2)
            if vehicle.battery.current:
                remote_info.better.a = int(vehicle.battery.current * 1e2)
            if vehicle.battery.level:
                remote_info.better.b = int(vehicle.battery.level * 1e2)

            remote_info.speed.x = int(vehicle.velocity[0] * 1e2)
            remote_info.speed.y = int(vehicle.velocity[1] * 1e2)
            remote_info.speed.z = int(vehicle.velocity[2] * 1e2)

            if vehicle.location and vehicle.location.global_frame:
                if vehicle.location.global_frame.lat:
                    remote_info.gps.lat = int(vehicle.location.global_frame.lat * 1e6)
                if vehicle.location.global_frame.lon:
                    remote_info.gps.lon = int(vehicle.location.global_frame.lon * 1e6)
                if vehicle.location.global_frame.alt:
                    remote_info.gps.alt = int(vehicle.location.global_frame.alt)

            if vehicle.gps_0:
                remote_info.gps.fix_type = vehicle.gps_0.fix_type
                remote_info.gps.eph = vehicle.gps_0.eph

            if vehicle.home_location:
                if vehicle.home_location.lat:
                    remote_info.home.lat = int(vehicle.home_location.lat * 1e6)
                if vehicle.home_location.lon:
                    remote_info.home.lon = int(vehicle.home_location.lon * 1e6)
                if vehicle.home_location.alt:
                    remote_info.home.alt = int(vehicle.home_location.alt)
            remote_info.attitude.heading = vehicle.heading
            if int(time.time() * 1000) - self.warning_time > 10000:
                self.warning_text = 0
            remote_info.warning = self.warning_text
        self.context_space.remoteInfo = remote_info

    def control_loop(self):
        while not self.stop_event.is_set():
            try:
                if self.vehicle and not self.vehicle.version or not self.vehicle.version.major:
                    print("Vehicle not ready.")
                    continue
                # 判断遥控器的连接状态
                # 获取原始值并创建深拷贝
                sbus = copy.deepcopy(getattr(self.sbus_space, 'sbus', None))
                sbus_before = copy.deepcopy(getattr(self.sbus_space, 'sbus_before', None))
                self.context.update_remote_status(sbus, sbus_before)
                # 同步无人机的状态和代码状态一致
                self.context.update_vehicle_state()
                self.context.execute()
                self.record_drone()
            except Exception as e:
                print("Error in control loop:", e)
            finally:
                time.sleep(0.05)
