import time
from control_context import OperateHandler


class RouteOperate(OperateHandler):

    def __init__(self, config_yaml):
        '''
        用于统计无人机的飞行轨迹，以便于后续返航的时候能够自动按照原路径返航。

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
