import math

from pymavlink import mavutil


class ChannelUtils:
    @staticmethod
    def normalize_channels(channels):
        """Normalize SBUS channel values to a range of 1000-2000."""
        if channels:
            return [max(1000, min(2000, int((value - 200) / 1600 * 1000 + 1000))) for value in channels]
        return channels

    @staticmethod
    def constrain(value, midpoint, range_):
        """Constrain a value within a specified range around a midpoint."""
        return int((value - midpoint) / 500 * range_ + midpoint)

    @staticmethod
    def is_add_armed_operate(channels):
        return channels[10] > 1790 and channels[11] > 1790 or channels[7] > 1790

    @staticmethod
    def is_arme_Operate(channels_from_rc):
        # 无人机未解锁，要先解锁，两个摇杆要求向内下控制
        c1 = channels_from_rc[0]
        c2 = channels_from_rc[1]
        c3 = channels_from_rc[2]
        c4 = channels_from_rc[3]
        c6 = channels_from_rc[6]
        if c6 > 1790:
            return True

        if c1 < 250 and c2 > 1750 and c3 < 250 and c4 > 1750:
            return True
        return False

    @staticmethod
    def is_guide_show_operate(channels):
        return channels[10] > 1790

    @staticmethod
    def is_guide_auto_operate(channels, subs_before):
        if subs_before is None:
            return channels[5] > 1790
        return channels[5] > 1790 > subs_before[5]

    @staticmethod
    def is_lock_target_operate(channels):
        return abs(channels[9] - 1750) < 100

    @staticmethod
    def is_Attack_target_operate(channels):
        return abs(channels[8] - 1750) < 100

    @staticmethod
    def is_rtl_mode_operate(channels):
        return abs(channels[4] - 1800) < 20

    @staticmethod
    def is_autotune_mode_operate(channels):
        return abs(channels[4] - 1400) < 20

    @staticmethod
    def is_poshold_mode_operate(channels):
        return abs(channels[4] - 1000) < 20

    @staticmethod
    def is_fpv_mode_operate(channels):
        return abs(channels[4] - 1000) < 20

    @staticmethod
    def is_stabilize_mode_operate(channels):
        return abs(channels[4] - 200) < 20

    @staticmethod
    def is_zero_mode_operate(channels):
        return channels[4] - 200 < 20

    @staticmethod
    def is_land_operate(channels):
        return abs(channels[2] - 200) < 20

    @staticmethod
    def is_zero_speed_operate(channles):
        return abs(channles[0] - 1000) < 20 and abs(channles[1] - 1000) < 20


class MAVLinkCommands:
    @staticmethod
    def get_force_disarm(vehicle):
        """Sends a MAV_CMD_COMPONENT_ARM_DISARM command with parameter 21196 to forcibly disarm the vehicle."""
        msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation flag
            0,  # Disarm
            21196,  # Force disarm bypassing checks
            0, 0, 0, 0, 0
        )
        return msg

    @staticmethod
    def get_force_speed_local(vehicle, vx, vy, vz):
        """Sends y, z velocity in m/s n-e-d"""
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        return msg

    @staticmethod
    def set_global_yaw(vehicle, yaw):
        """
        设置全局坐标系下的绝对偏航角（不改变速度）
        :param yaw: 目标偏航角（弧度，0=正北）
        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,  # time_boot_ms, target_system, target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 全局坐标系
            0b1100000000001000,  # type_mask: 仅启用yaw（其他全禁用）
            0, 0, 0,  # x, y, z 位置（忽略）
            0, 0, 0,  # 速度（忽略）
            0, 0, 0,  # 加速度（忽略）
            yaw, 0  # yaw（启用）, yaw_rate（禁用）
        )
        return msg

    @staticmethod
    def set_velocity_and_yaw_local_ned(vehicle, vx_body, vy_body, vz_body):
        """
        单条指令控制速度 + 朝向（yaw_global 或 yaw_rate）
        :param yaw_global: 全局偏航角（弧度），None 时启用 yaw_rate
        :param yaw_rate: 偏航率（rad/s），仅当 yaw_global=None 时生效
        """
        # 机体速度 → 全局坐标系转换
        body_yaw = vehicle.attitude.yaw
        vx_global = vx_body * math.cos(body_yaw) - vy_body * math.sin(body_yaw)
        vy_global = vx_body * math.sin(body_yaw) + vy_body * math.cos(body_yaw)
        type_mask = 0b0000111111000110  # 启用速度 + yaw（禁用 yaw_rate）

        print('set_velocity_body', vx_body, vy_body, vz_body, body_yaw, vx_global, vy_global)

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,  # x, y, z（始终为0，飞控忽略）
            vx_global, vy_global, vz_body,  # 全局速度
            0, 0, 0,  # 加速度（忽略）
            0,  # yaw
            0  # yaw_rate
        )
        return msg

    @staticmethod
    def get_force_speed_body(vehicle, vx, vy, vz, yaw_rate=0):
        """
        Sends velocity commands in the body frame (forward/backward, left/right, up/down).
        :param vehicle: The drone vehicle object.
        :param vx: Forward/backward velocity in m/s (positive is forward).
        :param vy: Left/right velocity in m/s (positive is right).
        :param vz: Up/down velocity in m/s (positive is down).
        :return: The MAVLink message.
        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame (body-relative frame)
            0b0000111111000110,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s (relative to the drone's body frame)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, yaw_rate)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        return msg

    @staticmethod
    def get_takeoff_msg(vehicle, altitude):
        """
        Commands the drone to take off to a specified altitude.
        :param vehicle: The drone vehicle object.
        :param altitude: Target altitude in meters.
        """
        msg = vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,  # 目标系统
            vehicle._master.target_component,  # 目标组件
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0,  # No parameters for yaw, latitude, longitude
            0, 0, altitude  # Altitude
        )
        return msg

    @staticmethod
    def set_velocity_body_yawrate(vehicle, vx, vy, vz, vd_deg):
        """
        在机体坐标系下设置速度，同时保持固定朝向 yaw_lock_deg（度, 北为0，顺时针为正）。
        - vehicle: dronekit.Vehicle
        - vx: 机体前向速度 (m/s)，前为正
        - vy: 机体右向速度 (m/s)，右为正
        - vz: 垂直速度 (m/s)，向下为正
        - yaw_lock_deg: 要保持的航向（度）
        """
        # 获取当前偏航角
        yaw = vehicle.attitude.yaw
        # 使用锁定航向把机体速度转换为 NED（北/东/下）
        Vn = vx * math.cos(yaw) - vy * math.sin(yaw)
        Ve = vx * math.sin(yaw) + vy * math.cos(yaw)
        Vd = vz
        IGNORE_POS = (1 << 0) | (1 << 1) | (1 << 2)
        IGNORE_ACCEL = (1 << 6) | (1 << 7) | (1 << 8)
        IGNORE_YAW_RATE = (1 << 11)
        type_mask = IGNORE_POS | IGNORE_ACCEL | IGNORE_YAW_RATE

        vd_deg_rad = math.radians(vd_deg)

        # 构造并发送 MAVLink 消息（LOCAL_NED），带上 yaw 字段（yaw_lock_rad），把 yaw_rate 忽略
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (ignored)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,  # x, y, z positions (ignored)
            Vn, Ve, Vd,  # vx, vy, vz (m/s) in NED frame
            0, 0, 0,  # afx, afy, afz (ignored)
            yaw + vd_deg_rad,  # yaw (rad) -> lock heading here
            0.0  # yaw_rate (rad/s) ignored by mask
        )

        return msg
