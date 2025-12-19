import time


class SerialSender():
    def __init__(self, sbus_serial, track_space, context_space, stop_event):
        self.sbus_serial = sbus_serial
        self.track_space = track_space
        self.context_space = context_space
        self.stop_event = stop_event

    def get_RemoteInfo(self):
        return getattr(self.context_space, 'remoteInfo', None)

    def remoteToByteArray(self, remoteInfo):
        json_bytes = bytearray()
        if not remoteInfo:
            return json_bytes
        json_bytes.append(remoteInfo.apmMode & 0xff)
        json_bytes.append(remoteInfo.armedTime >> 24 & 0xff)
        json_bytes.append(remoteInfo.armedTime >> 16 & 0xff)
        json_bytes.append(remoteInfo.armedTime >> 8 & 0xff)
        json_bytes.append(remoteInfo.armedTime & 0xff)
        json_bytes.append(remoteInfo.armed & 0xff)
        json_bytes.append(remoteInfo.armable & 0xff)

        json_bytes.append(remoteInfo.better.v >> 8 & 0xff)
        json_bytes.append(remoteInfo.better.v & 0xff)
        json_bytes.append(remoteInfo.better.a >> 8 & 0xff)
        json_bytes.append(remoteInfo.better.a & 0xff)
        json_bytes.append(remoteInfo.better.b >> 8 & 0xff)
        json_bytes.append(remoteInfo.better.b & 0xff)

        json_bytes.append(remoteInfo.speed.x >> 8 & 0xff)
        json_bytes.append(remoteInfo.speed.x & 0xff)
        json_bytes.append(remoteInfo.speed.y >> 8 & 0xff)
        json_bytes.append(remoteInfo.speed.y & 0xff)
        json_bytes.append(remoteInfo.speed.z >> 8 & 0xff)
        json_bytes.append(remoteInfo.speed.z & 0xff)

        json_bytes.append(remoteInfo.gps.lat >> 24 & 0xff)
        json_bytes.append(remoteInfo.gps.lat >> 16 & 0xff)
        json_bytes.append(remoteInfo.gps.lat >> 8 & 0xff)
        json_bytes.append(remoteInfo.gps.lat & 0xff)

        json_bytes.append(remoteInfo.gps.lon >> 24 & 0xff)
        json_bytes.append(remoteInfo.gps.lon >> 16 & 0xff)
        json_bytes.append(remoteInfo.gps.lon >> 8 & 0xff)
        json_bytes.append(remoteInfo.gps.lon & 0xff)

        json_bytes.append(remoteInfo.gps.alt >> 8 & 0xff)
        json_bytes.append(remoteInfo.gps.alt & 0xff)

        json_bytes.append(remoteInfo.home.lat >> 24 & 0xff)
        json_bytes.append(remoteInfo.home.lat >> 16 & 0xff)
        json_bytes.append(remoteInfo.home.lat >> 8 & 0xff)
        json_bytes.append(remoteInfo.home.lat & 0xff)

        json_bytes.append(remoteInfo.home.lon >> 24 & 0xff)
        json_bytes.append(remoteInfo.home.lon >> 16 & 0xff)
        json_bytes.append(remoteInfo.home.lon >> 8 & 0xff)
        json_bytes.append(remoteInfo.home.lon & 0xff)

        json_bytes.append(remoteInfo.home.alt >> 8 & 0xff)
        json_bytes.append(remoteInfo.home.alt & 0xff)

        json_bytes.append(remoteInfo.apmConnect & 0xff)

        json_bytes.append(remoteInfo.gps.fix_type & 0xff)
        json_bytes.append(remoteInfo.gps.eph >> 8 & 0xff)
        json_bytes.append(remoteInfo.gps.eph & 0xff)

        json_bytes.append(remoteInfo.attitude.heading >> 8 & 0xFF)  # 长度的高字节
        json_bytes.append(remoteInfo.attitude.heading & 0xFF)  # 长度的低字节

        json_bytes.append(remoteInfo.track & 0xff)

        json_bytes.append(remoteInfo.warning & 0xff)

        return json_bytes

    def __call__(self):
        '''
        获取GPS信息
        0：没有 GPS 信号
        1：2D 锁定（不适用于 Loiter 模式）
        2：2D 锁定，但精度较差（不适用于 Loiter 模式）
        3：3D 锁定，适合 Loiter 模式
        satellites_visible 表示当前可见的卫星数量，通常需要 6 颗以上卫星才能保证 GPS 精度。
        通过读取 vehicle.gps_0.eph 值来获取 HDOP 值，通常 HDOP 小于 2 被认为是较好。
        '''
        while not self.stop_event.is_set():
            try:
                remote_info = self.get_RemoteInfo()
                remote_bytes = self.remoteToByteArray(remote_info)
                remote_length = len(remote_bytes)
                data_packet = bytearray()
                data_packet.append(0x0F)  # 0x0F 开头
                # 添加长度字段（两个字节，高字节在前，低字节在后）
                data_packet.append((remote_length >> 8) & 0xFF)  # 长度的高字节
                data_packet.append(remote_length & 0xFF)  # 长度的低字节
                # 添加 JSON 字节数据
                data_packet.extend(remote_bytes)
                # 计算校验和（仅对 JSON 数据部分计算）
                checksum = sum(remote_bytes) & 0xFF
                # 添加校验和
                data_packet.append(checksum)
                self.sbus_serial.write(data_packet)
                # print(int(time.time() * 1000), 'SerialSender-->>', ' '.join(f'{b:02x}' for b in data_packet))

            except Exception as e:
                print('SerialSender __call__:', e)
            finally:
                time.sleep(0.5)
