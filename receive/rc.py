# -*- coding:utf-8 -*-
import time
import math
import pygame
import serial
import struct
import logging
import threading
import socket
import json

K = 4.5

def smooth(x, k):
    """ k值参考表(k值越大,摇杆响应越慢)
        k:  0.5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.951 -0.905 -0.861 -0.819 -0.779 -0.741 -0.705 -0.670  0.000  0.000  0.000  0.670  0.705  0.741  0.779  0.819  0.861  0.905  0.951


        k:  1
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.905 -0.819 -0.741 -0.670 -0.607 -0.549 -0.497 -0.449  0.000  0.000  0.000  0.449  0.497  0.549  0.607  0.670  0.741  0.819  0.905


        k:  1.5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.861 -0.741 -0.638 -0.549 -0.472 -0.407 -0.350 -0.301  0.000  0.000  0.000  0.301  0.350  0.407  0.472  0.549  0.638  0.741  0.861


        k:  2
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.819 -0.670 -0.549 -0.449 -0.368 -0.301 -0.247 -0.202  0.000  0.000  0.000  0.202  0.247  0.301  0.368  0.449  0.549  0.670  0.819


        k:  2.5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.779 -0.607 -0.472 -0.368 -0.287 -0.223 -0.174 -0.135  0.000  0.000  0.000  0.135  0.174  0.223  0.287  0.368  0.472  0.607  0.779


        k:  3
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.741 -0.549 -0.407 -0.301 -0.223 -0.165 -0.122 -0.091  0.000  0.000  0.000  0.091  0.122  0.165  0.223  0.301  0.407  0.549  0.741


        k:  3.5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.705 -0.497 -0.350 -0.247 -0.174 -0.122 -0.086 -0.061  0.000  0.000  0.000  0.061  0.086  0.122  0.174  0.247  0.350  0.497  0.705


        k:  4
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.670 -0.449 -0.301 -0.202 -0.135 -0.091 -0.061 -0.041  0.000  0.000  0.000  0.041  0.061  0.091  0.135  0.202  0.301  0.449  0.670


        k:  4.5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.638 -0.407 -0.259 -0.165 -0.105 -0.067 -0.043 -0.027  0.000  0.000  0.000  0.027  0.043  0.067  0.105  0.165  0.259  0.407  0.638


        k:  5
        x: -1.000 -0.900 -0.800 -0.700 -0.600 -0.500 -0.400 -0.300 -0.200 -0.100  0.000  0.100  0.200  0.300  0.400  0.500  0.600  0.700  0.800  0.900
        y: -1.000 -0.607 -0.368 -0.223 -0.135 -0.082 -0.050 -0.030 -0.018  0.000  0.000  0.000  0.018  0.030  0.050  0.082  0.135  0.223  0.368  0.607
    """
    abs_x = abs(x)

    if abs_x <= 0.1:
        return 0
    
    return x / abs_x * math.exp(k * abs_x - k)


def mapper(x, xmin=200, xmax=1800, flip=False):
    """ 线性变换,将XBOX数据0-1/-1-1转换到200-1800区间
        flip,摇杆默认上负下正,左负右正,是否需要对正负关系进行反转
    """
    xsum = xmax + xmin
    xrange = xmax - xmin
    x = int(x / (1 - -1) * xrange + xsum / 2)

    if flip:
        return xsum - x
    
    return x


def setSbusChannels(keys):
    """ XBOX按键映射到SBUS协议 200-1800
        channels[0]:  右摇杆(左右)      Roll
        channels[1]:  右摇杆(上下)      Pitch
        channels[2]:  左摇杆(上下)      Throttle
        channels[3]:  左摇杆(左右)      Row
        channels[4]:  B键/X键/Y键/三键  STABILIZE/RTL/POSHOLD/AUTOTUNE
        channels[5]:  未定义           Undefined
        channels[6]:  A键              armed 
        channels[7]:  未定义           Undefined
        channels[8]:  左上键           跟踪开启
        channels[9]:  右上键           发射开启
        channels[10]: 左上摇杆         测试阶段上锁1
        channels[11]: 右上摇杆         测试阶段上锁2
        channels[12]: 未定义           Undefined
        channels[13]: 未定义           Undefined
        channels[14]: 未定义           Undefined
        channels[15]: 未定义           Undefined
    """
    sbus_undefined = 200
    sbus_button_off = 200
    sbus_button_on = 1800
    sbus_axis_value_min = 200
    sbus_axis_value_max = 1800
    
    channels = [0] * 16
    channels[0] = mapper(keys[3], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=False)
    channels[1] = mapper(keys[4], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=False)
    channels[2] = mapper(keys[1], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=True)
    channels[3] = mapper(keys[0], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=False)
    
    mode_num = 0
    if keys[7]:
        mode_num = sbus_axis_value_min
    if keys[8]:
        mode_num = sbus_axis_value_max
    if keys[9]:
        mode_num = (sbus_axis_value_min + sbus_axis_value_max) // 2
    if keys[13]:
        mode_num = 1400
    channels[4] = mode_num

    channels[5] = sbus_undefined
    channels[6] = sbus_button_on if keys[6] else sbus_button_off
    channels[7] = sbus_undefined
    channels[8] = sbus_button_on if keys[10] else sbus_button_off
    channels[9] = sbus_button_on if keys[11] else sbus_button_off
    channels[10] = mapper(keys[2], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=False)
    channels[11] = mapper(keys[5], xmin=sbus_axis_value_min, xmax=sbus_axis_value_max, flip=False)
    channels[12] = sbus_undefined
    channels[13] = sbus_undefined
    channels[14] = sbus_undefined
    channels[15] = sbus_undefined
    
    return channels


def getXboxKeys():
    """ xbox对应按键(注意 win上按键映射和linux下可能有所不同)
        keys[0]:  左摇杆(左右) -1 - 1
        keys[1]:  左摇杆(上下) -1 - 1
        keys[2]:  左上摇杆     -1 - 1
        keys[3]:  右摇杆(左右) -1 - 1
        keys[4]:  右摇杆(上下) -1 - 1
        keys[5]:  右上摇杆     -1 - 1
        keys[6]:  A键          0 - 1
        keys[7]:  B键          0 - 1
        keys[8]:  X键          0 - 1
        keys[9]:  Y键          0 - 1
        keys[10]: 左上键        0 - 1
        keys[11]: 右上键        0 - 1
        keys[12]: 复制键        0 - 1
        keys[13]: 目录键        0 - 1
        keys[14]: X灯键         0 - 1
        keys[15]: 未定义        未定义
        keys[16]: 未定义        未定义
        keys[17]: 退出键        0 - 1
    """
    import sys
    pygame.init()

    running = True
    joystick = None
    while running:
        try:
            # 检查当前是否有手柄连接
            if pygame.joystick.get_count() == 0:
                print("未检测到手柄，等待连接...")
                time.sleep(1)
                pygame.joystick.quit()
                pygame.joystick.init()
                continue
            # 如果手柄对象不存在或已断开，重新获取
            if joystick is None or not joystick.get_init():
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                print("手柄已连接！")

            keys = list()

            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # 获取控制器的轴输入
            num_axes = joystick.get_numaxes()
            for axis_id in range(num_axes):
                keys.append(smooth(joystick.get_axis(axis_id), K))

            # 获取控制器的按钮输入
            num_buttons = joystick.get_numbuttons()
            for button_id in range(num_buttons):
                keys.append(joystick.get_button(button_id))

            # 填充不足的元素为0，确保共有16个元素
            while len(keys) < 16:
                keys.append(0)

            channels = setSbusChannels(keys)

            # 转换为35字节的字节数组
            byte_array = struct.pack('>' + 'h' * 16, *channels)
            byte_array = bytes([0xf]) + byte_array + bytes([0, 0])
            print(byte_array)
            # 发送字节数组到串口
            ser.write(byte_array)

            time.sleep(0.1)
        except pygame.error as e:
            print(f"手柄异常：{e}，等待重连...")
            joystick = None
            time.sleep(1)
            pygame.joystick.quit()
            pygame.joystick.init()
            continue
        except Exception as e:
            print(f"未知错误：{e}")
            time.sleep(1)
            continue

    # 关闭串口
    ser.close()
    # 退出PyGame
    pygame.quit()


logging.basicConfig(filename='com4_log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')


def dataRecv():
    #udp
    udp_soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
    local_address = ('0.0.0.0', 9966)
    udp_soc.bind(local_address)

    while True:
        data_length = 0
        data_buffer = []
        data_temp = []

        while True:
            byte = ser.read(1)
            print(byte)
            
            if byte[0] == 0x0f:
                data_temp = ser.read(2)
                if len(data_temp) == 2:
                    break

        data_buffer = [0x0f] + list(data_temp)
        data_length = (data_temp[0] << 8) + data_temp[1]
        if data_length != 47:
            erro_data = {}
            erro_data["flight controller erro"] = 1
            erro_send_data = json.dumps(erro_data)
            udp_soc.sendto(erro_send_data.encode("utf-8"), ('127.0.0.1', 12345))
            continue

        data_buffer += list(ser.read(data_length + 1))
      

        if (sum(data_buffer[3:-1]) & 0xFF) != data_buffer[-1]:
            print(">>> check sum error")
            continue

        valid_data = data_buffer[3:-1]
        mode_list = ['ALT_HOLD', 'STABILIZE', 'LAND', 'POSHOLD', 'RTL', 'Loiter', "AutoTUNE", "GUIDED", "BREAK", ...]
        # decoded_data = bytes(valid_data).decode('utf-8')
        feixingshijian = (int(valid_data[1] << 24) + int(valid_data[2] << 16) + int(valid_data[3] << 8) + int(
            valid_data[4]))
        armed_state = valid_data[5]
        is_armable = valid_data[6]
        dianya = int(valid_data[7] << 8) + int(valid_data[8])
        dianliu = int(valid_data[9] << 8) + int(valid_data[10])
        dianliang = int(valid_data[11] << 8) + int(valid_data[12])
        vx_list = bytes([valid_data[13], valid_data[14]])
        vx = struct.unpack('>h', vx_list)[0]
        vy_list = bytes([valid_data[15], valid_data[16]])
        vy = struct.unpack('>h', vy_list)[0]
        vz_list = bytes([valid_data[17], valid_data[18]])
        vz = struct.unpack('>h', vz_list)[0]
        
        # 有符号4位整数
        lat_list = bytes([valid_data[19], valid_data[20], valid_data[21], valid_data[22]])
        lat = struct.unpack('>l', lat_list)[0]
        
        # 有符号4位整数
        lon_list = bytes([valid_data[23], valid_data[24], valid_data[25], valid_data[26]])
        lon = struct.unpack(">l", lon_list)[0]

        # 有符号4位整数
        alt_list = bytes([valid_data[27], valid_data[28]])
        alt = struct.unpack(">h", alt_list)[0]

        # 有符号4位整数
        homelat_list = bytes([valid_data[29], valid_data[30], valid_data[31], valid_data[32]])
        homelat = struct.unpack(">l", homelat_list)[0]

        # 有符号4位整数
        homelon_list = bytes([valid_data[33], valid_data[34], valid_data[35], valid_data[36]])
        homelon = struct.unpack(">l", homelon_list)[0]

        # 有符号4位整数
        homealt_list = bytes([valid_data[37], valid_data[38]])
        homealt = struct.unpack(">h", homealt_list)[0]

        # print(lat,lon,alt,homelat,homelon,homealt)
        state = valid_data[39]
        fix_type = valid_data[40]
        hdop1 = int(valid_data[41] << 8)
        hdop2 = int(valid_data[42])
        hdop = hdop1 + hdop2
        heading1 = int(valid_data[43] << 8)
        heading2 = int(valid_data[44])
        heading = heading1 + heading2
        stacking_state = int(valid_data[45])
        animal_type =   int(valid_data[46])
        log_dict = {}
        log_dict["fly_mode"] = mode_list[valid_data[0]]
        log_dict["fly_time"] = feixingshijian / 1000
        log_dict["armed_state"] = armed_state
        log_dict["is_armable"] = is_armable
        log_dict["dianya"] = dianya / 100
        log_dict["dianliu"] = dianliu / 100
        log_dict["dianliang"] = dianliang / 100
        log_dict["vx"] = vx / 100
        log_dict["vy"] = vy / 100
        log_dict["vz"] = vz / 100
        log_dict["lat"] = lat / 1e6
        log_dict["lon"] = lon / 1e6
        log_dict["alt"] = alt
        log_dict["homelat"] = homelat / 1e6
        log_dict["homelon"] = homelon / 1e6
        log_dict["homealt"] = homealt
        log_dict["state"] = state
        log_dict["fix_type"] = fix_type
        log_dict["hdop"] = hdop
        log_dict["heading"] = heading
        log_dict["hight"] = alt-homealt
        log_dict["stacking_state"] = stacking_state
        log_dict["animal_type"] = animal_type

        logging.info(log_dict)
        print("log dict: ", log_dict)
        send_data = json.dumps(log_dict)
        
        try:
            udp_soc.sendto(send_data.encode("utf-8"), ('127.0.0.1', 12345))
        except Exception as e:
            print(e)
            continue


if __name__ == '__main__':
    # 初始化串口
    ser = serial.Serial('/dev/ttyS0', 57600)  # 请将'COM4'替换为您实际使用的串口
    print("串口已打开：", ser.name)
    job1 = threading.Thread(target=getXboxKeys)
    job2 = threading.Thread(target=dataRecv)
    job1.start()
    job2.start()
