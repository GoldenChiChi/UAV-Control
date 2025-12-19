import socket

import serial
import yaml
import time
import signal

from fastapi import FastAPI
from starlette.middleware.cors import CORSMiddleware
from starlette.responses import HTMLResponse
from dronekit import connect

from servo import ServoController, Pwm_Servo
from toremote import SerialSender
from fromremote import SerialSbusReader
from algo import Track
from vehiclecontrol import VehicleController


# 读取YAML文件
def read_yaml(file_path):
    with open(file_path, 'r') as file:
        try:
            return yaml.safe_load(file)
        except yaml.YAMLError as e:
            print(f"YAML 解析错误: {e}")
        return None


def signal_handler(sig, frame):
    print('捕获到 Ctrl+C，关闭服务器...')
    stop_event.set()
    for p in process_list:
        p.terminate()


def start_fastapi(config_yaml):
    """启动 FastAPI 服务"""
    app = FastAPI()
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"],
                       allow_headers=["*"])

    @app.get("/")
    async def read_root():
        return HTMLResponse(content="Hello, World!")

    import uvicorn
    host = config_yaml["control"]["fastapi"]["host"]
    port = config_yaml["control"]["fastapi"]["port"]
    print(f"启动 Uvicorn 服务器：{host}:{port}")
    uvicorn.run(app, host=host, port=port)


def init_connect(config_yaml, stop_event):
    conversion = config_yaml["control"]["conversion"]
    from_serial = None
    vehicle = None
    track_udp = None
    if conversion["from"] == "sbus":
        sbus_config = conversion["sbus"]
        try:
            from_serial = serial.Serial(sbus_config['serial'], baudrate=sbus_config["baudrate"], timeout=1)
        except serial.SerialException as e:
            print(f"无法打开sbuser串口：{e}")
            return
    if conversion["to"] == 'mavlink':
        while not stop_event.is_set():
            mavlink_config = conversion["mavlink"]
            try:
                if mavlink_config['binding'] == 'ip':
                    vehicle = connect('%s:%s' % (mavlink_config["ip"], mavlink_config["port"]),
                                      wait_ready=False)
                if mavlink_config['binding'] == 'serial':
                    vehicle = connect(mavlink_config["serial"], baud=mavlink_config["baud"], wait_ready=False)
                time.sleep(2)
                break
            except Exception as e:
                print("无人机飞控连接异常，重试", e)
                time.sleep(1)
    algo = config_yaml["control"]["algo"]
    if algo['me']:
        track_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        track_udp.bind((algo["me"]["ip"], algo["me"]["port"]))  # 替换为实际的端口

    servo = config_yaml['control']['servo']
    pwm_servo = None
    if servo['switch']:
        pwm_servo = Pwm_Servo(pwm_chip=servo['pwm_chip'])

    return from_serial, vehicle, track_udp, pwm_servo


if __name__ == "__main__":
    start_time = int(time.time() * 1000)
    config_path = 'config.yaml'
    config_yaml = read_yaml(config_path)
    if config_yaml is None:
        raise Exception(f'配置文件不存在: {config_path}')

    import multiprocessing

    manager = multiprocessing.Manager()
    sbus_space = manager.Namespace()
    track_space = manager.Namespace()
    context_space = manager.Namespace()
    stop_event = multiprocessing.Event()

    from_serial, vehicle, track_udp, pwm_servo = init_connect(config_yaml, stop_event)

    serial_reader = SerialSbusReader(from_serial, sbus_space, stop_event)
    track_algo = Track(track_udp, track_space, stop_event)
    serial_sender = SerialSender(from_serial, track_space, context_space, stop_event)
    servo_control = None
    if pwm_servo:
        servo_control = ServoController(pwm_servo, sbus_space, track_space, config_yaml, stop_event)
    vehicleControl = VehicleController(sbus_space, track_space, servo_control, context_space, vehicle, track_udp,
                                       config_yaml, stop_event)

    process_list = []
    process_list.append(multiprocessing.Process(target=serial_reader))
    process_list.append(multiprocessing.Process(target=track_algo.reciveUdp))
    process_list.append(multiprocessing.Process(target=serial_sender))
    process_list.append(multiprocessing.Process(target=servo_control))
    process_list.append(multiprocessing.Process(target=start_fastapi, args=(config_yaml,)))

    for p in process_list:
        p.start()

    # 捕获 Ctrl+C 信号
    signal.signal(signal.SIGINT, signal_handler)
    # 对无人机的控制必须放在主进程中，否则控制和状态返回不生效。
    vehicleControl.control_loop()

    for p in process_list:
        p.join()

    stop_event.clear()
    print('end')

# 基本模式
# 这些模式是飞控中最常用的基本模式，适合大多数任务。

# STABILIZE（稳定模式）
# 效果：手动控制无人机，飞控帮助维持姿态（Roll 和 Pitch 的自稳），不维持高度。
# 应用场景：适合初学者的手动飞行训练或应急情况。

# ACRO（特技模式）
# 效果：无人机不自稳，直接响应摇杆输入（控制角速度）。
# 应用场景：适用于特技飞行和高级手动飞行。

# ALT_HOLD（高度保持模式）
# 效果：飞控保持当前高度，用户手动控制水平运动和方向。
# 应用场景：适合需要悬停的拍摄或任务。

# AUTO（自动模式）
# 效果：无人机按照预定义的任务航点（Mission）飞行。
# 应用场景：用于复杂任务自动化，例如巡逻和测绘。

# GUIDED（引导模式）
# 效果：由地面站或程序实时发送目标点，飞控根据命令飞行。
# 应用场景：适合编程控制无人机任务。

# LOITER（盘旋模式）
# 效果：无人机自动维持当前位置，用户可以调整位置，飞控会自动悬停。
# 应用场景：适用于精确悬停和摄影。

# RTL（返航模式）
# 效果：无人机返回起飞点并自动降落。
# 应用场景：应急返航，低电量保护。

# CIRCLE（圆圈模式）
# 效果：无人机以指定半径绕一个点飞行。
# 应用场景：适合全景拍摄和观测。

# LAND（降落模式）
# 效果：无人机缓慢下降并安全着陆。
# 应用场景：任务结束时的标准操作。

# BRAKE（刹车模式）
# 效果：无人机快速减速并稳定悬停在当前位置。
# 应用场景：紧急停止或惯性对抗。

# 增强模式
# 这些模式提供更高的自动化或特定场景的功能。
#
# OF_LOITER（光流悬停模式）
# 效果：使用光流传感器在没有 GPS 的环境中悬停。
# 应用场景：室内飞行或 GPS 信号较弱的场景。

# DRIFT（漂移模式）
# 效果：飞控在前进时自动调整方向，手感类似固定翼。
# 应用场景：适用于动态拍摄或娱乐飞行。

# SPORT（运动模式）
# 效果：响应更快，适合高速和灵活的飞行。
# 应用场景：竞速飞行或娱乐飞行。

# FLIP（翻转模式）
# 效果：无人机自动做一个空中翻转。
# 应用场景：特技飞行表演。

# AUTOTUNE（自动调参模式）
# 效果：飞控自动调整 PID 参数以优化飞行性能。
# 应用场景：无人机首次飞行调试或性能优化。

# POSHOLD（位置保持模式）
# 效果：类似 LOITER，但手动输入时运动更灵敏。
# 应用场景：适合需要悬停又有一定灵活性的任务。

# THROW（投掷起飞模式）
# 效果：无人机检测到被投掷后自动启动电机并飞行。
# 应用场景：适合手动投掷起飞的场景。

# 特殊模式
# 这些模式针对特定任务或需求设计。
#
# AVOID_ADSB（避让模式）
# 效果：基于 ADS-B 数据避开其他飞行器。
# 应用场景：提高飞行安全性。

# GUIDED_NOGPS（无 GPS 引导模式）
# 效果：无需 GPS 的实时指令控制。
# 应用场景：适用于室内或 GPS 信号较弱的场景。

# SMART_RTL（智能返航模式）
# 效果：无人机沿着返回路径逐步后退到起飞点。
# 应用场景：节省能量的返航。

# FLOWHOLD（光流保持模式）
# 效果：使用光流传感器保持位置。
# 应用场景：室内环境。

# FOLLOW（跟随模式）
# 效果：无人机自动跟随指定目标。
# 应用场景：适合动态视频拍摄。

# ZIGZAG（之字形飞行模式）
# 效果：无人机沿之字形路径飞行。
# 应用场景：测绘或搜索任务。

# SYSTEMID（系统校准模式）
# 效果：测试和校准飞控的动力系统。
# 应用场景：飞控硬件调试。

# AUTOROTATE（自转模式）
# 效果：在动力丧失情况下模拟自动旋翼降落（适用于直升机）。
# 应用场景：直升机应急训练。

# AUTO_RTL（自动返航模式）
# 效果：任务完成后自动返航。
# 应用场景：任务自动化。
