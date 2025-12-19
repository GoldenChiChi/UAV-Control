import serial
import struct
import time

PORT = "/dev/ttyUSB1"
BAUDRATE = 57600
TIMEOUT = 0.5  # 秒

FRAME_HEAD = 0x0F

def read_exact(ser, size):
    """确保读取到指定字节数"""
    data = b""
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT
    )

    print(f"Opened serial {PORT} @ {BAUDRATE}")

    try:
        while True:
            # 1. 寻找帧头 0x0F
            byte = ser.read(1)
            if not byte:
                continue

            if byte[0] != FRAME_HEAD:
                continue

            # 2. 读取长度字段（2 字节）
            length_bytes = read_exact(ser, 2)
            if length_bytes is None:
                continue

            # ===== 大端解析 =====
            length = (length_bytes[0] << 8) | length_bytes[1]

            # ===== 如果是小端，用下面这行替换 =====
            # length = (length_bytes[1] << 8) | length_bytes[0]

            if length <= 0 or length > 4096:
                print(f"Invalid length: {length}")
                continue

            # 3. 读取数据区
            payload = read_exact(ser, length)
            if payload is None:
                continue

            # 4. 处理数据
            print(f"Frame received:")
            print(f"  Length : {length}")
            print(f"  Payload: {payload.hex(' ')}")

    except KeyboardInterrupt:
        print("Exit by Ctrl+C")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

