import cv2

def main():
    # 0 表示第一个 USB 摄像头
    # 如果不行，试试 1、2
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("❌ 无法打开摄像头")
        return

    print("✅ 摄像头已打开，按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 读取失败")
            break

        # 显示画面
        cv2.imshow("USB Camera", frame)

        # 按 q 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
