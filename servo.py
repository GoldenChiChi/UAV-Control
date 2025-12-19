import time


class Pwm_Servo:
    def __init__(self, pwm_chip):
        from periphery import PWM
        self.mid_duty = 1500  # 中位脉宽 (μs)
        self.min_duty = 500  # 最小脉宽 (μs)
        self.max_duty = 2500  # 最大脉宽 (μs)
        self.pwm_freq = 100  # PWM频率 (Hz)
        self.pwm_chip = pwm_chip
        self.pwm_channel = 0
        self.pwm = None
        self.duty = self.mid_duty
        # 初始化舵机
        self.init_pwm()

    def init_pwm(self):
        from periphery import PWM
        print('舵机初始化开始')
        # 初始化PWM
        self.pwm = PWM(self.pwm_chip, self.pwm_channel)
        self.pwm.frequency = self.pwm_freq
        self.pwm.polarity = 'normal'
        self.pwm.enable()
        self._move_check_()
        print('舵机初始化结束')

    def back_mid(self):
        self.set_duty_cycle_(self.mid_duty)

    def set_duty_cycle_(self, duty):
        """设置舵机脉宽"""
        self.duty = duty
        duty_cycle = duty / (1000000 / self.pwm_freq)
        self.pwm.duty_cycle = duty_cycle

    def get_point(self):
        '''
        获取摄像头当前的朝向角
        '''
        duty = (self.pwm.duty_cycle * 1000000 / self.pwm_freq)
        return (duty - self.min_duty) / (self.max_duty - self.min_duty) * 180

    def _move_check_(self):
        """舵机自检"""
        self.set_duty_cycle_(self.min_duty)
        time.sleep(1)
        self.set_duty_cycle_(self.mid_duty)
        time.sleep(1)
        self.set_duty_cycle_(self.max_duty)
        time.sleep(1)
        self.set_duty_cycle_(self.mid_duty)
        time.sleep(1)
        self.set_duty_cycle_(self.min_duty)
        time.sleep(1)
        self.set_duty_cycle_(self.mid_duty)
        time.sleep(1)


class ServoController:

    def __init__(self, pwm_servo, sbus_space, track_space, config, stop_event):
        # 系统参数
        self.pwm_servo = pwm_servo
        self.sbus_space = sbus_space
        self.track_space = track_space
        self.track = False
        self.config = config['control']['servo']
        self.stop_event = stop_event
        # 控制相关参数
        self.control_states = {'prev_error': 0, 'integral': [], 'control_mode': "ACCELERATING", 'max': 0, 'accel': 1.5,
                               'dead': 15, 'kp': self.config['track']['ypid']['kp'],
                               'ki': self.config['track']['ypid']['ki'], 'kd': self.config['track']['ypid']['kd']}

    def get_point(self):
        return self.pwm_servo.get_point()

    def get_duty_track_target(self, target_x, center_x):
        """
        根据目标X坐标控制舵机
        target_x: 目标在图像中的X坐标 (0~image_width)
        """
        # 计算目标与中心的偏差
        error = target_x - center_x
        if abs(error) > 200:
            error = 200 * error / abs(error)

        derivative = error - self.control_states['prev_error']
        self.control_states['prev_error'] = error

        output = 0
        # 状态转换逻辑
        if self.control_states['control_mode'] == "ACCELERATING":
            if abs(error) < 80:  # 接近目标速度，进入减速阶段
                self.control_states['control_mode'] = "DECELERATING"
                self.control_states['integral'] = []
        if self.control_states['control_mode'] == "DECELERATING":
            if abs(error) >= 60:  # 速度偏差过大，重新加速
                self.control_states['control_mode'] = "ACCELERATING"
                self.control_states['integral'] = []
        # 加速阶段
        if self.control_states['control_mode'] == "ACCELERATING":
            # 加速阶段使用全PID控制
            self.control_states['integral'].append(error)
            output = (self.control_states['kp'] * error +
                      self.control_states['ki'] * sum(self.control_states['integral']) +
                      self.control_states['kd'] * derivative)
        # 减速阶段
        if self.control_states['control_mode'] == "DECELERATING":
            # 减速阶段减弱积分项，增强微分项
            self.control_states['integral'].append(error * 0.4)
            output = (self.control_states['kp'] * error +
                      self.control_states['ki'] * sum(self.control_states['integral']) +
                      self.control_states['kd'] * derivative)

        if len(self.control_states['integral']) > 20:
            self.control_states['integral'].pop(0)
        duty = self.pwm_servo.duty - output * 0.6
        if abs(error) < 20:
            duty = self.pwm_servo.duty
        duty = int(max(self.pwm_servo.min_duty, min(self.pwm_servo.max_duty, duty)))
        return duty

    def close(self):
        """关闭PWM"""
        self.pwm_servo.close()

    def __call__(self):

        while not self.stop_event.is_set():
            try:
                if not self.config['switch'] or self.pwm_servo is None:
                    continue
                if not hasattr(self.track_space, 'track'):
                    continue
                target_box_class = self.track_space.track
                if target_box_class is None:
                    continue
                if target_box_class.tracking_status:
                    duty = self.get_duty_track_target(target_box_class.target_center_y,
                                                      target_box_class.center_y)
                    self.pwm_servo.set_duty_cycle_(duty)
            except Exception as e:
                print('ServoController __call__:', e)
            finally:
                time.sleep(0.04)
