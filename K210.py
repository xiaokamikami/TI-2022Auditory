# Untitled - By: 13173 - 周三 7月 27 2022

import time,math,time,sys
from Maix import MIC_ARRAY as mic
from machine import Timer,PWM
class Servo:
    def __init__(self, pwm, dir=50, duty_min=2.5, duty_max=12.5):
        self.value = dir
        self.pwm = pwm
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_range = duty_max -duty_min
        self.enable(True)
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)

    def enable(self, en):
        if en:
            self.pwm.enable()
        else:
            self.pwm.disable()

    def dir(self, percentage):
        if percentage > 100:
            percentage = 100
        elif percentage < 0:
            percentage = 0
        self.pwm.duty(percentage/100*self.duty_range+self.duty_min)

    def drive(self, inc):
        self.value += inc
        if self.value > 100:
            self.value = 100
        elif self.value < 0:
            self.value = 0
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)
class Gimbal:
    def __init__(self, pitch, pid_pitch, roll=None, pid_roll=None, yaw=None, pid_yaw=None):
        self._pitch = pitch
        self._roll = roll
        self._yaw = yaw
        self._pid_pitch = pid_pitch
        self._pid_roll = pid_roll
        self._pid_yaw = pid_yaw

    def set_out(self, pitch, roll, yaw=None):
        pass

    def run(self, pitch_err, roll_err=50, yaw_err=50, pitch_reverse=False, roll_reverse=False, yaw_reverse=False):
        out = self._pid_pitch.get_pid(pitch_err, 1)
        # print("err: {}, out: {}".format(pitch_err, out))
        if pitch_reverse:
            out = - out
        self._pitch.drive(out)
        if self._roll:
            out = self._pid_roll.get_pid(roll_err, 1)
            if roll_reverse:
                out = - out
            self._roll.drive(out)
        if self._yaw:
            out = self._pid_yaw.get_pid(yaw_err, 1)
            if yaw_reverse:
                out = - out
            self._yaw.drive(out)
class PID:
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_t = 0
    _RC = 1/(2 * pi * 20)
    def __init__(self, p=0, i=0, d=0, imax=0):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = None

    def get_pid(self, error, scaler):
        tnow = time.ticks_ms()
        dt = tnow - self._last_t
        output = 0
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()
        self._last_t = tnow
        delta_time = float(dt) / float(1000)
        output += error * self._kp
        if abs(self._kd) > 0 and dt > 0:
            if self._last_derivative == None:
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            derivative = self._last_derivative + \
                                     ((delta_time / (self._RC + delta_time)) * \
                                        (derivative - self._last_derivative))
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax
            output += self._integrator
        return output

    def reset_I(self):
        self._integrator = 0
        self._last_derivative = None

def get_mic_dir():
    AngleX=0
    AngleY=0
    AngleR=0
    Angle=0
    AngleAddPi=0
    mic_list=[]
    imga = mic.get_map()    # 获取声音源分布图像
    b = mic.get_dir(imga)   # 计算、获取声源方向
    for i in range(len(b)):
        if b[i]>=2:
            AngleX+= b[i] * math.sin(i * math.pi/6)
            AngleY+= b[i] * math.cos(i * math.pi/6)
    AngleX=round(AngleX,6) #计算坐标转换值
    AngleY=round(AngleY,6)
    if AngleY<0:AngleAddPi=180
    if AngleX<0 and AngleY > 0:AngleAddPi=360
    if AngleX!=0 or AngleY!=0: #参数修正
        if AngleY==0:
            Angle=90 if AngleX>0 else 270 #填补X轴角度
        else:
            Angle=AngleAddPi+round(math.degrees(math.atan(AngleX/AngleY)),4) #计算角度
        AngleR=round(math.sqrt(AngleY*AngleY+AngleX*AngleX),4) #计算强度
        mic_list.append(AngleX)
        mic_list.append(AngleY)
        mic_list.append(AngleR)
        mic_list.append(Angle)
        print("强度:",AngleR)
    a = mic.set_led(b,(0,0,255))# 配置 RGB LED 颜色值
    return mic_list #返回列表，X坐标，Y坐标，强度，角度
#系统初始化
gc.collect()    #垃圾回收器

lcd.init()
mic.init()
#mic.init(i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20, i2s_ws=19, i2s_sclk=18, sk9822_dat=24, sk9822_clk=25)
#外设初始化
img = image.Image()
pitch_pwm = PWM(tim0, freq=50, duty=50, pin=24)#开机回中
roll_pwm  = PWM(tim1, freq=50, duty=50, pin=25)

'''
    servo:
        freq: 50 (Hz)
        T:    1/50 = 0.02s = 20ms
        duty: [0.5ms, 2.5ms] -> [0.025, 0.125] -> [2.5%, 12.5%]
    pin:
        IO24 <--> pitch
        IO25 <--> roll
'''

#变量初始化
init_pitch = 80       # init position, value: [0, 100], means minimum angle to maxmum angle of servo
init_roll = 50        # 50 means middle

pitch_pid = [0.23, 0, 0.015, 0]  # P I D I_max
roll_pid  = [0.23, 0, 0.015, 0]  # P I D I_max
pitch = Servo(pitch_pwm, dir=init_pitch)
roll = Servo(roll_pwm, dir=init_roll)
pid_pitch = PID(p=pitch_pid[0], i=pitch_pid[1], d=pitch_pid[2], imax=pitch_pid[3])
pid_roll = PID(p=roll_pid[0], i=roll_pid[1], d=roll_pid[2], imax=roll_pid[3])
gimbal = Gimbal(pitch, pid_pitch, roll, pid_roll)

target_pitch = init_pitch
target_roll = init_roll

sound = [0,0,0,0]
distance = 0
#////////初始化完成
while True:
# get target error
       # interval limit to > 10ms
       if time.ticks_ms() - t0 < 10:
           continue
       t0 = time.ticks_ms()
       # run
       sound = get_mic_dir()
       err_roll = sound[0]
       err_pitch = sound[1]
       #计算距离
       distance = 250/(math.cos(sound[3]));
       #输出信息
       text = 'D:'+str(distance)+'CM' 'Roll:' + str(sound[3])
       if(sound[2] > 2):#阈值
            gimbal.run(err_pitch, err_roll, pitch_reverse = pitch_reverse, roll_reverse=roll_reverse)
            image.draw_string(rol[0], rol[1], text , color=color_G, scale=2.5)
#关机
mic.deinit()
pwm.deinit()
ips.display(img)
