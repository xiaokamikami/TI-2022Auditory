# Untitled - By: 13173 - 周三 7月 27 2022

import time,math,time,sys,lcd,image
import gc,machine
from Maix import GPIO
from board import board_info
from fpioa_manager import fm
from Maix import MIC_ARRAY as mic
from board import board_info

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
    _RC = 1/(2 * math.pi * 20)
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
color_G = (0,255,0)
def drawConfidenceText(image, rol, text, value):
    image.draw_string(rol[0], rol[1], text , color=color_G, scale=2)
global AngleX
global AngleY
global AngleR
global Angle
AngleX = 0
AngleY = 0
AngleR = 0
Angle = 0
def get_mic_dir():
    global AngleX
    global AngleY
    global AngleR
    global Angle
    AngleX = 0
    AngleY = 0
    AngleR = 0
    Angle = 0
    AngleAddPi=0
    #mic_list=[]
    imga = mic.get_map()    # 获取声音源分布图像
    imgb = imga.resize(160,160)
    imgc = imgb.to_rainbow(1)
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
        #mic_list.append(AngleX)
        #mic_list.append(AngleY)
        #mic_list.append(AngleR)
        #mic_list.append(Angle)


    a = mic.set_led(b,(0,0,255))# 配置 RGB LED 颜色值

    #计算距离
    distance = 250/(math.cos(AngleR));
    text1 = 'D:'+str(distance)+'CM'
    text2 = 'Roll:' + str(Angle)
    drawConfidenceText(imgc,(0,0),text1,2)
    drawConfidenceText(imgc,(0,24),text2,2)
    #输出信息   测评时关闭
    if(AngleR != 0):
        print("x:",AngleX ,"y:",AngleY,"roll:",Angle,"强度:",AngleR,"距离",distance)
    #显示声源图
    lcd.display(imgc)

#系统初始化
gc.collect()    #垃圾回收器
lcd.init()
mic.init(i2s_d0=34, i2s_d1=8, i2s_d2=33, i2s_d3=9, i2s_ws=32, i2s_sclk=10,sk9822_dat=7, sk9822_clk=35)#可自定义配置 IO
fm.register(21, fm.fpioa.GPIO0)
fm.register(22, fm.fpioa.GPIO1)


#外设初始化
img = image.Image()

pitch_pwm = machine.PWM(1, freq=50, duty=11, pin=25)#开机回中
roll_pwm  = machine.PWM(2, freq=50, duty=20, pin=24)
pitch_pwm.enable()
roll_pwm.enable()
gpio_key1 = GPIO(GPIO.GPIO0,GPIO.OUT)
gpio_red  = GPIO(GPIO.GPIO1,GPIO.OUT)
num = 2
#test
#while(num<80):
    #num = num+1
    #roll_pwm.duty(num)
    #print(num)
    #time.sleep_ms(500)

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
init_pitch = 50       # init position, value: [0, 100], means minimum angle to maxmum angle of servo
init_roll = 50        # 50 means middle
mode = 0              # 0为指向模式  1为追踪模式

pitch_pid = [0.25, 0, 0.015, 0]  # P I D I_max
roll_pid  = [0.25, 0, 0.015, 0]  # P I D I_max
pitch = Servo(pitch_pwm, dir=init_pitch)
roll = Servo(roll_pwm, dir=init_roll)
pid_pitch = PID(p=pitch_pid[0], i=pitch_pid[1], d=pitch_pid[2], imax=pitch_pid[3])
pid_roll = PID(p=roll_pid[0], i=roll_pid[1], d=roll_pid[2], imax=roll_pid[3])
gimbal = Gimbal(pitch, pid_pitch, roll, pid_roll)

target_pitch = init_pitch
target_roll = init_roll

t0 = 0
last_run = 0
#////////初始化完成
while True:
       global imgc
# get target error
       # interval limit to > 20ms
       if time.ticks_ms() - t0 < 20:
           continue
       t0 = time.ticks_ms()
       # run
       sound = []
       if(gpio_key1.value()==0):
           mode = 1
       else:
           mode = 0
       get_mic_dir()        #更新麦克风数据
       err_roll = 0-AngleX
       err_pitch = AngleY
       if( mode == 1):      #跟踪模式
           gpio_red.value(1)
           if(AngleR > 10):#阈值
               gimbal.run(err_pitch, err_roll, pitch_reverse =False, roll_reverse=True)
       else :               #非跟踪
           gpio_red.value(0)
           if(err_roll-3< last_run and last_run < err_roll+3 ):
           #计算距离
               servo_duty = AngleR *1.0/180*10+2.5
               distance = 250/(math.cos(sound[3]))
               #显示信息
               text = 'D:'+str(distance)+'CM'+'Roll:' + str(Angle)
               if(AngleR > 6):#阈值
                    gimbal.run(err_pitch, err_roll, pitch_reverse =False, roll_reverse=True)
               if( mode == 0):      #非跟踪模式
                    gpio_red.value(1)
                    time.sleep(1)
                    gpio_red.value(0)

           last_run = err_roll
#关机
mic.deinit()
pwm.deinit()
ips.display(img)
