# Untitled - By: 13173 - 周三 7月 27 2022

import time,math,time,lcd,image
import gc,machine
from Maix import GPIO
from board import board_info
from fpioa_manager import fm
from Maix import MIC_ARRAY as mic
from board import board_info
from Maix import utils
from machine import Timer,PWM
#utils.gc_heap_size(256*1024)
#machine.reset()


color_G = (0,255,0)
def drawConfidenceText(image, rol, text, value):
    image.draw_string(rol[0], rol[1], text , color=color_G, scale=2)
global AngleX
global AngleY
global AngleR
global Angle
global out_flag
AngleX = 0
AngleY = 0
AngleR = 0
Angle = 0
fc = 3    # //截止频率
Ts = 0.05   #//采样周期
alpha = 0   #  //滤波系数
b = 2.0 * math.pi * fc * Ts;
alpha = b / (b + 1);
global lv_num
lv_num =0
def get_mic_dir():
    global AngleX
    global AngleY
    global AngleR
    global Angle
    global lv_num
    L_AngleX_IN = 0
    L_AngleY_IN = 0
    L_AngleR_IN = 0
    L_Angle_IN = 0
    L_AngleAddPi_IN=0
    AngleX_IN = 0
    AngleY_IN = 0
    AngleR_IN = 0
    Angle_IN = 0
    AngleAddPi_IN=0
    lv_num = 0
    while(lv_num<2):
        imga = mic.get_map()    # 获取声音源分布图像
        imgb = imga.resize(160,160)
        imgc = imgb.to_rainbow(1)
        b = mic.get_dir(imga)   # 计算、获取声源方向
        a = mic.set_led(b,(0,0,255))# 配置 RGB LED 颜色值
        for i in range(len(b)):
            if b[i]>=2:
                AngleX_IN+= b[i] * math.sin(i * math.pi/6)
                AngleY_IN+= b[i] * math.cos(i * math.pi/6)
        AngleX_IN=round(AngleX_IN,6) #计算坐标转换值
        AngleY_IN=round(AngleY_IN,6)
        if AngleY_IN<0:AngleAddPi_IN=180
        if AngleX_IN<0 and AngleY_IN > 0:AngleAddPi_IN=360
        if AngleX_IN!=0 or AngleY_IN!=0: #参数修正
            if AngleY_IN==0:
                Angle_IN=90 if AngleX_IN>0 else 270 #填补X轴角度
            else:
                Angle_IN=AngleAddPi_IN+round(math.degrees(math.atan(AngleX_IN/AngleY_IN)),4) #计算角度
            AngleR_IN=round(math.sqrt(AngleY_IN*AngleY_IN+AngleX_IN*AngleX_IN),4) #计算强度
        if(AngleR_IN >10):
            lv_num = lv_num+1
            if(lv_num==1):
                L_AngleX_IN = AngleX_IN
                L_AngleY_IN = AngleY_IN
                L_AngleR_IN = AngleR_IN
                L_Angle_IN = Angle_IN
                L_AngleAddPi_IN=AngleAddPi_IN
            else:
                AngleX = L_AngleX_IN + alpha*(AngleX_IN - L_AngleX_IN)
                AngleY = L_AngleY_IN + alpha*(AngleY_IN - L_AngleY_IN)
                AngleR = L_AngleR_IN + alpha*(AngleR_IN - L_AngleR_IN)
                Angle = L_Angle_IN + alpha*(Angle_IN - L_AngleX_IN)
                AngleAddPi = L_AngleAddPi_IN + alpha*(AngleAddPi_IN - L_AngleAddPi_IN)
        time.sleep_ms(20)
        #计算距离
        distance = math.sqrt(62500+math.pow((AngleX*2),2))
        text1 = 'Di:'+str(distance)+'CM'
        text2 = 'Roll:' + str(Angle)
        drawConfidenceText(imgc,(0,0),text1,2)
        drawConfidenceText(imgc,(0,24),text2,2)
        #mic_list.append(AngleX)
        #mic_list.append(AngleY)
        #mic_list.append(AngleR)
        #mic_list.append(Angle)


        #输出信息   测评时关闭
        if(AngleR != 0):
            print("x:",AngleX ,"y:",AngleY,"roll:",Angle,"强度:",AngleR,"距离",distance)
        #print("xyrrd:",AngleX,AngleY,Angle,AngleR,distance)
        #print("x:",AngleX)
        #time.sleep_ms(1)
        #print("y:",AngleY)
        #time.sleep_ms(1)
        #print("roll:",Angle)
        #time.sleep_ms(1)
        #print("R:",AngleR)
        #time.sleep_ms(1)
        #print("DIS:",distance)
        #time.sleep_ms(1)
        #显示声源图
        lcd.display(imgc)
        gc.collect()    #垃圾回收器
#系统初始化
gc.collect()    #垃圾回收器
lcd.init()
mic.init(i2s_d0=34, i2s_d1=8, i2s_d2=33, i2s_d3=9, i2s_ws=32, i2s_sclk=10,sk9822_dat=7, sk9822_clk=35)#可自定义配置 IO
fm.register(21, fm.fpioa.GPIO0)
fm.register(22, fm.fpioa.GPIO1)


#外设初始化
init_pitch = 100       # init position, value: [0, 100], means minimum angle to maxmum angle of servo
init_roll = 51.5           # 50 means middle
img = image.Image()
tim1 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
tim2 = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
pitch_pwm = machine.PWM(tim1, freq=50, duty=51.5, pin=24)#开机回中
roll_pwm  = machine.PWM(tim2, freq=50, duty=51.5, pin=25)
pitch_pwm.enable()
roll_pwm.enable()
gpio_key1 = GPIO(GPIO.GPIO0,GPIO.IN,GPIO.PULL_UP)
gpio_red  = GPIO(GPIO.GPIO1,GPIO.OUT)
num = 45
Q_duck =0
#gpio_red.value(1)
roll_pwm.duty(51.6)
pitch_pwm.duty(51.5)
time.sleep_ms(2000)
#test
#while(num<52):
    #num = num+1
    ###roll_pwm.duty(num)
    #pitch_pwm.duty(num)
    #print(num)
    #time.sleep_ms(300)

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

mode = 0              # 0为指向模式  1为追踪模式

t0 = 0
last_run = 0
last_gen = 0
tage_run = 51.5
if(gpio_key1.value()==0):
    mode = 0
else:
    mode = 1
print("mode:",mode)
B_su = 0.9
min_dock = 47
max_dock = 56
#////////初始化完成
while True:
       global imgc
# get target error
       # interval limit to > 50ms
       if time.ticks_ms() - t0 < 20:
           continue
       t0 = time.ticks_ms()
       # run

       get_mic_dir()        #更新麦克风数据
       #err_pitch = last_run - AngleX
       err_pitch = AngleX
       err_roll = AngleY
       #Xprint("tagX:",err_pitch)
       if( mode == 1):      #跟踪模式
           gpio_red.value(1)
           if(AngleR > 5):#阈值
               out =  -(AngleR *B_su/180*10)+51.5
               if(last_gen !=out):
                   last_gen = out
                   if   out > max_dock:
                        out = max_dock
                   elif  out < min_dock:
                         out =  min_dock
                   if(  tage_run <  out):
                         tage_run = tage_run +0.2
                   if(  otage_run >  out ):
                         tage_run = tage_run -0.2
                   print("outX:",out)
                   pitch_pwm.duty(out)
                   time.sleep_ms(200)

       else :               #非跟踪
           gpio_red.value(0)

           if(AngleR > 8 ):#阈值
                    time.sleep_ms(20)
                    #快速转向
                    Q_duck =  -(AngleR *B_su/180*10)+51.5
                    if Q_duck > 56:
                        Q_duck = 56
                    elif Q_duck <47:
                        Q_duck  = 47
                    print("out:",Q_duck)
                    pitch_pwm.duty(Q_duck)
                    gpio_red.value(1)
                    time.sleep(2)
                    AngleR = 0
                    #gpio_red.value(0)

           last_run = err_roll
       time.sleep_ms(20)
       last_run = AngleX
       gc.collect()    #垃圾回收器
#关机
mic.deinit()

