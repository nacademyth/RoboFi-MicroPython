from ST7735 import TFT
from sysfont import sysfont
from machine import SPI,Pin, PWM, ADC
from servo import Servo
import time
import math

# .. MOTOR ..........................
M1_EN 			= 4
M1_DIR 			= 21
M2_EN 			= 32
M2_DIR 			= 33
M3_EN 			= 22
M3_DIR 			= 25
M4_EN 			= 26
M4_DIR 			= 27	
FREQ 			= 5000
RESOLUTION		= 8
ALL				= 1234

#.. SERVO ..........................
SERVO1			= 19
SERVO2			= 16
SERVO3			= 12

#.. GLCD ............................
GLCD_RED		= 0xf800
GLCD_GREEN		= 0x07e0
GLCD_BLUE		= 0x001f
GLCD_BLACK		= 0x0000
GLCD_WHITE		= 0xffff

# .. ANALOG ..........................
AN0				= 36								
AN1				= 39								
AN2				= 34							
AN3				= 35								
AN4				= 14								
AN5				= 2		

# .. BUZZER ..........................
BUZZER     		= 17 

# .. SW ..............................
SW				= 13

spi = SPI(2, baudrate=20000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(12))
tft=TFT(spi, 15, 5)
tft.initr()
tft.rgb(True)

glcd_init = False

val             = [None]*6
val_min         = [None]*6
val_max         = [None]*6

def analog(ch):
    adc = ADC(Pin(ch))
    adc.atten(ADC.ATTN_11DB)
    return adc.read()

# .....................................
servo1  = Servo(pin=SERVO1)
servo2  = Servo(pin=SERVO2)
servo3  = Servo(pin=SERVO3)

def servo(servo, angle):
    if(servo == 1): 
        servo1.move(angle)
    if(servo == 2): 
        servo2.move(angle)
    if(servo == 3): 
        servo3.move(angle)

# .....................................
class Motor(object):
    def stop(self):
        self.pwm.duty(0)
        
    def forward(self, duty):
        duty_t = int((duty*1023)/100)
        self.pwm.duty(duty_t)
        self.dir(0)
        
    def backward(self, duty):
        duty_t = int((duty*1023)/100)
        self.pwm.duty(duty_t)
        self.dir(1)
        
    def __init__(self, motor):
        if(motor==1):         
            self.pwm    = PWM(Pin(M1_EN), freq=FREQ)
            self.dir    = Pin(M1_DIR, Pin.OUT)
        elif(motor==2):
            self.pwm    = PWM(Pin(M2_EN), freq=FREQ)
            self.dir    = Pin(M2_DIR, Pin.OUT)
        elif(motor==3):
            self.pwm    = PWM(Pin(M3_EN), freq=FREQ)
            self.dir    = Pin(M3_DIR, Pin.OUT)
        elif(motor==4):
            self.pwm    = PWM(Pin(M4_EN), freq=FREQ)
            self.dir    = Pin(M4_DIR, Pin.OUT)

motor1  = Motor(1)
motor2  = Motor(2)
motor3  = Motor(3)
motor4  = Motor(4)

def motor(ch, speed):
    if(ch==1):
        if(speed > 0):
            motor1.forward(speed)
        else:
            motor1.backward(abs(speed))
    elif(ch==2):
        if(speed > 0):
            motor2.forward(speed)
        else:
            motor2.backward(abs(speed))
    elif(ch==3):
        if(speed > 0):
            motor3.forward(speed)
        else:
            motor3.backward(abs(speed))
    elif(ch==4):
        if(speed > 0):
            motor4.forward(speed)
        else:
            motor4.backward(abs(speed))

def fd(duty):
    motor1.forward(duty)
    motor2.forward(duty)
    motor3.forward(duty)
    motor4.forward(duty)
    
def bk(duty):
    motor1.backward(duty)
    motor2.backward(duty)
    motor3.backward(duty)
    motor4.backward(duty)

def sl(duty):
    motor1.backward(duty)
    motor2.forward(duty)
    motor3.backward(duty)
    motor4.forward(duty)
    
def sll(duty):
    motor1.backward(duty)
    motor2.forward(duty)
    motor3.forward(duty)
    motor4.backward(duty)
    
def sr(duty):
    motor1.forward(duty)
    motor2.backward(duty)
    motor3.forward(duty)
    motor4.backward(duty)
    
def slr(duty):
    motor1.forward(duty)
    motor2.backward(duty)
    motor3.backward(duty)
    motor4.forward(duty)


def motor_stop(motor=None):
    if(motor==None):
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
    elif(motor==1234 or motor=="ALL"):
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
    elif(motor==1):         
        motor1.stop()
    elif(motor==2):
        motor2.stop()
    elif(motor==3):
        motor3.stop()
    elif(motor==4):
        motor4.stop()

# .....................................
def glcdConfig():
    global glcd_init
    if(not glcd_init):
        tft.rotation(1)
        tft.fill(TFT.BLACK)
        glcd_init = True
        
def glcdClear():
    glcdConfig()
    tft.fill(TFT.BLACK)
    
def glcd(x,y,text):
    glcdConfig()
    tft.text((x, y), text, tft.textColor, sysfont, tft.textSize, nowrap=True)

def setTextSize(newSize):
    glcdConfig()
    tft.textSize = newSize

def glcdFillRect(x, y, w, h, color):
    glcdConfig()
    tft.fillrect((x,y), (w,h), color)

def glcdVLine(x,y,h):
    glcdConfig()
    tft.vline((x, y), h, TFT.WHITE)

def glcdHLine(x,y,w):
    glcdConfig()
    tft.hline((x,y), w, TFT.WHITE)

def setTextColor(color):
    glcdConfig()
    tft.textColor = color
# .....................................

def beep():
    buzzer = Pin(BUZZER, Pin.OUT)
    buzzer(1)
    time.sleep(0.1)
    buzzer(0)

# .....................................
def sw_OK_press():
    sw = Pin(SW, Pin.IN)
    return not bool(sw.value())

def sw_OK():
    glcdClear()
    setTextSize(2)
    glcd(0,0,"Press OK...")
    while (sw_OK_press() == False):
        time.sleep(0.1) 
    beep();
    glcdClear()

def readAllAnalogs():
    global val
    global val_min
    global val_max
    
    beep()
    
    for i in range(6):
        glcdFillRect(32,  i*15+35, 25, 10, GLCD_BLACK)
        glcdFillRect(64,  i*15+35, 25, 10, GLCD_BLACK)
        glcdFillRect(96,  i*15+35, 25, 10, GLCD_BLACK)
        glcdFillRect(128, i*15+35, 25, 10, GLCD_BLACK)

        if(i==0):
            val[i] = analog(AN0)
        if(i==1):
            val[i] = analog(AN1)
        if(i==2):
            val[i] = analog(AN2)
        if(i==3):
            val[i] = analog(AN3)
        if(i==4):
            val[i] = analog(AN4)
        if(i==5):
            val[i] = analog(AN5)

        if(val[i] < val_min[i]):
            val_min[i] = val[i]
        if(val[i] > val_max[i]):
            val_max[i] = val[i]

        glcd(32,  i*15+35,  f"{val[i]}")
        glcd(64,  i*15+35,  f"{val_min[i]}")
        glcd(96,  i*15+35,  f"{val_max[i]}")
        glcd(128,  i*15+35, f"{int((val_max[i]-val_min[i]) / 2) + val_min[i]}")
    
def demo():
    global val
    global val_min
    global val_max
    
    sw_OK()
    setTextSize(1)
    glcd(0,0, "RoboFi by N Academy")
    glcd(0,10,"ANALOG INPUT DASHBOARD")
    
    glcd(0,   20,  "AN")
    glcd(32,  20,  "PV")
    glcd(64,  20,  "MIN")
    glcd(96,  20,  "MAX")
    glcd(128, 20,  "AVG")
    
    glcd(0,   35,  "A0")
    glcd(0,   50,  "A1")
    glcd(0,   65,  "A2")
    glcd(0,   80,  "A3")
    glcd(0,   95,  "A4")
    glcd(0,   110,  "A5")
    glcdHLine(0, 30, tft.size()[0]);


    # Initial min
    val_min[0] = analog(AN0);
    val_min[1] = analog(AN1);
    val_min[2] = analog(AN2);
    val_min[3] = analog(AN3);
    val_min[4] = analog(AN4);
    val_min[5] = analog(AN5);

    # Initial max
    val_max[0] = analog(AN0);
    val_max[1] = analog(AN1);
    val_max[2] = analog(AN2);
    val_max[3] = analog(AN3);
    val_max[4] = analog(AN4);
    val_max[5] = analog(AN5);
	
    while(True):
        for i in range(1,3):

            servo(1, 0)
            servo(2, 0)
            servo(3, 0)

            speed = 50*i
            readAllAnalogs()
            fd(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 30)
            servo(2, 30)
            servo(3, 30)

            readAllAnalogs()
            bk(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 60)
            servo(2, 60)
            servo(3, 60)

            readAllAnalogs()
            sll(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 90)
            servo(2, 90)
            servo(3, 90)

            readAllAnalogs()
            slr(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 120)
            servo(2, 120)
            servo(3, 120)

            readAllAnalogs()
            sl(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 150)
            servo(2, 150)
            servo(3, 150)

            readAllAnalogs()
            sr(speed)
            time.sleep(1)
            motor_stop()  
            time.sleep(1)

            servo(1, 180)
            servo(2, 180)
            servo(3, 180)
            

motor_stop()
beep()
