from pwminit import *
from machine import Pin, PWM, I2C
from imu_new import MPU6050
import math
import time
#Set I2C pins
i2c=I2C(0, sda=Pin(16), scl=Pin(17), freq=200000)

mpu = MPU6050(i2c)
#IMU constants
roll=0
pitch=0
yaw=0
tLoop=0
cnt=0

#Set the direction based on the value of the amplitude
def Acc1(W):
    if W > 0:
        m1f.duty_u16(abs(W))
        m1b.duty_u16(0)
    if W < 0:
        m1b.duty_u16(abs(W))
        m1f.duty_u16(0)
    else:
        m1b.duty_u16(0)
        m1f.duty_u16(0)
def Acc2(W):
    if W > 0:
        m2f.duty_u16(abs(W))
        m2b.duty_u16(0)
    if W < 0:
        m2b.duty_u16(abs(W))
        m2f.duty_u16(0)
    else:
        m2b.duty_u16(0)
        m2f.duty_u16(0)
def Acc3(W):
    if W > 0:
        m3f.duty_u16(abs(W))
        m3b.duty_u16(0)
    if W < 0:
        m3b.duty_u16(abs(W))
        m3f.duty_u16(0)
    else:
        m3b.duty_u16(0)
        m3f.duty_u16(0)
def Acc4(W):
    if W > 0:
        m4f.duty_u16(abs(W))
        m4b.duty_u16(0)
    if W < 0:
        m4b.duty_u16(abs(W))
        m4f.duty_u16(0)
    else:
        m4b.duty_u16(0)
        m4f.duty_u16(0)
def Acc5(W):
    if W > 0:
        m5f.duty_u16(abs(W))
        m5b.duty_u16(0)
    if W < 0:
        m5b.duty_u16(abs(W))
        m5f.duty_u16(0)
    else:
        m5b.duty_u16(0)
        m5f.duty_u16(0)
        
def Acc6(W):
    if W > 0:
        m6f.duty_u16(abs(W))
        m6b.duty_u16(0)
    if W < 0:
        m6b.duty_u16(abs(W))
        m6f.duty_u16(0)
    else:
        m6b.duty_u16(0)
        m6f.duty_u16(0)
        
def Acc7(W):
    if W > 0:
        m7f.duty_u16(abs(W))
        m7b.duty_u16(0)
    if W < 0:
        m7b.duty_u16(abs(W))
        m7f.duty_u16(0)
    else:
        m7b.duty_u16(0)
        m7f.duty_u16(0)
        
def Acc8(W):
    if W > 0:
        m8f.duty_u16(abs(W))
        m8b.duty_u16(0)
    if W < 0:
        m8b.duty_u16(abs(W))
        m8f.duty_u16(0)
    else:
        m8b.duty_u16(0)
        m8f.duty_u16(0)
        
#Set the wavelength, frequency and the maximum amplitude
wavelength = 220 #mm
frequency = 1 #Hz
amp = 65536

#h is constant so does not need to be in the loop
h=2*pi/wavelength

# Position of actuators
x1= 0#mm
x2 = 33#mm
x3 = 66#mm
x4 = 99#mm

#PID constants for direction
derror = 0
freq = 0
dlast_error = 0
dintergral = 0
dkp = 0
dki = 0
dkd = 0
dtarget = 0

#Forever loop 
while True:
    #Sets the last error as the error from the previous loop then calculates the change in error
    dlast_error = derror
    dderivative = derror - dlast_error
    #Starts the timer for future Gyro calculations
    tStart=time.ticks_ms()
    #Calling the Gyro and accelerometer functions
    xGyro=mpu.gyro.x
    yGyro=mpu.gyro.y
    zGyro=mpu.gyro.z
    
    xAccel=mpu.accel.x
    yAccel=mpu.accel.y
    zAccel=mpu.accel.z
    
    # Calculating roll, pitch and Yaw
    roll=roll+yGyro*tLoop
    pitch=pitch+xGyro*tLoop
    yaw=yaw+zGyro*tLoop
    
    # stoping the timer and setting the loop time for the above calculations
    tStop=time.ticks_ms()
    tLoop=(tStop-tStart)*.001
    
    #Final PID calculations for thr direction
    dintergral = dintergral+ derror
    freq = dkp*derror + dkd*dderivative + dki*dintergral
    derror = yaw - dtarget
    #if depth > tdepth:
    #Acc1(255)
    #Acc2(255)
    #Acc3(255)
    #Acc4(255)
    #Acc5(255)
    #Acc6(255)
    #Acc7(255)
    #Acc8(255)
    #Setting the constants for the amplitude calculations
    kl = 2*pi*(frequency+freq)
    kr = 2*pi*(frequency-freq)
    
    #Calculating the amplitude for each actuator at a given time
    A1 = amp*sin(h*x1-kl*t)
    A2 = amp*sin(h*x2-kl*t)
    A3 = amp*sin(h*x3-kl*t)
    A4 = amp*sin(h*x4-kl*t)
    A5 = amp*sin(h*x1-kr*t)
    A6 = amp*sin(h*x2-kr*t)
    A7 = amp*sin(h*x3-kr*t)
    A8 = amp*sin(h*x4-kr*t)
    
    #Setting the acutators positions from the above amplitude
    Acc1(A1)
    Acc2(A2)
    Acc3(A3)
    Acc4(A4)
    Acc5(A5)
    Acc6(A6)
    Acc7(A7)
    Acc8(A8)