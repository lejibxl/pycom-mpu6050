#from lib.mpu6050.mpu6050 import MPU
from machine import Pin
import mpu6050 as mpu6050
mpu=mpu6050.MPU()
import time

def testInter(arg):
     print("got an interrupt in pin %s" % (arg.id()))
     a=mpu.read_byte(MPU6050_RA_INT_STATUS)


mpu.reset()

mpu.write_byte(MPU6050_RA_INT_PIN_CFG,0xA0)
mpu.write_byte(MPU6050_RA_MOT_DETECT_CTRL,0x15)
mpu.write_byte(MPU6050_RA_ACCEL_CONFIG,0x07)
mpu.write_byte(MPU6050_RA_MOT_THR,0x80)
mpu.write_byte(MPU6050_RA_MOT_DUR,0x01)
#mpu.write_byte(MPU6050_RA_INT_ENABLE,mpu.read_byte(MPU6050_RA_INT_ENABLE) | 0x19)
mpu.write_byte(MPU6050_RA_INT_ENABLE,mpu.read_byte(MPU6050_RA_INT_ENABLE) | 0x40)
#mpu.write_byte(MPU6050_RA_INT_ENABLE,mpu.read_byte(MPU6050_RA_INT_ENABLE) | 0x01)

c = mpu.read_byte(MPU6050_RA_PWR_MGMT_2);
mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c & ~0xC7); # Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c |  0x47); # Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

c = mpu.read_byte(MPU6050_RA_PWR_MGMT_1);
mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c & ~0x20); # Clear sleep and cycle bit 5
mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c |  0x20); # Set cycle bit 5 to begin low power accelerometer motion interrupts

p=Pin("P13", mode=Pin.IN)
p.callback(trigger=Pin.IRQ_FALLING, handler=testInter)
a=mpu.read_byte(MPU6050_RA_INT_STATUS)
while True:
    b=p.value()
    #a=mpu.read_byte(MPU6050_RA_INT_STATUS)
    #print(mpu.temperature, mpu.acceleration, mpu.gyro)
    print(b)
    time.sleep(1)
