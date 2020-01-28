#from lib.mpu6050.mpu6050 import MPU
from machine import Pin
import mpu6050 as mpu6050
mpu=mpu6050.MPU()
#mpu.calibrate()
MPU6050_RA_INT_STATUS = 0x3A
MPU6050_RA_INT_ENABLE = 0x38
MPU6050_RA_INT_PIN_CFG = 0x37
mpu.write_byte(MPU6050_RA_INT_ENABLE,mpu.read_byte(MPU6050_RA_INT_ENABLE) | 0x19)
mpu.write_byte(MPU6050_RA_INT_PIN_CFG,0x20)
print(bin(mpu.read_byte(MPU6050_RA_INT_ENABLE)))
print(bin(mpu.read_byte(MPU6050_RA_INT_PIN_CFG)))
p=Pin("P13", mode=Pin.IN)
i=0
while True:
    a=mpu.read_byte(MPU6050_RA_INT_STATUS)
    if a != 0x00 and i == 0:
        i=1
    if 1 > 0:
        if i > 10:
            break
        else:
            i +=1
