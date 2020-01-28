from machine import Pin
import mpu6050 as mpu6050
mpu=mpu6050.MPU()
import time



def LowPowerAccelOnlyMPU6050_2():
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) & ~0x60)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_2 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_2) & ~0x38)
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG ,mpu.read_byte(MPU6050_RA_ACCEL_CONFIG) & ~0x07)
    mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) & ~0x07)
    mpu.write_byte(MPU6050_RA_INT_ENABLE ,0x40)
    mpu.write_byte(MPU6050_RA_MOT_DUR, 0x01) # Set motion detect duration to 1  ms LSB is 1 ms @ 1 kHz rate
    mpu.write_byte(MPU6050_RA_MOT_THR, 0x14) # Set motion detection to 0.256 g LSB = 2 mg
    time.sleep_ms(100)  # Add delay for accumulation of samples
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG ,mpu.read_byte(MPU6050_RA_ACCEL_CONFIG) | 0x07)
    mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) & ~0xC7)
    mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) | 0x40 | 0x07)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) & ~0x48)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) | 0x20)

def LowPowerAccelOnlyMPU6050():
    # The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
    # Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
    # above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
    # threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
    # consideration for these threshold evaluations otherwise, the flags would be set all the time!

    c = mpu.read_byte(MPU6050_RA_PWR_MGMT_1)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c & ~0x30) # Clear sleep and cycle bits [5:6]
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c |  0x30) # Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = mpu.read_byte(MPU6050_RA_PWR_MGMT_2)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c & ~0x38) # Clear standby XA, YA, and ZA bits [3:5]
    mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c |  0x00) # Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = mpu.read_byte(MPU6050_RA_ACCEL_CONFIG)
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c & ~0x07) # Clear high-pass filter bits [2:0]
    # Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG,  c | 0x00)  # Set ACCEL_HPF to 0 reset mode disbaling high-pass filter

    c = mpu.read_byte(MPU6050_RA_CONFIG)
    mpu.write_byte(MPU6050_RA_CONFIG, c & ~0x07) # Clear low-pass filter bits [2:0]
    mpu.write_byte(MPU6050_RA_CONFIG, c |  0x00)  # Set DLPD_CFG to 0 260 Hz bandwidth, 1 kHz rate

    c = mpu.read_byte(MPU6050_RA_INT_ENABLE)
    mpu.write_byte(MPU6050_RA_INT_ENABLE, c & ~0xFF)  # Clear all interrupts
    mpu.write_byte(MPU6050_RA_INT_ENABLE, 0x40)  # Enable motion threshold (bits 5) interrupt only

    # Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    # for at least the counter duration
    mpu.write_byte(MPU6050_RA_MOT_THR, 0x80) # Set motion detection to 0.256 g LSB = 2 mg
    mpu.write_byte(MPU6050_RA_MOT_DUR, 0x01) # Set motion detect duration to 1  ms LSB is 1 ms @ 1 kHz rate

    time.sleep_ms(100)  # Add delay for accumulation of samples

    c = mpu.read_byte(MPU6050_RA_ACCEL_CONFIG)
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c & ~0x07) # Clear high-pass filter bits [2:0]
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c |  0x07)  # Set ACCEL_HPF to 7 hold the initial accleration value as a referance

    c = mpu.read_byte(MPU6050_RA_PWR_MGMT_2)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c & ~0xC7) # Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    mpu.write_byte(MPU6050_RA_PWR_MGMT_2, c |  0x47) # Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

    c = mpu.read_byte(MPU6050_RA_PWR_MGMT_1)
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c & ~0x20) # Clear sleep and cycle bit 5
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, c |  0x20) # Set cycle bit 5 to begin low power accelerometer motion interrupts

def initMPU6050():
    # Initialize MPU6050 device

    #  wake up device-don't need this here if using calibration function below
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, 0x00)  # Clear sleep mode bit (6), enable all sensors
    time.sleep_ms(100)  # Delay 100 ms for PLL to get established on x-axis gyro  should check for PLL ready interrupt

    # get stable time source
    mpu.write_byte(MPU6050_RA_PWR_MGMT_1, 0x01)   # Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    # Configure Gyro and Accelerometer
    # Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively
    # DLPF_CFG = bits 2:0 = 010  this sets the sample rate at 1 kHz for both
    mpu.write_byte(MPU6050_RA_CONFIG, 0x03)

    # Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    mpu.write_byte(MPU6050_RA_SMPLRT_DIV, 0x04)   # Use a 200 Hz sample rate

    # Set gyroscope full scale range
    # Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    c =  mpu.read_byte(MPU6050_RA_GYRO_CONFIG)
    Gscale=0
    mpu.write_byte(MPU6050_RA_GYRO_CONFIG, c & ~0xE0)  # Clear self-test bits [7:5]
    mpu.write_byte(MPU6050_RA_GYRO_CONFIG, c & ~0x18)  # Clear AFS bits [4:3]
    mpu.write_byte(MPU6050_RA_GYRO_CONFIG, c | Gscale << 3)  # Set full scale range for the gyro

    # Set accelerometer configuration
    c =  mpu.read_byte(MPU6050_RA_ACCEL_CONFIG)
    Ascale=0
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c & ~0xE0)  # Clear self-test bits [7:5]
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c & ~0x18)  # Clear AFS bits [4:3]
    mpu.write_byte(MPU6050_RA_ACCEL_CONFIG, c | Ascale << 3)  # Set full scale range for the accelerometer

    # Configure Interrupts and Bypass Enable
    # Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    # can join the I2C bus and all can be controlled by the Arduino as master
    mpu.write_byte(MPU6050_RA_INT_PIN_CFG, 0x02)
    mpu.write_byte(MPU6050_RA_INT_ENABLE, 0x01)   # Enable data ready (bit 0) interrupt

def testInter(arg):
     print("got an interrupt in pin %s" % (arg.id()))
     a=mpu.read_byte(MPU6050_RA_INT_STATUS)

mpu.reset()

mpu.calibrate()
LowPowerAccelOnlyMPU6050_2()
#initMPU6050()

p=Pin("P13", mode=Pin.IN)
p.callback(trigger=Pin.IRQ_RISING, handler=testInter)
a=mpu.read_byte(MPU6050_RA_INT_STATUS)

while True:
    b=p.value()
    #a=mpu.read_byte(MPU6050_RA_INT_STATUS)
    #print(mpu.temperature, mpu.acceleration, mpu.gyro)
    print(b)
    time.sleep(1)
