import gc
from machine import Pin, I2C, PWM
import time
import micropython
from ustruct import unpack
import logging.logging as logging

from mpu6050.mpu6050Const import *
import mpu6050.mpu6050Cfilter as cfilter

l = logging.getLogger(__name__)
micropython.alloc_emergency_exception_buf(100)

default_pin_scl = 'P10'
default_pin_sda = 'P9'
default_pin_intr = 'P13'
default_pin_led = None
default_sample_rate = 0x20

default_calibration_numsamples = 200
default_calibration_accel_deadzone = 15
default_calibration_gyro_deadzone = 5

accel_range = [2, 4, 8, 16]
gyro_range = [250, 500, 1000, 2000]

# These are what the sensors ought to read at rest
# on a level surface
expected = [0, 0, 16384, None, 0, 0, 0]

class CalibrationFailure(Exception):
    pass

class MPU(object):
    def __init__(self, scl=None, sda=None,
                 intr=None, led=None, rate=None,
                 address=None):

        self.scl = scl if scl is not None else default_pin_scl
        self.sda = sda if sda is not None else default_pin_sda
        self.intr = intr if intr is not None else default_pin_intr
        self.led = led if led is not None else default_pin_led
        self.rate = rate if rate is not None else default_sample_rate
        self.WOM_handler = None

        self.address = address if address else MPU6050_DEFAULT_ADDRESS

        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(14)

        self.calibration = [0] * 7

        self.filter = cfilter.ComplementaryFilter()

        self.init_pins()
        self.init_led()
        self.init_i2c()
        self.init_device()


    def write_byte(self, reg, val):
        self.bus.writeto_mem(self.address, reg, bytes([val]))

    def read_byte(self, reg):
        buf=bytearray(1)
        self.bus.readfrom_mem_into(self.address, reg, buf)
        return buf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2**length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def read_word(self, reg):
        buf=bytearray(2)
        self.bus.readfrom_mem_into(self.address, reg, buf)
        return unpack('>h', buf)[0]

    def init_i2c(self):
        l.debug('* initializing i2c')
        self.bus = I2C(0, mode=I2C.MASTER, pins=(self.pin_sda,self.pin_scl))

    def init_pins(self):
        l.debug('* initializing pins')
        self.pin_sda = Pin(self.sda)
        self.pin_scl = Pin(self.scl)
        self.pin_intr = Pin(self.intr, mode=Pin.IN)
        #self.pin_led = PWM(Pin(self.led, mode=Pin.OUT))

    def set_state_uncalibrated(self):
        #self.pin_led.freq(1)
        #self.pin_led.duty(500)
        pass

    def set_state_calibrating(self):
        #self.pin_led.freq(10)
        #self.pin_led.duty(500)
        pass

    def set_state_calibrated(self):
        #self.pin_led.freq(1000)
        #self.pin_led.duty(500)
        pass

    def set_state_disabled(self):
        #self.pin_led.duty(0)
        pass

    def init_led(self):
        #self.set_state_uncalibrated()
        pass

    def identify(self):
        l.debug('* identifying i2c device')
        val = self.read_byte(MPU6050_RA_WHO_AM_I)
        if val != MPU6050_ADDRESS_AD0_LOW:
            raise OSError("No mpu6050 at address {}".format(self.address))

    def reset(self):
        l.debug('* reset')
        '''
        self.write_byte(MPU6050_RA_PWR_MGMT_1, (
            (1 << MPU6050_PWR1_DEVICE_RESET_BIT)
        ))
        time.sleep_ms(100)
        '''
        self.write_byte(MPU6050_RA_SIGNAL_PATH_RESET, (
            (1 << MPU6050_PATHRESET_GYRO_RESET_BIT) |
            (1 << MPU6050_PATHRESET_ACCEL_RESET_BIT) |
            (1 << MPU6050_PATHRESET_TEMP_RESET_BIT)
        ))
        time.sleep_ms(100)

    def init_device(self):
        l.debug('* initializing mpu')

        self.identify()

        # disable sleep mode and select clock source
        self.write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO)

        # enable all sensors
        self.write_byte(MPU6050_RA_PWR_MGMT_2, 0)

        # set sampling rate
        self.write_byte(MPU6050_RA_SMPLRT_DIV, self.rate)

        # enable dlpf
        self.write_byte(MPU6050_RA_CONFIG, 1)

        # explicitly set accel/gyro range
        self.set_accel_range(MPU6050_ACCEL_FS_2)
        self.set_gyro_range(MPU6050_GYRO_FS_250)

    def set_gyro_range(self, fsr):
        self.gyro_range = gyro_range[fsr]
        self.set_bitfield(MPU6050_RA_GYRO_CONFIG,
                          MPU6050_GCONFIG_FS_SEL_BIT,
                          MPU6050_GCONFIG_FS_SEL_LENGTH,
                          fsr)

    def set_accel_range(self, fsr):
        self.accel_range = accel_range[fsr]
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_AFS_SEL_BIT,
                          MPU6050_ACONFIG_AFS_SEL_LENGTH,
                          fsr)

    def read_sensors(self):
        self.bus.readfrom_mem_into(self.address,
                                   MPU6050_RA_ACCEL_XOUT_H,
                                   self.sensors)

        data = unpack('>hhhhhhh', self.sensors)

        # apply calibration values
        return [data[i] + self.calibration[i] for i in range(7)]

    def read_sensors_scaled(self):
        data = self.read_sensors()
        data[0:3] = [x/(65536//self.accel_range//2) for x in data[0:3]]
        data[4:7] = [x/(65536//self.gyro_range//2) for x in data[4:7]]
        return data

    def read_position(self):
        self.filter.input(self.read_sensors_scaled())
        return [
            self.filter.filter_pos,
            self.filter.accel_pos,
            self.filter.gyro_pos,
        ]

    def set_dhpf_mode(self, bandwidth):
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_ACCEL_HPF_BIT,
                          MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
                          bandwidth)

    def set_motion_detection_threshold(self, threshold):
        self.write_byte(MPU6050_RA_MOT_THR, threshold)

    def set_motion_detection_duration(self, duration):
        self.write_byte(MPU6050_RA_MOT_DUR, duration)

    def set_int_motion_enabled(self, enabled):
        self.set_bitfield(MPU6050_RA_INT_ENABLE,
                          MPU6050_INTERRUPT_MOT_BIT,
                          1,
                          enabled)

    def get_sensor_avg(self, samples, softstart=100):
        '''Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle.'''
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return [x//samples for x in counters]

    stable_reading_timeout = 10
    max_gyro_variance = 5


    def wait_for_stable(self, numsamples=10):
        l.debug('* waiting for gyros to stabilize')

        gc.collect()
        time_start = time.time()
        samples = []

        while True:
            now = time.time()
            if now - time_start > self.stable_reading_timeout:
                raise CalibrationFailure()

            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            samples.append(sample[4:7])
            if len(samples) < numsamples:
                continue

            samples = samples[-numsamples:]

            totals = [0] * 3
            for cola, colb in zip(samples, samples[1:]):
                deltas = [abs(a-b) for a,b in zip(cola, colb)]
                totals = [a+b for a,b in zip(deltas, totals)]

            avg = [a/numsamples for a in totals]
            l.debug("* delta = {}".format(avg))
            if all(x < self.max_gyro_variance for x in avg):
                break

        now = time.time()
        l.debug('* gyros stable after {:0.2f} seconds'.format(now-time_start))

    def calibrate(self,
                  numsamples=None,
                  accel_deadzone=None,
                  gyro_deadzone=None):

        old_calibration = self.calibration
        self.calibration = [0] * 7

        numsamples = (numsamples if numsamples is not None
                   else default_calibration_numsamples)
        accel_deadzone = (accel_deadzone if accel_deadzone is not None
                          else default_calibration_accel_deadzone)
        gyro_deadzone = (gyro_deadzone if gyro_deadzone is not None
                         else default_calibration_gyro_deadzone)

        l.debug('* start calibration')
        self.set_state_calibrating()

        try:
            self.wait_for_stable()
            gc.collect()

            # calculate offsets between the expected values and
            # the average value for each sensor reading
            avg = self.get_sensor_avg(numsamples)
            off = [0 if expected[i] is None else expected[i] - avg[i]
                   for i in range(7)]

            accel_ready = False
            gyro_read = False
            for passno in range(20):
                self.calibration = off
                avg = self.get_sensor_avg(numsamples)

                check = [0 if expected[i] is None else expected[i] - avg[i]
                       for i in range(7)]
                l.debug('- pass {}: {}'.format(passno, check))

                # check if current values are within acceptable offsets
                # from the expected values
                accel_ready = all(abs(x) < accel_deadzone
                                  for x in check[0:3])
                gyro_ready = all(abs(x) < gyro_deadzone
                                 for x in check[4:7])

                if accel_ready and gyro_ready:
                    break

                if not accel_ready:
                    off[0:3] = [off[i] + check[i]//accel_deadzone
                                for i in range(3)]

                if not gyro_ready:
                    off[4:7] = [off[i] + check[i]//gyro_deadzone
                                for i in range(4, 7)]
            else:
                raise CalibrationFailure()
        except CalibrationFailure:
            self.calibration = old_calibration
            l.info('! calibration failed')
            self.set_state_uncalibrated()
            return

        l.info('* calibrated!')
        self.set_state_calibrated()

    def _WakeOnMotionINT(self,pin_o):
        l.debug("interrupt ok")
        l.info("got an interrupt in pin %s" % (pin_o.id()))
        a=mpu.read_byte(MPU6050_RA_INT_STATUS) #Reset Interrupts
        if self.WOM_handler is not None:
            self.WOM_handler(pin_o)

    def WakeOnMotion(self, threshold=0x14, duration=0x01, handler=None):
        # see doc : http://www.4tronix.co.uk/arduino/specs/mpu6050.pdf pg 33
        # The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
        # Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
        # above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
        # threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
        # consideration for these threshold evaluations otherwise, the flags would be set all the time!

        self.WOM_handler = handler
        #self.pin_intr.callback(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._WakeOnMotionINT)
        self.pin_intr.callback(trigger=Pin.IRQ_RISING, handler=self._WakeOnMotionINT)
        a=mpu.read_byte(MPU6050_RA_INT_STATUS)
        #INT_OPEN When this bit is equal to 0, the INT pin is configured as push-pull.
        #LATCH_INT_EN When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
        mpu.write_byte(MPU6050_RA_INT_PIN_CFG ,mpu.read_byte(MPU6050_RA_INT_PIN_CFG) & ~0xF8)
        mpu.write_byte(MPU6050_RA_INT_PIN_CFG ,mpu.read_byte(MPU6050_RA_INT_PIN_CFG) | 0x20)
        #Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running
        mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) & ~0x60)
        #Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
        mpu.write_byte(MPU6050_RA_PWR_MGMT_2 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_2) & ~0x38)
        #Set ACCEL_HPF to 0 reset mode disbaling high-pass filter
        mpu.write_byte(MPU6050_RA_ACCEL_CONFIG ,mpu.read_byte(MPU6050_RA_ACCEL_CONFIG) & ~0x07)
        #Set DLPD_CFG to 0 260 Hz bandwidth, 1 kHz rate
        mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) & ~0x07)
        #Enable motion threshold (bits 5) interrupt only
        mpu.write_byte(MPU6050_RA_INT_ENABLE ,0x40)
        #Set motion detect duration to 1  ms LSB is 1 ms @ 1 kHz rate
        mpu.write_byte(MPU6050_RA_MOT_DUR, duration) # Set motion detect duration to 1  ms LSB is 1 ms @ 1 kHz rate
        # Set motion detection to 20 LSB
        mpu.write_byte(MPU6050_RA_MOT_THR, threshold) #
        # Add delay for accumulation of samples
        time.sleep_ms(100)
        #Set ACCEL_HPF to 7 hold the initial accleration value as a referance
        mpu.write_byte(MPU6050_RA_ACCEL_CONFIG ,mpu.read_byte(MPU6050_RA_ACCEL_CONFIG) | 0x07)
        #Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])
        mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) & ~0xC7)
        mpu.write_byte(MPU6050_RA_CONFIG ,mpu.read_byte(MPU6050_RA_CONFIG) | 0x40 | 0x07)
        #Set cycle bit 5 to begin low power accelerometer motion interrupts
        mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) & ~0x48)
        mpu.write_byte(MPU6050_RA_PWR_MGMT_1 ,mpu.read_byte(MPU6050_RA_PWR_MGMT_1) | 0x20)

    def enable_activity_interrupt(self, threshold, duration, handler=None):
        # Threshold is in mg, duration is ms
        # A faire
        mpu.write_byte(MPU6050_RA_INT_PIN_CFG,0x20) #write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.set_dhpf_mode(MPU6050_DHPF_5)
        self.set_motion_detection_threshold(threshold)
        self.set_motion_detection_duration(duration)
        #self.set_int_motion_enabled(1)
        self.set_bitfield(MPU6050_RA_INT_ENABLE,
                          MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
                          1,
                          1)
        self._user_handler = handler
        self.pin_intr.callback(trigger=Pin.IRQ_RISING, handler=self._user_handler)
        a=mpu.read_byte(MPU6050_RA_INT_STATUS)

    @property
    def temperature(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        # Get the raw data
        raw_temp = self.read_word(MPU6050_RA_TEMP_OUT_H)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340) + 36.53

        # Return the temperature
        return actual_temp

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        raw_data= bytearray(6)
        self.bus.readfrom_mem_into(self.address, MPU6050_RA_GYRO_XOUT_H , raw_data)
        raw=unpack('>hhh',raw_data)
        # setup range dependant scaling
        gyro_x = raw[0]/(65536//self.gyro_range//2)
        gyro_y = raw[1]/(65536//self.gyro_range//2)
        gyro_z = raw[2]/(65536//self.gyro_range//2)

        return (gyro_x, gyro_y, gyro_z)

    @property
    def acceleration(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        raw_data= bytearray(6)
        self.bus.readfrom_mem_into(self.address, MPU6050_RA_ACCEL_XOUT_H , raw_data)
        raw=unpack('>hhh',raw_data)
        # setup range dependant scaling
        accel_x = raw[0]/(65536//self.accel_range//2)
        accel_y  = raw[1]/(65536//self.accel_range//2)
        accel_z  = raw[2]/(65536//self.accel_range//2)

        return (accel_x, accel_y, accel_z)

def testInter(arg):
     print("gooot an interrupt in pin %s" % (arg.id()))


if __name__ == '__main__':
    import machine

    # get the wake reason and the value of the pins during wake up
    wake_s = machine.wake_reason()
    print(wake_s)

    mpu=MPU()
    mpu.reset()
    mpu.WakeOnMotion(threshold=0x14,duration=1,handler=testInter)
    machine.pin_sleep_wakeup(['P13'], machine.WAKEUP_ANY_HIGH, False)
    machine.deepsleep(10*1000)
