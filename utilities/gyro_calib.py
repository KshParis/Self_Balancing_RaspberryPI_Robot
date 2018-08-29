# Utility to identify offset value for your sensor from a trail of 1000 + runs
# The offset hugely varies from sensor to sensor
#
#
import smbus  # import SMBus module of I2C
import time

from mpu6050 import mpu6050
import math

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

# some MPU6050 Registers and their Address
PWR_MGMT     = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47


# ///////////////////////////////   CONFIGURATION   /////////////////////////

mean_ax = 0
mean_ay = 0
mean_az = 0
mean_gx = 0
mean_gy = 0
mean_gz = 0
acel_deadzone = 8  # Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
giro_deadzone = 1  # Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
buffersize = 1100  # Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)

def calib_sensor():
    print("Mean offset value calculation in progres (wait time ~1 minute)...")
    buff_acc_x = 0
    buff_acc_y = 0
    buff_acc_z = 0
    buff_gyro_x = 0
    buff_gyro_y = 0
    buff_gyro_z = 0

    print("Buffer values initialized.." + str(buffersize))

    for i in range(buffersize):
        # print("progress : " + str("#")*i)
        # read raw accel/gyro measurements from device
        # Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        buff_acc_x = buff_acc_x + acc_x
        buff_acc_y = buff_acc_y + acc_y
        buff_acc_z = buff_acc_z + acc_z

        buff_gyro_x = buff_gyro_x + gyro_x
        buff_gyro_y = buff_gyro_y + gyro_y
        buff_gyro_z = buff_gyro_z + gyro_z

        if i == 1099:
            print("condition true")
            mean_ax = buff_acc_x / i
            mean_ay = buff_acc_y / i
            mean_az = buff_acc_z / i
            mean_gx = buff_gyro_x / i
            mean_gy = buff_gyro_y / i
            mean_gz = buff_gyro_z / i

            print('Mean_AX :' + str(mean_ax))
            print('Mean_AY :' + str(mean_ay))
            print('Mean_AZ :' + str(mean_az))
            print('Mean GX :' + str(mean_gx))
            print('Mean GY :' + str(mean_gy))
            print('Mean GZ :' + str(mean_gz))

        time.sleep(0.02)

    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    if (abs(mean_ax) <= acel_deadzone):
        print("mean less than deadzone " + str(mean_ax))
    else:
        ax_offset = ax_offset - mean_ax / acel_deadzone
        print("Ax_offset value :" + str(ax_offset))

    if (abs(mean_ay) <= acel_deadzone):
        print("mean less than deadzone " + str(mean_ay))
    else:
        ay_offset = ay_offset - mean_ay / acel_deadzone
        print("ay_offset value :" + str(ay_offset))

    if (abs(16384 - mean_az) <= acel_deadzone):
        print("mean less than deadzone " + str(16384 - mean_az))
    else:
        az_offset = az_offset + (16384 - mean_az) / acel_deadzone
        print("az_offset value :" + str(az_offset))

    if (abs(mean_gx) <= giro_deadzone):
        print("mean less than deadzone " + str(mean_gx))
    else:
        gx_offset = gx_offset - mean_gx / (giro_deadzone + 1)
        print("gx_offset value :" + str(gx_offset))

    if (abs(mean_gy) <= giro_deadzone):
        print("mean less than deadzone " + str(mean_gy))
    else:
        gy_offset = gy_offset - mean_gy / (giro_deadzone + 1)
        print("gy_offset value :" + str(gy_offset))

    if (abs(mean_gz) <= giro_deadzone):
        print("mean less than deadzone " + str(mean_gz))
    else:
        gz_offset = gz_offset - mean_gz / (giro_deadzone + 1)
        print("gz_offset value :" + str(gz_offset))

    print("\nData is printed as: acelX acelY acelZ giroX giroY giroZ")
    print("Check that your sensor readings are close to 0 0 16384 0 0 0")


# Code ends


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    print("MPU Initialized..")


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value

def start_reading():
    # this function gives you realtime reading from the sensor
    MPU_Init()

    while True:
        # Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz,
              u'\u00b0' + "/s", "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)
    sleep(1)


def prog_offset_values():

    sensor = mpu6050(0x68)
    # K and K1 --> Constants used with the complementary filter
    K = 0.98
    K1 = 1 - K

    time_diff = 0.02
    ITerm = 0

    # Calling the MPU6050 data
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    aTempX = accel_data['x']
    aTempY = accel_data['y']
    aTempZ = accel_data['z']

    gTempX = gyro_data['x']
    gTempY = gyro_data['y']
    gTempZ = gyro_data['z']

    # some math
    def distance(a, b):
        return math.sqrt((a * a) + (b * b))

    def y_rotation(x, y, z):
        radians = math.atan2(x, distance(y, z))
        return -math.degrees(radians)

    def x_rotation(x, y, z):
        radians = math.atan2(y, distance(x, z))
        return math.degrees(radians)

    last_x = x_rotation(aTempX, aTempY, aTempZ)
    last_y = y_rotation(aTempX, aTempY, aTempZ)

    gyro_offset_x = gTempX
    gyro_offset_y = gTempY

    print('(single execution)gx_offset :' + str(gyro_offset_x))
    print('(single execution)gy_offset :' + str(gyro_offset_y))

    gyro_total_x = (last_x) - gyro_offset_x
    gyro_total_y = (last_y) - gyro_offset_y

    print('(single execution)Total gyro value (x) : ' + str(gyro_total_x))
    print('(single execution)Total gyro value (y) : ' + str(gyro_total_y))

if __name__ == '__main__':

    MPU_Init()
    calib_sensor()
    
    #start_reading()
    #realtime sensor feed for gyro and accelerometer data.
    
    #Alternate method to identify offset values
    #prog_offset_values()
