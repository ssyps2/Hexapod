#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import ctypes
import pigpio   # should be BCM number

SERVO_FRAME_HEADER         = 0x55
SERVO_MOVE_TIME_WRITE      = 1
SERVO_MOVE_TIME_READ       = 2
SERVO_MOVE_TIME_WAIT_WRITE = 7
SERVO_MOVE_TIME_WAIT_READ  = 8
SERVO_MOVE_START           = 11
SERVO_MOVE_STOP            = 12
SERVO_ID_WRITE             = 13
SERVO_ID_READ              = 14
SERVO_ANGLE_OFFSET_ADJUST  = 17
SERVO_ANGLE_OFFSET_WRITE   = 18
SERVO_ANGLE_OFFSET_READ    = 19
SERVO_ANGLE_LIMIT_WRITE    = 20
SERVO_ANGLE_LIMIT_READ     = 21
SERVO_VIN_LIMIT_WRITE      = 22
SERVO_VIN_LIMIT_READ       = 23
SERVO_TEMP_MAX_LIMIT_WRITE = 24
SERVO_TEMP_MAX_LIMIT_READ  = 25
SERVO_TEMP_READ            = 26
SERVO_VIN_READ             = 27
SERVO_POS_READ             = 28
SERVO_OR_MOTOR_MODE_WRITE  = 29
SERVO_OR_MOTOR_MODE_READ   = 30
SERVO_LOAD_OR_UNLOAD_WRITE = 31
SERVO_LOAD_OR_UNLOAD_READ  = 32
SERVO_LED_CTRL_WRITE       = 33
SERVO_LED_CTRL_READ        = 34
SERVO_LED_ERROR_WRITE      = 35
SERVO_LED_ERROR_READ       = 36

rx_pin = 7  # physical pin
tx_pin = 13

pi = pigpio.pi()

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200

def portInit():
    pi.set_mode(rx_pin, OUTPUT)   # config RX_CON(GPIO17) to be OUTPUT 
    pi.write(rx_pin, 0)
    pi.set_mode(tx_pin, OUTPUT)   # config TX_CON(GPIO27) to be OUTPUT 
    pi.write(tx_pin, 1)

portInit()

def portWrite():
    pi.write(tx_pin, 1)    # pull up TX_CON(GPIO27)
    pi.write(rx_pin, 0)    # pull down RX_CON(GPIO17)

def portRead():
    pi.write(rx_pin, 1)    # pull up RX_CON(GPIO17)
    pi.write(tx_pin, 0)    # pull down TX_CON(GPIO27)

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    pi.write(rx_pin, 1)
    pi.write(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    sum = 0x00
    for b in buf:
        sum += b
    sum = sum - 0x55 - 0x55  # remove the two header(0x55)
    sum = ~sum  # inverse
    return sum & 0xff

## send write command to servo
def serial_servo_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    portWrite()
    buf = bytearray(b'\x55\x55')  # header
    buf.append(id)
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # bias
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])
    buf.append(checksum(buf))

    serialHandle.write(buf) # send out

## send read command to servo
def serial_servo_read_cmd(id=None, r_cmd=None):
    portWrite()
    buf = bytearray(b'\x55\x55')  # header
    buf.append(id)
    buf.append(3)  # length
    buf.append(r_cmd)
    buf.append(checksum(buf))
    serialHandle.write(buf)  # send
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    serialHandle.flushInput()
    portRead()
    time.sleep(0.005)   # wait for completed
    count = serialHandle.inWaiting()    # obtain the number of bytes in recv buffer
    if count != 0:  # if received message
        recv_data = serialHandle.read(count)
        # for i in recv_data:
        #     print('%#x' %ord(i))
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # convert to signed int
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            print(e)
    else:
        serialHandle.flushInput()  # clear recv buffer
        return None
