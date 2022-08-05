#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import ctypes
import pigpio   # should be BCM number

SERVO_FRAME_HEADER         = 0x55
SERVO_MOVE_TIME_WRITE      = 1
SERVO_POS_READ             = 28

rx_pin = 4  # physical pin 7, BCM 4
tx_pin = 27 # physical pin 13, BCM 27

pi = pigpio.pi()

serialHandle = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.5)  # init serial port with baud rate 115200

def portInit():
    pi.set_mode(rx_pin, pigpio.OUTPUT)   # config RX_CON to be OUTPUT 
    pi.write(rx_pin, 1)
    pi.set_mode(tx_pin, pigpio.OUTPUT)   # config TX_CON to be OUTPUT 
    pi.write(tx_pin, 1)

portInit()

def portWrite():
    pi.write(tx_pin, 1)    # pull up TX_CON
    pi.write(rx_pin, 1)    # pull down RX_CON

def portRead():
    pi.write(rx_pin, 1)    # pull up RX_CON
    pi.write(tx_pin, 0)    # pull down TX_CON

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
    time.sleep(0.001) #0.00034

def serial_servo_get_rmsg(cmd):
    portRead()

    time.sleep(0.001)
    count = serialHandle.inWaiting()    # obtain the number of bytes in recv buffer
    # print('obtained data length: ', count)

    recv_data = serialHandle.read(count)
    # print('received raw data: ', recv_data)

    try:
        if count < 8:
            return None
        for index in range(count-7):
            if recv_data[index] == 0x55 and recv_data[index+1] == 0x55 and\
            recv_data[index+3] == 0x05 and recv_data[index+4] == cmd:
                pos = 0xffff & (recv_data[index+5] | (0xff00 & (recv_data[index+6] << 8)))
                return ctypes.c_int16(pos).value
            else:
                index += 1
        return None
    except BaseException as e:
        print("servo_read_error:", e)

