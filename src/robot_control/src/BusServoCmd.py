#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import ctypes

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

serialHandle = serial.Serial("/dev/ttyUSB0", 115200)

def checksum(buf):
    sum = 0x00
    for b in buf:
        sum += b
    sum = sum - 0x55 - 0x55  # remove the two header (0x55)
    sum = ~sum  # reverse
    return sum & 0xff

def serial_servo_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    write
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    buf = bytearray(b'\x55\x55')  # header
    buf.append(id)
    # command length
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # write command
    
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # bias
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])
    
    buf.append(checksum(buf))
    
    serialHandle.write(buf)  # send

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    read
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    buf = bytearray(b'\x55\x55')
    buf.append(id)
    buf.append(3)  # cmd length
    buf.append(r_cmd)  # read command
    buf.append(checksum(buf))
    serialHandle.write(buf)  # send
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    '''
    # 获取指定读取命令的数据
    :param cmd: 读取命令
    :return: 数据
    '''
    serialHandle.flushInput()  # 清空接收缓存
    # portRead()  # 将单线串口配置为输入
    time.sleep(0.005)  # 稍作延时，等待接收完毕
    count = serialHandle.inWaiting()    # 获取接收缓存中的字节数
    if count != 0:  # 如果接收到的数据不空
        recv_data = serialHandle.read(count)  # 读取接收到的数据
        # for i in recv_data:
        #     print('%#x' %ord(i))
        # 是否是读id指令
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # 清空接收缓存
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # 转换成有符号整型
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
        serialHandle.flushInput()  # 清空接收缓存
        return None
