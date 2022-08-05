#!/usr/bin/env python3
import pigpio
from BusServoCmd import *

pi = pigpio.pi()

def isInt(num):
    if int(num):
        return True
    else:
        return False

def setBuzzer(new_state):
    pi.set_mode(6, pigpio.OUTPUT)
    pi.write(6, new_state)

def setBusServoPulse(id, pulse, use_time):
    if isInt(id) and isInt(pulse) and isInt(use_time):
        pass
    else:
        raise Exception("Input param for setBusServoPulse should be int")

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_servo_wirte_cmd(id, SERVO_MOVE_TIME_WRITE, pulse, use_time)

def getBusServoPulse(id):
    while True:
        serial_servo_read_cmd(id, SERVO_POS_READ)
        msg = serial_servo_get_rmsg(SERVO_POS_READ)
        if msg is not None:
            return msg
