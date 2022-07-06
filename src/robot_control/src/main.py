#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import time
import logging
import numpy as np
import kinematics

ik = kinematics.IK()

if sys.version_info.major == 2:     # check python version
    print('Please run this program with python3!')
    sys.exit(0)

def startHexapod():
    while True:
        time.sleep(0.03)

        while True:
            try:
                pass
            except BaseException as e:
                print(e)
                break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startHexapod()
