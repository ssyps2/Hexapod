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

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

def startHexapod():
    while True:
        time.sleep(0.03)

        while True:
            try:
                if 
            except BaseException as e:
                print(e)
                break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startHexapod()

# 主线程，已经以后台的形式开机自启
# 自启方式systemd，自启文件/etc/systemd/system/spiderpi.service
# sudo systemctl stop spiderpi  此次关闭
# sudo systemctl disable spiderpi 永久关闭
# sudo systemctl enable spiderpi 永久开启
# sudo systemctl start spiderpi 此次开启