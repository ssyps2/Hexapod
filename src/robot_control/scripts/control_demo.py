import kinematics
import time
from threading import Thread

hexapod = kinematics.hex_kine(ctrl_freq = 12)

hexapod.initHexapod()
time_now = time.time()

while True:
    try:
        read_thread = Thread(target=hexapod.getJointAngle)
        read_thread.start()
        print(read_thread.is_alive())

        last_time = time_now
        hexapod.cmdHexapodMove(0.1,0,0,kinematics.hex_mode_e.TRIPOD)
        time_now = time.time()
        print("task_freq:", 1/(time_now-last_time))
    except Exception:
        pass
