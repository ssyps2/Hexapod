import kinematics
import time
import Board

hexapod = kinematics.hex_kine(ctrl_freq = 20)

hexapod.initHexapod()
time.sleep(2)
print('init completed')

# while True:
#     hexapod.cmdHexapodMove(vx=0.1, vy=0, rz=0, mode=kinematics.hex_mode_e().TRIPOD)