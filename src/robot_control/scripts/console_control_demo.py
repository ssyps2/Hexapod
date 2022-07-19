import kinematics
import time

hexapod = kinematics.hex_kine(ctrl_freq = 30)

hexapod.initHexapod()
print('init...')
time.sleep(2)

while True:
    # hexapod.cmdHexapodMove(vx=0.1, vy=0, rz=0, mode=kinematics.hex_mode_e().TRIPOD)
    joint_angle = hexapod.getJointAngle()
    print(joint_angle)
    time.sleep(1/30)