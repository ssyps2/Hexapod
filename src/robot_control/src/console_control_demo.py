import kinematics
import time

hexapod = kinematics.hex_kine(ctrl_freq = 30)

hexapod.initHexapod()
time.sleep(2)

while True:
    # hexapod.cmdHexapodMove(0.1, 0, 0, kinematics.hex_mode_e().TRIPOD)
    joint_angle = hexapod.getJointAngle()
    print(joint_angle)
    time.sleep(1/30)