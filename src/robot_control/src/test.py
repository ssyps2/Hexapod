import time
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb

def getEndEffectorPose():
    legs_joint_angle = (0, np.deg2rad(-12/2), np.deg2rad(-74/2))

    # d=offset, a=length, alpha, theta
    standard_DH_lambda = [ [0, 0.043, -np.pi/2, legs_joint_angle[0]],\
        [0, 0.073, np.pi, legs_joint_angle[1]], [0, 0.133, 0, legs_joint_angle[2]] ]

    # update DH parameter for each leg
    hex_legs = rtb.DHRobot([\
        rtb.RevoluteDH(a=standard_DH_lambda[0][1], alpha=standard_DH_lambda[0][2], offset=standard_DH_lambda[0][3]),\
        rtb.RevoluteDH(a=standard_DH_lambda[1][1], alpha=standard_DH_lambda[1][2], offset=standard_DH_lambda[1][3]),\
        rtb.RevoluteDH(a=standard_DH_lambda[2][1], alpha=standard_DH_lambda[2][2], offset=standard_DH_lambda[2][3])])

    leg_eePose2base_lambda = lambda tran_x,tran_y,rota_z: sm.SE3(tran_x,tran_y,0) * sm.SE3.Rz(rota_z)\
        * hex_legs.fkine(legs_joint_angle)
        
    pos3 = leg_eePose2base_lambda(-0.12, -0.06, np.pi-np.arctan2(0.06,0.12))
    print(pos3.t)
    
    # print(hex_legs.fkine(legs_joint_angle))
    # hex_legs.plot(legs_joint_angle)
    # time.sleep(60)

getEndEffectorPose()