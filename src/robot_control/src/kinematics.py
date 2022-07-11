import time

from sympy import rot_axis3
import Board
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb

# constants define
TASK_FREQ = 500 # task frequency in Hz
SERVO_ANGLE_RANGE = 240 # in degree
SERVO_MAX_PULSE = 1000
SERVO_HALF_PULSE = SERVO_MAX_PULSE/2  # 500
PULSE2ANGLE = SERVO_ANGLE_RANGE/SERVO_MAX_PULSE  # 0.24, in degree

## init joint servo ID in legs
leg_ID_lambda = lambda id: (id,id+1,id+2)
#               leg1            leg2             leg3              leg4             leg5              leg6
legs_ID = (leg_ID_lambda(7),leg_ID_lambda(4),leg_ID_lambda(1),leg_ID_lambda(10),leg_ID_lambda(13),leg_ID_lambda(16))

## feedback angle by servo in rad (for IK)
def getJointAngle():
    """
    :retval: legs_joint_angle[legNum][jointID]
    """
    pulse2angle_lambda = lambda pulse: (pulse-SERVO_HALF_PULSE)*PULSE2ANGLE # in degree
    leg_joint_angle_lambda = lambda i: [pulse2angle_lambda(Board.getBusServoPulse(legs_ID[i][0])),\
                                        -pulse2angle_lambda(Board.getBusServoPulse(legs_ID[i][1])),\
                                        pulse2angle_lambda(Board.getBusServoPulse(legs_ID[i][2]))]

    # from leg1 to leg6, leg3~6 was inversed because of assembling
    legs_joint_angle = [ leg_joint_angle_lambda(0), leg_joint_angle_lambda(1), leg_joint_angle_lambda(2),\
                        -leg_joint_angle_lambda(3), -leg_joint_angle_lambda(4), -leg_joint_angle_lambda(5) ]

    return legs_joint_angle

## DH paramter for each leg: offset(i), a(i), alpha(i), theta(i)
def updateStandardDH():
    legs_joint_angle = getJointAngle()

    standard_DH_lambda = lambda legNum: [ [0, 0.043, -np.pi/2, legs_joint_angle[legNum][0]],\
        [0, 0.073, np.pi, legs_joint_angle[legNum][1]], [0, 0.133, 0, legs_joint_angle[legNum][2]] ]

    hexapod_leg1 = rtb.DHRobot([rtb.RevoluteDH(standard_DH_lambda(0)[0]),\
        rtb.RevoluteDH(standard_DH_lambda(0)[1]),rtb.RevoluteDH(standard_DH_lambda(0)[2])])

## matrix transformation for one leg
#  rotx/y/z would related to fixed angle, so that:
#  euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
#  for the leg, in euler angle: z1->x1->z2->z3

# R0_1 = [rtb.ET.tz(theta(1,1)),[0;0;0]; 0,0,0,1];
# R1_2 = [rotx(standard_DH(1,4))*rotz(theta(1,2)),[0;0;0]; 0,0,0,1];
# R2_3 = [rotx(standard_DH(2,4))*rotz(theta(1,3)),[0;0;0]; 0,0,0,1];
# R0_3 = R0_1 * R1_2 * R2_3;
# R3_0 = transpose(R0_3);

# T0_1 = [eye(3,3),[0.043; 0; 0]; 0,0,0,1];
# T1_2 = [eye(3,3),[0.073; 0; 0]; 0,0,0,1];
# T2_3 = [eye(3,3),[0.133; 0; 0]; 0,0,0,1];

# HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3;
# HTM_base2leg = cell(6,1);

# for i = 1:6
#     HTM_base2leg{i} = [rotz(angle_base2leg(1,i)),[0;0;0]; 0,0,0,1] * transl(length_base2leg,0,0) * HTM_leg;

## input x,y,z cmd velocity, integrate it and generate the trajectory
#  according to its current position, and return the pulse values of 
#  each joint servo
def cmdHexpodMove(x,y,z):
    

# Board.setBusServoPulse(id, pulse, 1000)