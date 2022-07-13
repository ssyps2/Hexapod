import enum
import time
import Board
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb

class hex_mode_e(enum.Enum):
    STAND  = 0,
    TRIPOD = 1,

class hex_kine(object):
    def __init__(self, task_freq):
        # task frequency in Hz
        self.task_freq = task_freq

        # constants definition
        self.SERVO_ANGLE_RANGE = 240  # in degree
        self.SERVO_MAX_PULSE = 1000
        self.SERVO_HALF_PULSE = self.SERVO_MAX_PULSE/2  # 500
        self.PULSE2ANGLE = self.SERVO_ANGLE_RANGE / self.SERVO_MAX_PULSE  # 0.24, in degree
        self.MAX_PACE_LENGTH = 0.15  # unit:m
    
    ## init joint servo ID in legs
    leg_ID_lambda = lambda id: (id,id+1,id+2)
    #               leg1            leg2             leg3              leg4             leg5              leg6
    legs_ID = (leg_ID_lambda(7),leg_ID_lambda(4),leg_ID_lambda(1),leg_ID_lambda(10),leg_ID_lambda(13),leg_ID_lambda(16))

    # initialize the behaviour mode to be tripod gait
    global hex_mode
    hex_mode = hex_mode_e.TRIPOD

    @staticmethod
    def getJointAngle():
        """
        : feedback angle by servo in rad (for IK)
            >>> retval: legs_joint_angle[legNum][jointID]
        """
        pulse2angle_lambda = lambda pulse: (pulse-hex_kine.SERVO_HALF_PULSE)*hex_kine.PULSE2ANGLE # in degree

        legs_joint_angle = []

        # joint2 and 3 of leg3~6 was inversed because of assembling
        for i in range(0,6):
            if i < 3:
                legs_joint_angle.append([pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][0])),
                                        pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][1])),
                                        pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][2]))])
            elif i >= 3:
                legs_joint_angle.append([pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][0])),
                                        -pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][1])),
                                        -pulse2angle_lambda(Board.getBusServoPulse(hex_kine.legs_ID[i][2]))])

        return legs_joint_angle

    @staticmethod
    def getEndEffectorPose():
        """
        : [FK Part] matrix transformation for legs related to base frame
        : rotx/y/z would related to fixed angle, so that:
        : euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
        : for the leg, in euler angle: z1->x1->z2->z3
            >>> retval: list of end-effector pose to base frame
        """
        legs_joint_angle = hex_kine.getJointAngle()

        # d=offset(i), a=length(i), alpha(i), theta(i)
        standard_DH_lambda = lambda legNum: [ [0, 0.043, -np.pi/2, legs_joint_angle[legNum][0]],
            [0, 0.073, np.pi, legs_joint_angle[legNum][1]], [0, 0.133, 0, legs_joint_angle[legNum][2]] ]

        hex_legs = []

        # update DH parameter for each leg, qlim is the joint angle limitation
        for i in range(0,6):
            hex_legs.append(rtb.DHRobot([
                rtb.RevoluteDH(a = standard_DH_lambda(i)[0][1], alpha = standard_DH_lambda(i)[0][2],
                    offset = standard_DH_lambda(i)[0][3], qlim = (np.deg2rad(-45),np.deg2rad(45))),
                rtb.RevoluteDH(a = standard_DH_lambda(i)[1][1], alpha = standard_DH_lambda(i)[1][2],
                    offset = standard_DH_lambda(i)[1][3], qlim = (np.deg2rad(-60),np.deg2rad(90))),
                rtb.RevoluteDH(a = standard_DH_lambda(i)[2][1], alpha = standard_DH_lambda(i)[2][2],
                    offset = standard_DH_lambda(i)[2][3], qlim = (np.deg2rad(-120),0))],
                name='leg'+str(i+1)))    # create each leg as a "serial-link robot"

        processed_theta = []
        
        for i in range(0,6):
            processed_theta.append(hex_legs[i].theta)

        # end-effector pose of each leg related to base frame (left-handed coordinate)
        leg_eePose2base_lambda = lambda tran_x,tran_y,rota_z,leg_id: sm.SE3.Trans(tran_x,tran_y)*\
            sm.SO3.Rz(rota_z) * hex_legs[leg_id-1].fkine(processed_theta[leg_id-1])  # hex_legs[leg_id-1].theta
        
        leg_eePose2base = [
            leg_eePose2base_lambda(0.12,  -0.06, np.arctan2(6,12),       1),
            leg_eePose2base_lambda(0,     -0.09, np.pi/2,                2),
            leg_eePose2base_lambda(-0.12, -0.06, np.pi-np.arctan2(6,12), 3),
            leg_eePose2base_lambda(0.12,  0.06,  -np.arctan2(6,12),      4),
            leg_eePose2base_lambda(0,     -0.09, -np.pi/2,               5),
            leg_eePose2base_lambda(-0.12, 0.06,  np.arctan2(6,12)-np.pi, 6)]
        
        return leg_eePose2base
    
    @staticmethod
    def hexapodIK(vx,vy,rz,mode):
        """
        : [IK Part] input x,y,z cmd velocity, integrate it and generate the trajectory,
        : according to its current position, and return the angle of each joint
            >>> retval: list of joint angle
        """
        # in tripod gait mode
        if hex_mode == hex_mode_e.TRIPOD:
            # superposition the cmd velocity
            legs_joint_angle = hex_kine.getJointAngle()


            foots_pose = hex_kine.getEndEffectorPose()

        # in stand mode
        elif hex_mode == hex_mode_e.STAND:
            pass

    #convert joint angle to pulse value of servo
    angle2pulse_lambda = lambda angle: angle/hex_kine.PULSE2ANGLE + hex_kine.SERVO_HALF_PULSE # in degree
    
    @classmethod
    def cmdHexapodMove(vx,vy,rz,mode):
        """
        : send the pulse value to each joint servo
        """
        # inverse kinematics to get joint angle
        leg_joint_angle = hex_kine.hexapodIK(vx,vy,rz,mode)

        # joint2 and 3 of leg3~6 was inversed because of assembling
        for i in range(0,6):
            if i < 3:
                Board.setBusServoPulse(hex_kine.legs_ID[i][0], hex_kine.angle2pulse_lambda(leg_joint_angle[i][0]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][1], hex_kine.angle2pulse_lambda(leg_joint_angle[i][1]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][2], hex_kine.angle2pulse_lambda(leg_joint_angle[i][2]), 1000)
            elif i >= 3:
                Board.setBusServoPulse(hex_kine.legs_ID[i][0], hex_kine.angle2pulse_lambda(leg_joint_angle[i][0]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][1], hex_kine.angle2pulse_lambda(-leg_joint_angle[i][1]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][2], hex_kine.angle2pulse_lambda(-leg_joint_angle[i][2]), 1000)
    
    @classmethod
    def initHexapod():
        # "stand pose" joint angle (theta, joint1~3)
        # TODO: find out why there should be /2
        theta_stand = (0, np.deg2rad(-12/2), np.deg2rad(-74/2))

        for i in range(0,6):
            if i < 3:
                Board.setBusServoPulse(hex_kine.legs_ID[i][0], hex_kine.angle2pulse_lambda(theta_stand[0]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][1], hex_kine.angle2pulse_lambda(theta_stand[1]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][2], hex_kine.angle2pulse_lambda(theta_stand[2]), 1000)
            elif i >= 3:
                Board.setBusServoPulse(hex_kine.legs_ID[i][0], hex_kine.angle2pulse_lambda(theta_stand[0]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][1], hex_kine.angle2pulse_lambda(-theta_stand[1]), 1000)
                Board.setBusServoPulse(hex_kine.legs_ID[i][2], hex_kine.angle2pulse_lambda(-theta_stand[2]), 1000)

hex_kine.initHexapod()