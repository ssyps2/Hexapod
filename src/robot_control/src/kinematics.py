from cmath import pi
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
        self.MAX_PACE_LENGTH = 0.15  # max pace length: 15cm
    
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
        pulse2angle_lambda = lambda pulse: (pulse - hex_kine.SERVO_HALF_PULSE) * hex_kine.PULSE2ANGLE # in degree

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
    def calcLegFK(theta):
        R0_1 = sm.SE3.Rz(theta[0])
        R1_2 = sm.SE3.Rx(-np.pi/2) * sm.SE3.Rz(theta[1])
        R2_3 = sm.SE3.Rx(np.pi) * sm.SE3.Rz(theta[2])
        R0_3 = R0_1 * R1_2 * R2_3
        R3_0 = np.transpose(R0_3)

        T0_1 = sm.SE3.Tx(0.043)
        T1_2 = sm.SE3.Tx(0.073)
        T2_3 = sm.SE3.Tx(0.133)

        HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3

        return [HTM_leg, R0_3, R3_0]
    
    @staticmethod
    def createRobotLegs():
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
                name='leg'+str(i)))    # create each leg as a "serial-link robot"

        return hex_legs

    # hexapod legs model
    hex_legs = createRobotLegs()

    # hip servo assemble angle relative to base frame
    angle_hip2base = (np.arctan2(6,12), np.pi/2, np.pi-np.arctan2(6,12), 
                    -np.arctan2(6,12), -np.pi/2, np.arctan2(6,12)-np.pi)
    
    position_hip2base = [
        (0.12,  0.06,  -0.02),
        (0,     0.09,  -0.02),
        (-0.12, 0.06,  -0.02),
        (0.12,  -0.06, -0.02),
        (0,     -0.09, -0.02),
        (-0.12, -0.06, -0.02)]

    @staticmethod
    def getEndEffectorPose():
        """
        : [FK Part] matrix transformation for legs related to base frame
        : rotx/y/z would related to fixed angle, so that:
        : euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
        : for the leg, in euler angle: z1->x1->z2->z3
            >>> retval: list of end-effector pose to base frame
        """
        # legs_joint_angle = hex_kine.getJointAngle()

        # processed_theta = []  # "processed" by /2

        # for i in range(0,6):
        #     processed_theta.append([legs_joint_angle[i][0]/2, legs_joint_angle[i][1]/2, legs_joint_angle[i][2]/2])

        # leg_eePose2base_lambda = lambda tran_x,tran_y,rota_z,leg_id: sm.SE3(tran_x,tran_y,0)*sm.SE3.Rz(rota_z)*\
        #      hex_legs[leg_id].fkine(processed_theta[leg_id])

        # end-effector pose of each leg relative to base frame (right-handed coordinate)
        leg_eePose2base_lambda = lambda tran_x,tran_y,tran_z,rota_z,leg_id: sm.SE3(tran_x,tran_y,tran_z)*\
            sm.SE3.Rz(rota_z) * hex_kine.calcLegFK(hex_kine.hex_legs[leg_id].theta)[0]

        leg_eePose2base = []

        for i in range(0,6):
            leg_eePose2base.append(leg_eePose2base_lambda(
                hex_kine.position_hip2base[i][0], hex_kine.position_hip2base[i][1],
                hex_kine.position_hip2base[i][2], hex_kine.angle_hip2base[i], i))

        # end-effector pose of each leg relative to its hip (right-handed coordinate)
        leg_eePose2hip_lambda = lambda leg_id: hex_kine.calcLegFK(hex_kine.hex_legs[leg_id].theta)[0]

        leg_eePose2hip = [leg_eePose2hip_lambda(0),leg_eePose2hip_lambda(1),leg_eePose2hip_lambda(2),
                        leg_eePose2hip_lambda(3),leg_eePose2hip_lambda(4),leg_eePose2hip_lambda(5)]
        
        return [leg_eePose2base, leg_eePose2hip]

    @staticmethod
    def getFootOrientation(x,y,z):
        """
        : for IK
        """
        link_len = (0.043, 0.073, 0.133)
        cos_beta_angle = np.sqrt(x^2+y^2) / np.sqrt(x^2+y^2+z^2)
        L_pow2 = link_len[0]^2 + x^2 + y^2 + z^2 - 2*cos_beta_angle*link_len[0]*np.sqrt(x^2+y^2+z^2)

        theta1 = np.arctan2(y,x)
        theta2 = np.pi - np.arccos((link_len[0]^2+L_pow2-x^2-y^2-z^2) / (2*link_len[0]*np.sqrt(L_pow2)))\
            - np.arccos((link_len[1]^2+L_pow2-link_len[2]^2) / (2*link_len[1]*np.sqrt(L_pow2)))
        theta3 = np.arccos((link_len[0]^2-link_len[1]^2-link_len[2]^2+x^2+y^2+z^2-2*cos_beta_angle*link_len[0]*np.sqrt(x^2+y^2+z^2))\
            / (2*link_len[1]*link_len[2]))
        
        return [theta1, theta2, theta3]

    @staticmethod
    def hexapodIK(vx,vy,rz,vz,mode):
        """
        : [IK Part] input x,y,z cmd velocity, integrate it and generate the trajectory,
        : according to its current position, and return the angle of each joint
            >>> retval: list of joint angle
        """ 
        # in tripod gait mode
        if mode == hex_mode_e.TRIPOD:
            # ---------- get the cmd angle while self-rotation ----------
            legs_yaw_angle = []

            for i in range(0,6):
                legs_yaw_angle.append(hex_kine.getJointAngle(i)[0] + hex_kine.angle_hip2base[i])
            
            legRotateAngle = [] # leg rotate vector angle to base frame

            for i in range(0,6):    # perpendicular to "legs_yaw_angle"
                legRotateAngle.append(legs_yaw_angle[i] + np.pi)
            
            # angle limitation
            for i in range(0,6):
                if legRotateAngle[i] > np.pi:
                    legRotateAngle[i] -= 2*np.pi
                elif legRotateAngle[i] < -np.pi:
                    legRotateAngle[i] += 2*np.pi

            # ---------- orthogonal of the cmd velocity ----------
            leg_orthogonal_vel = []  # [x_speed, y_speed] relative to base frame

            for i in range(0,6):
                leg_orthogonal_vel.append([vx+rz*np.cos(legRotateAngle), vy+rz*np.sin(legRotateAngle), vz])
            
            # ---------- generate the next foot position (with constraint) ----------
            foots_position2base = hex_kine.getEndEffectorPose()[0].t  # only foot position of current stage

            next_foot_position = []

            # FIXME: unmoveable because the value was too small
            for i in range(0,6):
                next_foot_position.append([
                    foots_position2base[0] + leg_orthogonal_vel[0] / hex_kine.task_freq,
                    foots_position2base[1] + leg_orthogonal_vel[1] / hex_kine.task_freq,
                    foots_position2base[2] + leg_orthogonal_vel[2] / hex_kine.task_freq])

            # next foot position relative to each hip frame
            next_foot_position2hip = []

            for i in range(0,6):
                next_foot_position2hip.append([
                    next_foot_position[i][0] - hex_kine.position_hip2base[i][0],
                    next_foot_position[i][1] - hex_kine.position_hip2base[i][1],
                    next_foot_position[i][2] - hex_kine.position_hip2base[i][2]])
                
                # TODO: check whether it work for 1by3 matrix
                next_foot_position2hip[i] *= np.transpose(sm.SE3.Rz(hex_kine.angle_hip2base[i]))
            
            # ------ obtain the orientation of the EE of generated foot position ------
            next_foot_joint_angle = []  # joint1~3 angle

            for i in range(0,6):
                next_foot_joint_angle.append(hex_kine.getFootOrientation(next_foot_position2hip[i][0],
                    next_foot_position2hip[i][1], next_foot_position2hip[i][2]))

            next_foot_orientation2hip = []

            next_foot_orientation2base = []

        # in stand mode
        elif mode == hex_mode_e.STAND:
            pass

    # convert joint angle to pulse value of servo
    angle2pulse_lambda = lambda angle: angle/hex_kine.PULSE2ANGLE + hex_kine.SERVO_HALF_PULSE # in degree
    
    @classmethod
    def cmdHexapodMove(vx,vy,rz,mode):
        """
        : send the pulse value to each joint servo
        """
        # "stand pose" joint angle (theta, joint1~3)
        theta_stand = (0, np.deg2rad(-12), np.deg2rad(-74))

        # inverse kinematics to get joint angle
        leg_joint_angle = hex_kine.hexapodIK(vx,vy,rz,mode)

        # if there are no velocity command, stay standing
        if np.abs(vx) < 1e-3 and np.abs(vy) < 1e-3 and np.abs(rz) < 1e-3 == True:
            for i in range(0,6):
                leg_joint_angle[i][0] = theta_stand[0]
                leg_joint_angle[i][1] = theta_stand[1]
                leg_joint_angle[i][2] = theta_stand[2]

        # joint2 and 3 of leg3~5 was inversed because of assembling
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
        hex_kine.cmdHexapodMove(0,0,0,hex_mode_e.TRIPOD)

hex_kine.initHexapod()