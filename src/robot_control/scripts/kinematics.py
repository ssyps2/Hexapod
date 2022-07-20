from cmath import pi
import enum
import numpy as np
import spatialmath as sm
# import roboticstoolbox as rtb
import threading
import time

class hex_mode_e(enum.Enum):
    STAND  = 0,
    TRIPOD = 1,

# "stand pose" joint angle (theta, joint0~2)
theta_stand = (0, np.deg2rad(-12), np.deg2rad(-74))

## init joint servo ID in legs
leg_ID_lambda = lambda id: (id,id+1,id+2)
#               leg0            leg1             leg2              leg3             leg4              leg5
legs_ID = (leg_ID_lambda(7),leg_ID_lambda(4),leg_ID_lambda(1),leg_ID_lambda(10),leg_ID_lambda(13),leg_ID_lambda(16))

class hexKine():
    def __init__(self, ctrl_freq):
        self.ctrl_freq = ctrl_freq  # task frequency in Hz

        self.hex_timer =  self._Timer(increment = 1/(4*hexKine.pace_freq)) # create a timer
        self.restart_flag = 0

        self.legs_joint_angle = [[]]
        self.next_feet_joint_angle = [[]]

    SERVO_ANGLE_RANGE = 240  # in degree
    SERVO_MAX_PULSE = 1000
    SERVO_HALF_PULSE = SERVO_MAX_PULSE/2  # 500
    PULSE2ANGLE = SERVO_ANGLE_RANGE / SERVO_MAX_PULSE  # 0.24, in degree

    @staticmethod
    def calcLegFK(theta):
        """
        :[FK Part]
        """
        R0_1 = sm.SE3.Rz(theta[0])
        R1_2 = sm.SE3.Rx(-pi/2) * sm.SE3.Rz(theta[1])
        R2_3 = sm.SE3.Rx(pi) * sm.SE3.Rz(theta[2])
        # R0_3 = R0_1 * R1_2 * R2_3
        # R3_0 = sm.SE3.inv(R0_3)

        T0_1 = sm.SE3.Tx(0.043)
        T1_2 = sm.SE3.Tx(0.073)
        T2_3 = sm.SE3.Tx(0.133)

        HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3

        return HTM_leg
    
    # def createRobotLegs(self):
    #     # d=offset(i), a=length(i), alpha(i), theta(i)
    #     standard_DH_lambda = lambda legNum: [ [0, 0.043, -pi/2, self.legs_joint_angle[legNum][0]],
    #         [0, 0.073, pi, self.legs_joint_angle[legNum][1]], [0, 0.133, 0, self.legs_joint_angle[legNum][2]] ]

    #     hex_legs = []

    #     # update DH parameter for each leg, qlim is the joint angle limitation
    #     for i in range(0,6):
    #         hex_legs.append(rtb.DHRobot([
    #             rtb.RevoluteDH(a = standard_DH_lambda(i)[0][1], alpha = standard_DH_lambda(i)[0][2],
    #                 offset = standard_DH_lambda(i)[0][3], qlim = (np.deg2rad(-45),np.deg2rad(45))),
    #             rtb.RevoluteDH(a = standard_DH_lambda(i)[1][1], alpha = standard_DH_lambda(i)[1][2],
    #                 offset = standard_DH_lambda(i)[1][3], qlim = (np.deg2rad(-60),np.deg2rad(90))),
    #             rtb.RevoluteDH(a = standard_DH_lambda(i)[2][1], alpha = standard_DH_lambda(i)[2][2],
    #                 offset = standard_DH_lambda(i)[2][3], qlim = (np.deg2rad(-120),0))],
    #             name='leg'+str(i)))    # create each leg as a "serial-link robot"

    #     return hex_legs

    # # hexapod legs model
    # hex_legs = createRobotLegs()

    # hip servo assemble angle relative to base frame
    angle_hip2base = (np.arctan2(6,12), pi/2, pi-np.arctan2(6,12), 
                    -np.arctan2(6,12), -pi/2, np.arctan2(6,12)-pi)
    
    position_hip2base = [
        (0.12,  0.06,  -0.02),
        (0,     0.09,  -0.02),
        (-0.12, 0.06,  -0.02),
        (0.12,  -0.06, -0.02),
        (0,     -0.09, -0.02),
        (-0.12, -0.06, -0.02)]

    def getEndEffectorPose(self):
        """
        : matrix transformation for legs related to base frame
        : rotx/y/z would related to fixed angle, so that:
        : euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
        : for the leg, in euler angle: z1->x1->z2->z3
            >>> retval: list of end-effector pose to base frame
        """
        # end-effector pose of each leg relative to base frame (right-handed coordinate)
        leg_eePose2base_lambda = lambda tran_x,tran_y,tran_z,rota_z,leg_id: sm.SE3(tran_x,tran_y,tran_z)*\
            sm.SE3.Rz(rota_z) * hexKine.calcLegFK([self.legs_joint_angle[leg_id][0],self.legs_joint_angle[leg_id][1],
            self.legs_joint_angle[leg_id][2]])

        leg_eePose2base = []

        for i in range(0,6):
            leg_eePose2base.append(leg_eePose2base_lambda(
                hexKine.position_hip2base[i][0], hexKine.position_hip2base[i][1],
                hexKine.position_hip2base[i][2], hexKine.angle_hip2base[i], i))

        return leg_eePose2base

    @staticmethod
    def calcJointAngleIK(x,y,z):
        """
        :[IK Part]
        """
        link_len = (0.043, 0.073, 0.133)
        cos_beta_angle = np.sqrt(x**2+y**2) / np.sqrt(x**2+y**2+z**2)
        L_pow2 = link_len[0]**2 + x**2 + y**2 + z**2 - 2*cos_beta_angle*link_len[0]*np.sqrt(x**2+y**2+z**2)

        theta1 = np.arctan2(y,x)
        theta2 = pi - np.arccos((link_len[0]**2+L_pow2-x**2-y**2-z**2) / (2*link_len[0]*np.sqrt(L_pow2)))\
            - np.arccos((link_len[1]**2+L_pow2-link_len[2]**2) / (2*link_len[1]*np.sqrt(L_pow2)))
        theta3 = np.arccos((link_len[0]**2-link_len[1]**2-link_len[2]**2+x**2+y**2+z**2-2*cos_beta_angle*link_len[0]*np.sqrt(x**2+y**2+z**2))\
            / (2*link_len[1]*link_len[2]))
        
        return [theta1, theta2, theta3]

    class _Timer():
        def __init__(self, increment):
            self.next_t=time.time()
            self.i=0
            self.done=False
            self.increment=increment

            self.leftPair_pace_flag = 0
            self.rightPair_pace_flag = 0

            self.left_flag_sequence = (1,-1,0,0)
            self.right_flag_sequence = (0,0,1,-1)

            self._run()

        def _run(self):
            self.next_t+=self.increment

            self.leftPair_pace_flag = self.left_flag_sequence[self.i]
            self.rightPair_pace_flag = self.right_flag_sequence[self.i]

            if not self.done:
                threading.Timer( self.next_t - time.time(), self._run).start()
                if self.i >= 3:
                    self.i = 0
                else:
                    self.i += 1
        
        def stop(self):
            self.done=True

        def restart(self):
            self.done=False
            self.next_t=time.time()
            self.i=0
            self._run()

    pace_freq = 0.5  # Hz

    # constraints
    MAX_PACE_LENGTH = 0.15  # max pace length: 15cm
    MAX_PACE_ABS_POS2BASE = -10 # foot limit absolute position: -10cm to base frame
    MAX_PACE_FREQ = 1  # Hz

    def generateTripodNextFoot(self,vx,vy,rz):
        """
        : input x,y,z cmd velocity, integrate it and generate the trajectory,
        : according to its current position, and return the angle of each joint,
        : as well as pace motion constraints and gait implementation
            >>> retval: list of next_foot_joint_angle
        """
        # ------------ get the cmd angle while self-rotation ------------
        legs_yaw_angle = []

        for i in range(0,6):
            legs_yaw_angle.append(self.legs_joint_angle[i][0] + hexKine.angle_hip2base[i])
        
        legRotateAngle = [] # leg rotate vector angle to base frame

        for i in range(0,6):    # perpendicular to "legs_yaw_angle"
            legRotateAngle.append(legs_yaw_angle[i] + pi)
        
        # angle format
        for i in range(0,6):
            if legRotateAngle[i] > pi:
                legRotateAngle[i] -= 2*pi
            elif legRotateAngle[i] < -pi:
                legRotateAngle[i] += 2*pi

        # ------------------ regulate the pace freq ------------------
        if hexKine.pace_freq > hexKine.MAX_PACE_FREQ:
            raise Exception(f"pace_freq should not be > {hexKine.MAX_PACE_FREQ}Hz")

        # ---------------- orthogonal of the cmd velocity ----------------
        foots_position2base = []  # only foot position of current stage

        for i in range(0,6):
            foots_position2base.append(hexKine.getEndEffectorPose()[i].t)  # only foot position of current stage

        leg_orthogonal_vel = []  # [x_speed, y_speed] relative to base frame

        leftPair_vz = self.hex_timer.leftPair_pace_flag * 0.04  # m/s
        rightPair_vz = self.hex_timer.rightPair_pace_flag * 0.04  # m/s

        # leg swinging: (0,2,4) is the left pair, (1,3,5) is the right pair
        for i in (0,2,4):
            if leftPair_vz==0 and self.hex_timer.done==False:
                leg_orthogonal_vel.append([-(vx+rz*np.cos(legRotateAngle[i])), -(vy+rz*np.sin(legRotateAngle[i])), 0])
            else:
                leg_orthogonal_vel.append([vx+rz*np.cos(legRotateAngle[i]), vy+rz*np.sin(legRotateAngle[i]), leftPair_vz])
        for i in (1,3,5):
            if rightPair_vz==0 and self.hex_timer.done==False:
                leg_orthogonal_vel.append([-(vx+rz*np.cos(legRotateAngle[i])), -(vy+rz*np.sin(legRotateAngle[i])), 0])
            else:
                leg_orthogonal_vel.append([vx+rz*np.cos(legRotateAngle[i]), vy+rz*np.sin(legRotateAngle[i]), rightPair_vz])
        
        # ---------- generate the next foot position (with constraint) ----------
        next_foot_position = []

        # FIXME: servo unable to move when ctrl_freq is high because the value would be too small
        for i in range(0,6):
            next_foot_position.append([
                foots_position2base[i][0] + leg_orthogonal_vel[i][0] / self.ctrl_freq,
                foots_position2base[i][1] + leg_orthogonal_vel[i][1] / self.ctrl_freq,
                foots_position2base[i][2] + leg_orthogonal_vel[i][2] / self.ctrl_freq])

        # next foot position relative to each hip frame
        next_foot_position2hip = []

        for i in range(0,6):
            next_foot_position2hip.append([
                next_foot_position[i][0] - hexKine.position_hip2base[i][0],
                next_foot_position[i][1] - hexKine.position_hip2base[i][1],
                next_foot_position[i][2] - hexKine.position_hip2base[i][2],
                1.]) # append 1 to create 1by4 matrix for multiplication
            
            # it also works for 1by3 matrix
            next_foot_position2hip[i] = np.dot(np.transpose(sm.SE3.Rz(hexKine.angle_hip2base[i])),
                next_foot_position2hip[i])
        
        # ------ calculate the joint angle according to the generated position ------
        next_foot_joint_angle = []  # joint0~2 angle

        for i in range(0,6):
            next_foot_joint_angle.append(hexKine.calcJointAngleIK(next_foot_position2hip[i][0],
                next_foot_position2hip[i][1], next_foot_position2hip[i][2]))
        
        return next_foot_joint_angle

    def implementCmdGait(self,vx,vy,rz,mode):
        """
        : send the pulse value to each joint servo
        """
        # inverse kinematics to get joint angle
        if mode == hex_mode_e.TRIPOD:
            next_leg_joint_angle = hexKine.generateTripodNextFoot(self,vx,vy,rz)
        elif mode == hex_mode_e.STAND:
            pass  # TODO: complete stand mode kinematics

        # if there are no velocity command, stay standing
        if np.abs(vx) < 1e-3 and np.abs(vy) < 1e-3 and np.abs(rz) < 1e-3:
            self.hex_timer.stop()

            # TODO: use jtraj to smooth it
            for i in range(0,6):
                next_leg_joint_angle[i][0] = theta_stand[0]
                next_leg_joint_angle[i][1] = theta_stand[1]
                next_leg_joint_angle[i][2] = theta_stand[2]
            
            self.restart_flag = 1

        else:
            if self.restart_flag == 1:
                self.hex_timer.restart()
                self.restart_flag = 0  # do once

        return next_leg_joint_angle
    
    def initHexapod(self):
        self.implementCmdGait(vx=0, vy=0, rz=0, mode=hex_mode_e.TRIPOD)  # "stand-up" pose