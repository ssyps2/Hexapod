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

# "stand pose" joint angle (theta, joint1~3)
theta_stand = (0, np.deg2rad(-12), np.deg2rad(-74))
pulse_stand = (500, 450, 193)

class hex_kine:
    def __init__(self, ctrl_freq):
        self.ctrl_freq = ctrl_freq  # task frequency in Hz
    
    SERVO_ANGLE_RANGE = 240  # in degree
    SERVO_MAX_PULSE = 1000
    SERVO_HALF_PULSE = SERVO_MAX_PULSE/2  # 500
    PULSE2ANGLE = SERVO_ANGLE_RANGE / SERVO_MAX_PULSE  # 0.24, in degree

    @staticmethod
    def getJointAngle():
        """
        : feedback angle by servo in rad (for IK)
            >>> retval: legs_joint_angle[legNum][jointID]
        """
        legs_joint_angle = []

        # joint2 and 3 of leg3~6 was inversed because of assembling
        for i in range(0,6):
            legs_joint_angle.append([pulse_stand[0],pulse_stand[1],pulse_stand[2]])

        return legs_joint_angle

    @staticmethod
    def calcLegFK(theta):
        """
        :[FK Part]
        """
        R0_1 = sm.SE3.Rz(theta[0])
        R1_2 = sm.SE3.Rx(-pi/2) * sm.SE3.Rz(theta[1])
        R2_3 = sm.SE3.Rx(pi) * sm.SE3.Rz(theta[2])
        R0_3 = R0_1 * R1_2 * R2_3
        R3_0 = sm.SE3.inv(R0_3)

        T0_1 = sm.SE3.Tx(0.043)
        T1_2 = sm.SE3.Tx(0.073)
        T2_3 = sm.SE3.Tx(0.133)

        HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3

        return [HTM_leg, HTM_leg.t, R0_3, R3_0]

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

    @staticmethod
    def getEndEffectorPose():
        """
        : matrix transformation for legs related to base frame
        : rotx/y/z would related to fixed angle, so that:
        : euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
        : for the leg, in euler angle: z1->x1->z2->z3
            >>> retval: list of end-effector pose to base frame
        """
        legs_joint_angle = hex_kine.getJointAngle()

        leg_eePose2base_lambda = lambda tran_x,tran_y,tran_z,rota_z,leg_id: sm.SE3(tran_x,tran_y,tran_z)*\
            sm.SE3.Rz(rota_z) * hex_kine.calcLegFK([legs_joint_angle[leg_id][0],legs_joint_angle[leg_id][1],legs_joint_angle[leg_id][2]])[0]

        leg_eePose2base = []

        for i in range(0,6):
            leg_eePose2base.append(leg_eePose2base_lambda(
                hex_kine.position_hip2base[i][0], hex_kine.position_hip2base[i][1],
                hex_kine.position_hip2base[i][2], hex_kine.angle_hip2base[i], i))

        # end-effector pose of each leg relative to its hip (right-handed coordinate)
        leg_eePose2hip_lambda = lambda leg_id: hex_kine.calcLegFK([legs_joint_angle[leg_id][0],legs_joint_angle[leg_id][1],legs_joint_angle[leg_id][2]])[0]

        leg_eePose2hip = [leg_eePose2hip_lambda(0),leg_eePose2hip_lambda(1),leg_eePose2hip_lambda(2),
                        leg_eePose2hip_lambda(3),leg_eePose2hip_lambda(4),leg_eePose2hip_lambda(5)]
        
        return [leg_eePose2base, leg_eePose2hip]

    @staticmethod
    def calcJointAngleIK(x,y,z):
        """
        :[IK Part]
        """
        link_len = (0.043, 0.073, 0.133)
        cos_beta_angle = np.sqrt(x^2+y^2) / np.sqrt(x^2+y^2+z^2)
        L_pow2 = link_len[0]^2 + x^2 + y^2 + z^2 - 2*cos_beta_angle*link_len[0]*np.sqrt(x^2+y^2+z^2)

        theta1 = np.arctan2(y,x)
        theta2 = pi - np.arccos((link_len[0]^2+L_pow2-x^2-y^2-z^2) / (2*link_len[0]*np.sqrt(L_pow2)))\
            - np.arccos((link_len[1]^2+L_pow2-link_len[2]^2) / (2*link_len[1]*np.sqrt(L_pow2)))
        theta3 = np.arccos((link_len[0]^2-link_len[1]^2-link_len[2]^2+x^2+y^2+z^2-2*cos_beta_angle*link_len[0]*np.sqrt(x^2+y^2+z^2))\
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
    hex_timer =  _Timer(increment = 1/(4*pace_freq)) # create a timer

    # constraints
    MAX_PACE_LENGTH = 0.15  # max pace length: 15cm
    MAX_PACE_ABS_POS2BASE = -10 # foot limit absolute position: -10cm to base frame
    MAX_PACE_FREQ = 1  # Hz

    def doTripodGait(self,vx,vy,rz):
        """
        : input x,y,z cmd velocity, integrate it and generate the trajectory,
        : according to its current position, and return the angle of each joint,
        : as well as pace motion constraints and gait implementation
            >>> retval: list of next_foot_joint_angle
        """
        # ------------ get the cmd angle while self-rotation ------------
        legs_yaw_angle = []

        for i in range(0,6):
            legs_yaw_angle.append(hex_kine.getJointAngle()[i][0] + hex_kine.angle_hip2base[i])
        
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
        if hex_kine.pace_freq > hex_kine.MAX_PACE_FREQ:
            raise Exception(f"pace_freq should not be > {hex_kine.MAX_PACE_FREQ}Hz")

        # ---------------- orthogonal of the cmd velocity ----------------
        foots_position2base = hex_kine.getEndEffectorPose()[1]  # only foot position of current stage

        leg_orthogonal_vel = []  # [x_speed, y_speed] relative to base frame

        leftPair_vz = hex_kine.hex_timer.leftPair_pace_flag * 0.04  # m/s
        rightPair_vz = hex_kine.hex_timer.rightPair_pace_flag * 0.04  # m/s

        # leg swinging: (0,2,4) is the left pair, (1,3,5) is the right pair
        for i in (0,2,4):
            if leftPair_vz == 0:
                leg_orthogonal_vel.append([-(vx+rz*np.cos(legRotateAngle)), -(vy+rz*np.sin(legRotateAngle)), 0])
            else:
                leg_orthogonal_vel.append([vx+rz*np.cos(legRotateAngle), vy+rz*np.sin(legRotateAngle), leftPair_vz])
        for i in (1,3,5):
            if rightPair_vz == 0:
                leg_orthogonal_vel.append([-(vx+rz*np.cos(legRotateAngle)), -(vy+rz*np.sin(legRotateAngle)), 0])
            else:
                leg_orthogonal_vel.append([vx+rz*np.cos(legRotateAngle), vy+rz*np.sin(legRotateAngle), rightPair_vz])
        
        # ---------- generate the next foot position (with constraint) ----------
        next_foot_position = []

        # FIXME: servo unable to move when ctrl_freq is high because the value would be too small
        for i in range(0,6):
            next_foot_position.append([
                foots_position2base[0] + leg_orthogonal_vel[i][0] / self.ctrl_freq,
                foots_position2base[1] + leg_orthogonal_vel[i][1] / self.ctrl_freq,
                foots_position2base[2] + leg_orthogonal_vel[i][2] / self.ctrl_freq])

        # next foot position relative to each hip frame
        next_foot_position2hip = []

        for i in range(0,6):
            next_foot_position2hip.append([
                next_foot_position[i][0] - hex_kine.position_hip2base[i][0],
                next_foot_position[i][1] - hex_kine.position_hip2base[i][1],
                next_foot_position[i][2] - hex_kine.position_hip2base[i][2],
                1]) # append 1 to create 1by4 matrix for multiplication
            
            # it also works for 1by3 matrix
            next_foot_position2hip[i] = np.matmul(np.transpose(sm.SE3.Rz(hex_kine.angle_hip2base[i])),
                next_foot_position2hip[i])
        
        # ------ calculate the joint angle according to the generated position ------
        next_foot_joint_angle = []  # joint1~3 angle

        for i in range(0,6):
            next_foot_joint_angle.append(hex_kine.calcJointAngleIK(next_foot_position2hip[i][0],
                next_foot_position2hip[i][1], next_foot_position2hip[i][2]))
        
        return next_foot_joint_angle

    # convert joint angle to pulse value of servo
    angle2pulse_lambda = lambda angle: angle/hex_kine.PULSE2ANGLE + hex_kine.SERVO_HALF_PULSE # in degree

    def cmdHexapodMove(self,vx,vy,rz,mode):
        """
        : send the pulse value to each joint servo
        """
        # inverse kinematics to get joint angle
        if mode == hex_mode_e.TRIPOD:
            leg_joint_angle = hex_kine.doTripodGait(self,vx,vy,rz)
        elif mode == hex_mode_e.STAND:
            pass  # TODO: complete stand mode kinematics

        # if there are no velocity command, stay standing
        if np.abs(vx) < 1e-3 and np.abs(vy) < 1e-3 and np.abs(rz) < 1e-3 == True:
            hex_kine.hex_timer.stop()

            for i in range(0,6):
                leg_joint_angle[i][0] = theta_stand[0]
                leg_joint_angle[i][1] = theta_stand[1]
                leg_joint_angle[i][2] = theta_stand[2]
        else:
            hex_kine.hex_timer.restart()

        print(hex_kine.angle2pulse_lambda(leg_joint_angle[0][0]),
            hex_kine.angle2pulse_lambda(leg_joint_angle[0][1]),
            hex_kine.angle2pulse_lambda(leg_joint_angle[0][2]))
    
    def initHexapod(self):
        hex_kine.cmdHexapodMove(self,0,0,0,hex_mode_e.TRIPOD)