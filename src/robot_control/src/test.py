import numpy as np
import spatialmath as sm
from cmath import pi

theta = (0, np.deg2rad(-12), np.deg2rad(-74))

R0_1 = sm.SE3.Rz(theta[0])
R1_2 = sm.SE3.Rx(-pi/2) * sm.SE3.Rz(theta[1])
R2_3 = sm.SE3.Rx(pi) * sm.SE3.Rz(theta[2])
R0_3 = R0_1 * R1_2 * R2_3
R3_0 = sm.SE3.inv(R0_3)

T0_1 = sm.SE3.Tx(0.043)
T1_2 = sm.SE3.Tx(0.073)
T2_3 = sm.SE3.Tx(0.133)

HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3

print(HTM_leg.t)