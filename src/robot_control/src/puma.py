import time
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb
 
puma = rtb.models.DH.Puma560()
T = puma.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])  
q = puma.ikine_a(T, 'ru').q
print(q)
puma.plot(q)
time.sleep(15)
