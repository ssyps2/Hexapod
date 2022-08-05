import Board, time
import matplotlib.pyplot as plt

flag = 0
Board.setBusServoPulse(9,250,1000)
time.sleep(2)

last_time = time.time()
read = []
step = 2

for pulse in range(250,751,step):
    t1 = time.time()
    Board.setBusServoPulse(9,pulse,1)
    t2 = time.time()
    t3 = 0.001 -(t2-t1)  # .00173
    if t3 > 0:
        time.sleep(t3)

time_now = time.time()
print('t: ', time_now - last_time)

# plt.plot(range(250,751), read)
# plt.show()