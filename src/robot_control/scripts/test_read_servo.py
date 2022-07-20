import Board, time

i = 0
time_now = time.time()

while True:
    print('-------')

    last_time = time_now
    time_now = time.time()
    print('t: ', time_now - last_time)

    for i in range(0,18):
        Board.setBusServoPulse(i+1,500,1)
        print(Board.getBusServoPulse(i+1))