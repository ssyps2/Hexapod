import threading, Board, time

class servoThread (threading.Thread):
    def __init__(self, name, id):
        threading.Thread.__init__(self)
        self.name = name
        self.id = id

    def run(self):
        while True:
            print(f"{self.name}: ", Board.getBusServoPulse(self.id))

servo_thread = []

for i in range(0,2):
    servo_thread.append(servoThread(name=f"Servo{i+1}", id=i+1))
    servo_thread[i].start()