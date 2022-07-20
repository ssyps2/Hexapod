import threading, Board

class servoThread (threading.Thread):
    def __init__(self, name, id):
        threading.Thread.__init__(self)
        self.name = name
        self.id = id

    def run(self):
        while True:
            Board.getBusServoPulse(self.id)

servo_thread = []

for i in range(0,18):
    servo_thread.append(servoThread(name=f"Servo_{i+1}", id=i))
    servo_thread[i].start()