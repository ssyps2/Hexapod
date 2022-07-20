import threading
import time

from pyrsistent import inc

class Timer():
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
            print(self.i)
    
    def stop(self):
        self.done=True

    def restart(self):
        self.done=False
        self.next_t=time.time()
        self.i=0
        self._run()

timer = Timer(increment=1)