
import sys
import signal

class SigIntHandler:
    def __init__(self):
        pass

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C! Please never use Ctrl+Z!')
        sys.exit(0)

    def run(self):

        signal.signal(signal.SIGINT, self.signal_handler)