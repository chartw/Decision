

class NormalStop():
    def __init__(self, control):
        self.pub_msg = control.pub_msg
        self.des_brake = 30
        
    def run(self, is_first):
        
        if is_first:
            self.pub_msg.emergency_stop = 0
            self.des_brake = 30
        else:
            self.des_brake += 2 #parameter

        self.pub_msg.brake = self.des_brake
    
        self.pub_msg.steer = 0
        self.pub_msg.speed = 0
        self.pub_msg.emergency_stop = 0
        self.pub_msg.auto_manual = 1
        self.pub_msg.encoder = 0
        self.pub_msg.gear = 0
