

class NormalStop():
    def __init__(self, control):
        self.pub_msg = control.pub_msg
        self.status = control.serial_info
        self.des_brake = 30
        
    def run(self):
        
        self.pub_msg.brake = 200
        
        self.pub_msg.steer = 0
        self.pub_msg.speed = 0
        self.pub_msg.emergency_stop = 0
        self.pub_msg.auto_manual = 1
        self.pub_msg.encoder = 0
        self.pub_msg.gear = 0
             
        if self.status.speed > 0.5:
            self.des_brake += 2 #parameter
            self.pub_msg.brake = self.des_brake
        else:
            self.pub_msg.emergency_stop = 1
            self.des_brake = 30
        