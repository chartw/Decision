

class NormalStop():
    def __init__(self, control):
        self.control_msg = control.control_msg
        self.status = control.serial_info
        self.des_brake = 30
        
    def run(self):
        
        self.control_msg.brake = 200
        
        self.control_msg.steer = 0
        self.control_msg.speed = 0
        self.control_msg.emergency_stop = 0
        self.control_msg.auto_manual = 1
        self.control_msg.encoder = 0
        self.control_msg.gear = 0
             
        if self.status.speed > 0.5:
            self.des_brake += 2 #parameter
            self.control_msg.brake = self.des_brake
        else:
            self.control_msg.emergency_stop = 1
            self.des_brake = 30
            
        


