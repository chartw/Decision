





class Parking():
    def __init__(self, controlplanner):
        self.data = controlplanner.control_data


    def parking_start(self):
        pass #여기 뭔지 모르겠음

    def parking_complete(self):
        self.data['break_val'] = 200
        self.data['speed'] = 0x00
        # steering = 1999
        # gear = 0x02

    def backward_start(self):
        self.data['speed'] = 0x50
        self.data['steering'] = 400
        self.data['gear'] = 0x02

    def backward_complete(self):
        self.data['break_val'] = 200
        self.data['speed'] = 0x00

    def driving_start(self):
        self.data['speed'] = 0x30
        self.data['steering'] = 0
        self.data['gear'] = 0x00

        ##여긴 뭐임????????????????????
        print("GPP start")
        self.mission_mode = ""
        self.GPP()
        self.mode_sw = GlobalPath
        #self.GPP_is_done = 1
        self.LPP_is_done = 1


