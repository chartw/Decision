

class Parking():
  def __init__(self, controlplanner):
      self.data = controlplanner.control_data


    def parking_process(self, break_val, speed, steering, gear):
        # 주차 완료
        if self.mission_mode == "PCOM":
            break_val = 200
            # self.mission_mode = "Backward"
            speed = 0x00
            # steering = 1999
            # gear = 0x02

        # 후진 완료
        if self.mission_mode == "BCOM":
            break_val = 200
            speed = 0x00

        # 전진
        if self.mission_mode == "Driving":
            # break_val = 0
            speed = 0x30
            steering = 0
            gear = 0x00

            print("GPP start")
            self.mission_mode = ""
            self.GPP()
            self.mode_sw = GlobalPath
            #self.GPP_is_done = 1
            self.LPP_is_done = 1

        # 후진
        if self.mission_mode == "Backward":
            speed = 0x50
            steering = 400
            gear = 0x02
            # self.mission_mode = "Driving"

    def parking_next_process(self):
        if self.mission_mode == "BCOM":
            time.sleep(2)
            self.mission_mode = "Driving"

        if self.mission_mode == "Backward":
            time.sleep(6)
            self.mission_mode = "BCOM"

        if self.mission_mode == "PCOM":
            print("PCOMing")
            time.sleep(2)
            
            self.mission_mode = "Backward"