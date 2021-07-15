
#0705 by 민규
class traffic_light_3:#main문에서 계속 돌아서 상태가 변하게끔 해서 사용하면되고(planner 객체에서 멤버변수 global_time을 와일문 돌때마다 갱신하고 받아와서사용)
# 이에 해당하는 명령을 수행하게 하면됨 
    def __init__(self):            
        self.traffic_state = 'red' # 여기에 파이썬에서 입력을 주고 입력에 따라서 넘어가게 하면됨
        self.traffic_flg_time = time.time()
        self.red_time = None # 3구 신호등 주기 측정후 넣어야함
        self.orange_time = None
        self.green_time = None 
    def change_traffic_state(self,global_time):
        if (self.traffic_state=='red' and global_time >= (self.traffic_flg_time + self.red_time)):
            self.traffic_state = 'green'
            self.traffic_flg_time = global_time
        elif (self.traffic_state=='green' and global_time >= (self.traffic_flg_time + self.green_time)):
            self.traffic_state = 'orange'
            self.traffic_flg_time = global_time   
        elif (self.traffic_state=='orange' and global_time >= (self.traffic_flg_time + self.orange_time)):
            self.traffic_state = 'red'
            self.traffic_flg_time = global_time         
        

class traffic_light_4:#main문에서 계속 돌아서 상태가 변하게끔 해서 사용하면되고(planner 객체에서 멤버변수 global_time을 와일문 돌때마다 갱신하고 받아와서사용)
# 이에 해당하는 명령을 수행하게 하면됨 
    def __init__(self):            
        self.traffic_state = 'red' # 여기에 파이썬에서 입력을 주고 입력에 따라서 넘어가게 하면됨
        self.traffic_flg_time = time.time()
        self.red_time = None # 3구 신호등 주기 측정후 넣어야함
        self.orange_time = None
        self.green_time = None 
    def change_traffic_state(self,global_time):
        if (self.traffic_state=='red' and global_time >= (self.traffic_flg_time + self.red_time)):
            self.traffic_state = 'green'
            self.traffic_flg_time = global_time
        elif (self.traffic_state=='green' and global_time >= (self.traffic_flg_time + self.green_time)):
            self.traffic_state = 'orange'
            self.traffic_flg_time = global_time   
        elif (self.traffic_state=='orange' and global_time >= (self.traffic_flg_time + self.orange_time)):
            self.traffic_state = 'red'
            self.traffic_flg_time = global_time         
        