from os import O_NOATIME


MAX_VALUE = 10000

class Uturn:
    def __init__(self):
        self.none = "NONE"

    def calc_dis(self, a_x, a_y, b_x, b_y):
        distance = ((a_x - b_x)**2 +  (a_y - b_y)**2)**0.5
        return distance

    def sameDirection(self, local, global_path):
        if abs(local.yaw - global_path.yaw) < 5:
            return True
        else:
            return False

    def select_new_target(self, local, global_path):
        
        distance_min = MAX_VALUE
        min_idx = MAX_VALUE

        for i in len(global_path):
            temp_dis = self.calc_dis(global_path[i].x, global_path[i].y, local.x, local.y)
            if temp_dis < distance_min and self.sameDirection(local, global_path[i]):
                distance_min = temp_dis
                min_idx = i

        return min_idx


    
        