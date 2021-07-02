from math import degrees, atan2, sin, radians


class General:
    def init(self, control):
        self.cur_x = control.planning_info.position.x
        self.cur_y = control.planning_info.position.y
        self.cur_yaw = control.planning_info.position.heading
        self.path = control.global_path
        self.lookahead = control.lookahead
        self.WB = 1
        self.target_index = 0

    def select_target(self):
        valid_idx_list = []

        for i in range(self.target_index, len(self.path)):
            dis = ((self.path.x[i] - self.cur_x) ** 2 + (self.path.y[i] - self.cur_y) ** 2) ** 0.5

            if dis <= self.lookahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.lookahead:
                break
        if len(valid_idx_list) == 0:
            self.target_index = 0
        else:
            self.target_index = valid_idx_list[len(valid_idx_list) - 1]

    def pure_pursuit(self):
        self.target_index = self.select_target()

        target_x = self.path.x[self.target_index]
        target_y = self.path.y[self.target_index]
        # pure pursuit 계산되는 부분
        tmp_th = degrees(atan2((target_y - self.cur_y), (target_x - self.cur_x)))

        tmp_th = tmp_th % 360

        alpha = self.cur_yaw - tmp_th
        if abs(alpha) > 180:
            if alpha < 0:
                alpha += 360
            else:
                alpha -= 360

        alpha = max(alpha, -90)
        alpha = min(alpha, 90)

        delta = degrees(atan2(2 * self.WB * sin(radians(alpha)) / self.lookahead, 1))

        if abs(delta) > 180:
            if delta < 0:
                delta += 360
            else:
                delta -= 360

        if abs(delta) >= 27.7:
            if delta > 0:
                return 27.7
            else:
                return -27.7
        else:

            return delta
