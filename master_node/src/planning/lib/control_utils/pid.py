class PID:
    def __init__(self):
        self.P = 4.0
        self.I = 0.5
        self.D = 0
        self.pre_error = 0.0
        self.error_sum = 0.0
        self.dt = 1.0 / 10.0



    def run(self, target, current, Vcurrent):

        if Vcurrent < 0.2  and current <= 1:
            self.pre_error = 0.0
            self.error_sum = 0.0
            return 0.7  # 어떤값으로 주는게 좋을지 찾아야함
        elif current >= 10:
            self.pre_error = 0.0
            self.error_sum = 0.0
            return 8.0
        else:
            error = current - target

            diff_error = error - self.pre_error
            self.pre_error = error
            self.error_sum += error
            return self.P * error + self.D * diff_error / self.dt + self.I * self.error_sum * self.dt
