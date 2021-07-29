from lib.control_utils.general import General

class Parking:
    def __init__(self, control):
        self.generalClass = General(control)

    def drivingParkingNode(self, parking_point):
        steer = self.generalClass.pure_pursuit(parking_point)

        return steer

class ParkingStack:
    def __init__(self):
        self.speed_stack= []
        self.brake_stack= []
        self.steering_stack = []


    def push(self, speed, brake, steering):
        self.speed_stack.append(speed)
        self.brake_stack.append(brake)
        self.steering_stack.append(steering)

    def pop(self):
        return self.speed_stack.pop(), self.brake_stack.pop(), self.steering_stack.pop()


