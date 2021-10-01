import queue
from math import hypot
from geometry_msgs.msg import Point32

class ExpMovAvgFilter:
    # 이전 스텝의 평균
    expAvg=Point32()

    a=0.05
    
    def __init__(self,input):
        self.expAvg=input
    
    def emaFilter(self, input):
        self.expAvg=Point32(self.a*input.x+(1-self.a)*self.expAvg.x,self.a*input.y+(1-self.a)*self.expAvg.y,0)
    
    def tracking(self, input):

        if hypot(input.x-self.expAvg.x, input.y-self.expAvg.y) < 1.5:
            return True

        return False

    def retAvg(self):

        return self.expAvg
