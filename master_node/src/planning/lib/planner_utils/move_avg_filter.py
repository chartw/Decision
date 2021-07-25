import queue
from geometry_msgs.msg import Point32

class MovAvgFilter:
    # 이전 스텝의 평균
    prevAvg = Point32()
    # 가장 최근 n개의 값을 저장하는 큐
    xBuf = queue.Queue()
    # 참조할 데이터의 갯수
    n = 0
    
    def __init__(self, _n, input):
        # 초기화로 n개의 값을 0으로 둡니다.
        for _ in range(_n):
            self.xBuf.put(input)
        # 참조할 데이터의 갯수를 저장합니다.
        self.n = _n
    
    def movAvgFilter(self, input):
        # 큐의 front 값은 x_(k-n) 에 해당합니다.
        front = self.xBuf.get()
        # 이번 스텝에 입력 받은 값을 큐에 넣습니다.
        self.xBuf.put(input)
        
        avg = Point32(self.prevAvg.x + (input.x - front.x) / self.n,self.prevAvg.y + (input.y - front.y) / self.n,0)     
        self.prevAvg = avg
        
        return avg