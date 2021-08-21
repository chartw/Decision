from numpy import argmax, array
from darknet_ros_msgs.msg import BoundingBoxes

class trafficLight:
    def __init__(self):
        self.lights_count = {'Green':0, 'Red': 0, 'Yellow':0, 'RedLeft':0, 'GreenLeft':0}
        self.light_names = ['Green', 'Red', 'Yellow', 'RedLeft', 'GreenLeft']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=10



    def run(self, boxes):
        # print('========')
        max_size = 0
        max_class = 'Red' #Default
        for box in boxes:
            # print(box)
            if box.Class in list(self.lights_count.keys()):
                x = box.xmax - box.xmin
                y = box.ymax - box.ymin
                ratio = y/x
                if ratio > self.ratio_th:
                    continue # 버스 컷
                

                size = x*y
                if size > max_size:
                    max_size = size
                    max_class = box.Class
        
        self.queue.append(max_class)
        self.lights_count[max_class]+=1
        print(self.lights_count)
        if len(self.queue) >self.queue_size:
            # print('================')
            traffic_name=self.queue.pop(0)
            # print('pop', traffic_name)
            self.lights_count[traffic_name]-=1
            

        max_traffic = max(self.lights_count, key=self.lights_count.get)

        print(self.lights_count.values())
        print(max_traffic)
        # return self.light_names[max_traffic]
        return max_traffic